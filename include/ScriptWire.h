/*
 * HyperLED - Open Source LED Controller
 *
 * Copyright (c) 2026 Dennis Guse
 *
 * Licensed under the EUPL, Version 1.2 or – as soon they will be approved by
 * the European Commission - subsequent versions of the EUPL (the "Licence");
 * You may not use this work except in compliance with the Licence.
 * You may obtain a copy of the Licence at:
 *
 * https://joinup.ec.europa.eu/software/page/eupl
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the Licence is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the Licence for the specific language governing permissions and
 * limitations under the Licence.
 */
#pragma once

// The bytes of the script commands (see CMD_SET_SCRIPT and the following ones in HyperBus.h).
// Header only, identical in the Master and the Slave repository: both sides use the same code, so
// they cannot drift apart.

#include <Arduino.h>
#include <string.h>
#include <vector>
#include "HyperBus.h"
#include "ScriptHost.h"

namespace Script {
namespace Wire {

constexpr size_t CONFIG_LEN = HYPERBUS_SCRIPT_CONFIG_LEN;
constexpr size_t REQUEST_LEN = HYPERBUS_SCRIPT_REQUEST_LEN;
constexpr size_t CHUNK_HEADER = HYPERBUS_SCRIPT_CHUNK_HEADER;
constexpr size_t CHUNK_DATA = HYPERBUS_SCRIPT_CHUNK_DATA;
constexpr size_t MAX_SCRIPT_BYTES = HYPERBUS_SCRIPT_MAX_BYTES;
constexpr size_t VALUES_MAX = HYPERBUS_SCRIPT_VALUES_MAX;
constexpr size_t STATUS_MAX = HYPERBUS_SCRIPT_STATUS_MAX;
constexpr size_t MESSAGE_MAX = 60;

enum Flag : uint8_t { FLAG_ON = 0x01, FLAG_RELEASE = 0x02 };
enum State : uint8_t { STATE_NONE = 0, STATE_LOADING = 1, STATE_RUNNING = 2, STATE_FAILED = 3 };

inline void put32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
inline uint32_t get32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
inline void put16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
inline uint16_t get16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

// CMD_SET_SCRIPT ------------------------------------------------------------------------------

struct Config {
    uint8_t flags = 0;
    uint8_t brightness = 255;
    uint8_t level = 1;
    uint16_t length = 0;
    uint32_t crc = 0;
};

inline void encodeConfig(const Config& c, uint8_t out[CONFIG_LEN]) {
    out[0] = c.flags;
    out[1] = c.brightness;
    out[2] = c.level;
    put16(&out[3], c.length);
    put32(&out[5], c.crc);
}

inline bool decodeConfig(const uint8_t* p, size_t n, Config& c) {
    if (!p || n < CONFIG_LEN) return false;
    c.flags = p[0];
    c.brightness = p[1];
    c.level = p[2];
    c.length = get16(&p[3]);
    c.crc = get32(&p[5]);
    return c.length <= MAX_SCRIPT_BYTES;
}

// CMD_REQUEST_SCRIPT --------------------------------------------------------------------------

inline void encodeRequest(uint32_t crc, uint8_t out[REQUEST_LEN]) { put32(out, crc); }

inline bool decodeRequest(const uint8_t* p, size_t n, uint32_t& crc) {
    if (!p || n < REQUEST_LEN) return false;
    crc = get32(p);
    return true;
}

// CMD_SCRIPT_CHUNK ----------------------------------------------------------------------------

struct Chunk {
    uint32_t crc = 0;
    uint16_t offset = 0;
    uint16_t total = 0;
    const uint8_t* data = nullptr;  // points into the packet it was decoded from
    size_t len = 0;
};

// Returns the packet length, or 0 when the piece is larger than a chunk may be.
inline size_t encodeChunk(uint32_t crc, uint16_t offset, uint16_t total, const uint8_t* data, size_t len, uint8_t* out) {
    if (len > CHUNK_DATA) return 0;
    put32(out, crc);
    put16(&out[4], offset);
    put16(&out[6], total);
    memcpy(&out[CHUNK_HEADER], data, len);
    return CHUNK_HEADER + len;
}

inline bool decodeChunk(const uint8_t* p, size_t n, Chunk& c) {
    if (!p || n <= CHUNK_HEADER || n > CHUNK_HEADER + CHUNK_DATA) return false;
    c.crc = get32(p);
    c.offset = get16(&p[4]);
    c.total = get16(&p[6]);
    c.data = &p[CHUNK_HEADER];
    c.len = n - CHUNK_HEADER;
    return c.total <= MAX_SCRIPT_BYTES && (size_t)c.offset + c.len <= c.total;
}

// CMD_SET_SCRIPT_VALUES -----------------------------------------------------------------------
// [sequence] [count], then per item: [kind | 0x80 if it is a setting] [nameLen] [name...] and
//   Number: [float, 4 bytes little endian]   Text: [len] [bytes...]   Bool: [0 or 1]
// Names are at most 24 bytes, texts at most 80. Items that do not fit are left out whole.

constexpr uint8_t SETTING_BIT = 0x80;

inline size_t itemSize(const Item& item) {
    size_t base = 2 + item.name.length();
    switch (item.kind) {
        case Item::Number: return base + 4;
        case Item::Text: return base + 1 + (item.text.length() > 80 ? 80 : item.text.length());
        case Item::Bool: return base + 1;
        default: return 0;
    }
}

inline size_t putItem(const Item& item, bool isSetting, uint8_t* out) {
    size_t o = 0;
    out[o++] = (uint8_t)item.kind | (isSetting ? SETTING_BIT : 0);
    size_t nameLen = item.name.length() > 24 ? 24 : item.name.length();
    out[o++] = (uint8_t)nameLen;
    memcpy(&out[o], item.name.c_str(), nameLen);
    o += nameLen;
    if (item.kind == Item::Number) {
        float f = (float)item.number;
        memcpy(&out[o], &f, 4);
        o += 4;
    } else if (item.kind == Item::Text) {
        size_t len = item.text.length() > 80 ? 80 : item.text.length();
        out[o++] = (uint8_t)len;
        memcpy(&out[o], item.text.c_str(), len);
        o += len;
    } else {
        out[o++] = item.number != 0 ? 1 : 0;
    }
    return o;
}

// Returns the packet length. `dropped` counts the non-nil items that did not fit.
inline size_t encodeItems(uint8_t sequence, const std::vector<Item>& settings, const std::vector<Item>& values,
                          uint8_t* out, size_t capacity, size_t& dropped) {
    dropped = 0;
    if (capacity < 2) return 0;
    size_t o = 2;
    uint8_t count = 0;
    auto add = [&](const Item& item, bool isSetting) {
        size_t size = itemSize(item);
        if (size == 0) return;  // nil: not sent
        if (item.name.length() == 0 || o + size > capacity || count == 255) {
            dropped++;
            return;
        }
        o += putItem(item, isSetting, &out[o]);
        count++;
    };
    for (const Item& item : settings) add(item, true);
    for (const Item& item : values) add(item, false);
    out[0] = sequence;
    out[1] = count;
    return o;
}

inline bool decodeItems(const uint8_t* p, size_t n, uint8_t& sequence, std::vector<Item>& settings, std::vector<Item>& values) {
    settings.clear();
    values.clear();
    if (!p || n < 2) return false;
    sequence = p[0];
    uint8_t count = p[1];
    size_t o = 2;
    for (uint8_t i = 0; i < count; i++) {
        if (o + 2 > n) return false;
        uint8_t kindByte = p[o++];
        bool isSetting = (kindByte & SETTING_BIT) != 0;
        uint8_t kind = kindByte & 0x7F;
        size_t nameLen = p[o++];
        if (kind < Item::Number || kind > Item::Bool || nameLen == 0 || nameLen > 24 || o + nameLen > n) return false;
        Item item;
        item.name = String((const char*)&p[o]).substring(0, nameLen);
        item.kind = (Item::Kind)kind;
        o += nameLen;
        if (kind == Item::Number) {
            if (o + 4 > n) return false;
            float f;
            memcpy(&f, &p[o], 4);
            item.number = f;
            o += 4;
        } else if (kind == Item::Text) {
            if (o + 1 > n) return false;
            size_t len = p[o++];
            if (len > 80 || o + len > n) return false;
            char text[81];
            memcpy(text, &p[o], len);
            text[len] = 0;
            item.text = String(text);
            o += len;
        } else {
            if (o + 1 > n) return false;
            item.number = p[o++] != 0 ? 1 : 0;
        }
        (isSetting ? settings : values).push_back(item);
    }
    return true;
}

// CMD_SCRIPT_STATUS ---------------------------------------------------------------------------
// [state] [result] [fps] [frame time L][H, in units of 0.1 ms] [frame crc 4] [memory in KB]
// [message length] [message...]. `frame crc` is the CRC-32 of the last finished frame's pixels
// (r, g, b per pixel, row by row, before dimming): authors and tests can see that a script draws
// what they expect without a camera.

struct Status {
    uint8_t state = STATE_NONE;
    uint8_t result = 0;      // Script::Result of the last failure
    uint8_t fps = 0;
    uint16_t frameUs10 = 0;  // average time of a frame, in 0.1 ms
    uint32_t frameCrc = 0;
    uint8_t memoryKb = 0;
    String message;
};

inline size_t encodeStatus(const Status& s, uint8_t out[STATUS_MAX]) {
    out[0] = s.state;
    out[1] = s.result;
    out[2] = s.fps;
    put16(&out[3], s.frameUs10);
    put32(&out[5], s.frameCrc);
    out[9] = s.memoryKb;
    size_t len = s.message.length() > MESSAGE_MAX ? MESSAGE_MAX : s.message.length();
    out[10] = (uint8_t)len;
    memcpy(&out[11], s.message.c_str(), len);
    return 11 + len;
}

inline bool decodeStatus(const uint8_t* p, size_t n, Status& s) {
    if (!p || n < 11) return false;
    s.state = p[0];
    s.result = p[1];
    s.fps = p[2];
    s.frameUs10 = get16(&p[3]);
    s.frameCrc = get32(&p[5]);
    s.memoryKb = p[9];
    size_t len = p[10];
    if (len > MESSAGE_MAX || 11 + len > n) return false;
    char text[MESSAGE_MAX + 1];
    memcpy(text, &p[11], len);
    text[len] = 0;
    s.message = String(text);
    return true;
}

}  // namespace Wire
}  // namespace Script
