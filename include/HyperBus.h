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
#ifndef HYPERBUS_H
#define HYPERBUS_H

#include <Arduino.h>

#define HYPERBUS_START_BYTE 0xAA
#define HYPERBUS_BROADCAST_ID 255
#define HYPERBUS_MASTER_ID 0

// The ESP-NOW transport needs a marker of its own. The UART framing (0xAA start byte plus CRC16)
// does not apply there, so without this every foreign ESP-NOW frame on the channel is parsed as a
// HyperBus packet. That is not theoretical: a stray frame whose command byte happened to read as
// 0x01 was taken for a PING, which made a Slave lock onto the wrong channel and cache the sender's
// MAC as "the Master" - from then on it received the real Master's pings but unicast every reply
// to a foreign device, so it was never discovered.
#define HYPERBUS_ESPNOW_MAGIC0 0x48  // 'H'
#define HYPERBUS_ESPNOW_MAGIC1 0x4C  // 'L'
#define HYPERBUS_ESPNOW_HEADER 6     // magic(2) + targetId + senderId + command + length

enum HyperBusCommand {
    CMD_PING = 0x01,
    CMD_PONG = 0x02,
    CMD_SET_CONFIG = 0x03,
    CMD_SET_LEDS = 0x04,
    CMD_TRIGGER_UPDATE = 0x05,
    CMD_SET_LEDS_CHUNK = 0x06,
    // Onboard status LED of a Slave. Payload: [on][r][g][b][brightness].
    // Deliberately its own command rather than more fields on CMD_SET_CONFIG: that keeps the
    // config payload stable (a mismatched one silently corrupts the fields behind it) and
    // lets the Master push LED changes on their own. Slaves on older firmware simply ignore
    // an unknown command.
    CMD_SET_STATUS_LED = 0x07,
    // Effect parameters for a Slave that renders locally. Payload (19 bytes):
    //   [effect][brightness][speed][intensity][palette][flags]
    //   [r][g][b] [r2][g2][b2] [cct] [effectStep L][effectStep H]
    //   [windowOffset L][windowOffset H] [windowTotal L][windowTotal H]
    // flags: bit0 isOn, bit1 color2Enabled, bit2 whiteOnly.
    //
    // The window is what makes sync mode work across devices. With sync on, one effect runs
    // across the whole chain, so a Slave must not restart it at its own pixel 0 - it renders the
    // effect over windowTotal pixels and displays only the slice starting at windowOffset. A
    // windowTotal of 0 means no sync: the Slave renders for its own pixel count as usual.
    //
    // This replaces per-frame pixel streaming for effects the Slave can render itself: a 64x64
    // panel is 4096 pixels per frame, which ESP-NOW cannot carry at a usable rate and which
    // exceeds the UART payload limit outright. These parameters are sent only when something
    // changes, plus a periodic refresh so a restarted Slave recovers on its own.
    CMD_SET_SEGMENT = 0x08,
    // "Uhr / Text" elements a Slave draws itself, sent as parameters instead of pixels. Payload
    // (variable length, see HYPERBUS_WIDGET_* below):
    //   [flags: HYPERBUS_WIDGET_FLAG_*] [brightness] [count]
    //   repeated `count` times, layout 1 (flag ALL_TYPES clear, Slaves 0.2.002 and later):
    //     [id] [type] [xL][xH] [yL][yH] [r][g][b] [scale] [format] [font] [speed] [width]
    //     [textLen] [text bytes...]
    //   layout 2 (flag ALL_TYPES set, Slaves 0.2.004 and later):
    //     [id] [type] [xL][xH] [yL][yH] [r][g][b] [scale] [format] [font] [speed] [width]
    //     [height] [crc0][crc1][crc2][crc3] [textLen] [text bytes...]
    //
    // Layout 1 carries only the custom-text and Lauftext types; a Slave that old cannot draw the
    // rest, so the Master draws those and streams them as pixels next to the Slave's own elements
    // (flag MASTER_LAYER). Layout 2 carries every type - clock, date, analog clock, weather and
    // image as well - and a panel whose elements all fit needs no pixel stream at all. What those
    // types need from the Master travels separately and rarely: the time (CMD_SET_TIME), the
    // weather (CMD_SET_WEATHER) and image pixels (CMD_WIDGET_IMAGE, fetched by the Slave itself
    // with CMD_REQUEST_WIDGET_IMAGE). `crc` identifies an image's pixels (0 = none); the others
    // leave it 0. `width` is the image width, the analog clock's diameter or the Lauftext window;
    // `height` is the image height. Sent on change plus a periodic refresh.
    //
    // With flag ENTRY_BRIGHTNESS the entries are followed by one byte per entry, in entry order:
    // that element's own brightness (255 = as bright as the segment). It sits behind the entries
    // so a Slave older than 0.2.006, which reads only `count` entries, simply ignores it.
    CMD_SET_WIDGETS = 0x09,
    // The Master's local wall-clock time, broadcast every few seconds while a Slave draws time
    // elements. Payload (HYPERBUS_TIME_PAYLOAD_LEN): [flags] [epoch 4 bytes, little endian]
    // [millis into that second, 2 bytes]. `epoch` is the local time expressed as seconds that,
    // read as UTC, give the local calendar fields (see WidgetRender::localEpoch) - the Slave needs
    // no time zone rules of its own. flags bit0: the Master's clock is NTP-synced.
    CMD_SET_TIME = 0x0A,
    // Current weather for weather elements, sent on change plus a periodic refresh.
    // Payload (HYPERBUS_WEATHER_PAYLOAD_LEN): [flags] [temp L] [temp H] [icon]; temp in whole
    // degrees (signed), flags bit0: data available.
    CMD_SET_WEATHER = 0x0B,
    // One piece of an image element's pixels. Payload: [id] [width] [height] [crc 4 bytes]
    // [offset L][offset H] [total L][total H] [RGB bytes...]; offset/total are byte positions in
    // the width * height * 3 byte image. Every piece but the last carries exactly
    // HYPERBUS_IMAGE_CHUNK_DATA bytes. Only sent on request, so a Slave that restarts, or misses a
    // piece, simply asks again.
    CMD_WIDGET_IMAGE = 0x0C,
    // Slave -> Master: "send me image `id` with these pixels". Payload
    // (HYPERBUS_IMAGE_REQUEST_LEN): [id] [crc 4 bytes]. The Master only answers if its image still
    // has that crc; an edited image reaches the Slave through the next CMD_SET_WIDGETS instead.
    CMD_REQUEST_WIDGET_IMAGE = 0x0D
};

// Bytes per pixel in a CMD_SET_LEDS payload (r, g, b, w, w2). CMD_SET_LEDS_CHUNK carries a
// pixel index as its offset, not a byte position - the receiver writes LEDs, so anything else
// puts the chunks in the wrong place.
#define HYPERBUS_LED_BYTES_PER_PIXEL 5

// Length of a CMD_SET_SEGMENT payload. Kept as a constant so both sides agree.
#define HYPERBUS_SEGMENT_PAYLOAD_LEN 19

// Fixed header/per-widget overhead of a CMD_SET_WIDGETS payload, and the hard cap on its total
// size - kept at the ESP-NOW payload limit (see EspNowBus.h) so a widget list that fits never
// needs chunking, on either transport. Whatever doesn't fit is simply left out of the payload
// (see LEDManagerClass::localWidgetMask on the Master) and that element keeps riding the streamed
// frame instead.
#define HYPERBUS_WIDGET_HEADER_LEN 3
#define HYPERBUS_WIDGET_ENTRY_FIXED_LEN 15
#define HYPERBUS_WIDGET_ENTRY_V2_FIXED_LEN 20

// Flag bits in the first byte of a CMD_SET_WIDGETS payload.
//
// MASTER_LAYER decides who owns the pixels outside the widgets. Set, the Master also streams a frame
// for this panel (clock, weather, image): the Slave must never touch anything outside its own widget
// rectangles, and drops streamed pixels that fall inside them - otherwise each side keeps erasing the
// other. Clear, nothing else is drawn on the panel and the Slave owns all of it.
//
// RELEASE ends widget mode (the segment left "Uhr / Text", or joined a sync group). It is best effort;
// a Slave that stops hearing CMD_SET_WIDGETS also gives up on its own after a timeout.
#define HYPERBUS_WIDGET_FLAG_ON           0x01
#define HYPERBUS_WIDGET_FLAG_MASTER_LAYER 0x02
#define HYPERBUS_WIDGET_FLAG_RELEASE      0x04
// Entries use layout 2 and may be of any type (see CMD_SET_WIDGETS). Only ever sent to Slaves that
// understand it: an older Slave would read the longer entries with the old layout.
#define HYPERBUS_WIDGET_FLAG_ALL_TYPES    0x08
// Per-element brightness bytes follow the entries (see CMD_SET_WIDGETS).
#define HYPERBUS_WIDGET_FLAG_ENTRY_BRIGHTNESS 0x10
#define HYPERBUS_WIDGETS_MAX_PAYLOAD 240

#define HYPERBUS_TIME_PAYLOAD_LEN 7
#define HYPERBUS_TIME_FLAG_SYNCED 0x01
#define HYPERBUS_WEATHER_PAYLOAD_LEN 4
#define HYPERBUS_WEATHER_FLAG_VALID 0x01
// A CMD_WIDGET_IMAGE packet is its 11-byte header plus up to this many pixel bytes - together the
// 240-byte ESP-NOW payload limit.
#define HYPERBUS_IMAGE_CHUNK_HEADER 11
#define HYPERBUS_IMAGE_CHUNK_DATA 229
#define HYPERBUS_IMAGE_REQUEST_LEN 5

// Which effects a Slave can render on its own is decided by EffectEngine::canRender() alone.
// This header deliberately does not carry a second copy of that list: it did, and adding effects
// to the engine without updating it made the Master stream pixels for effects the Slave could
// already draw - flooding the link and knocking the Slave offline.

struct HyperBusPacket {
    uint8_t targetId;
    uint8_t senderId;
    uint8_t command;
    uint16_t length;
    uint8_t* payload;
    
    // For internal use
    bool isValid;
    bool isWireless; // Tracks if this packet came via ESP-NOW
};

class BusInterface {
public:
    typedef void (*PacketReceivedCallback)(const HyperBusPacket& packet);
    
    virtual ~BusInterface() = default;
    virtual void loop() = 0;
    virtual bool sendPacket(uint8_t targetId, uint8_t senderId, uint8_t command, const uint8_t* payload, uint16_t length) = 0;
    virtual void setCallback(PacketReceivedCallback cb) = 0;
};

class HyperBusClass : public BusInterface {
public:
    HyperBusClass(HardwareSerial& serial);
    void begin(unsigned long baud, int8_t rxPin, int8_t txPin);
    void loop();
    
    // Send a packet
    bool sendPacket(uint8_t targetId, uint8_t senderId, uint8_t command, const uint8_t* payload, uint16_t length);
    
    // Callbacks
    typedef void (*PacketReceivedCallback)(const HyperBusPacket& packet);
    void setCallback(PacketReceivedCallback cb) { _callback = cb; }

private:
    HardwareSerial& _serial;
    PacketReceivedCallback _callback = nullptr;
    unsigned long _lastReceiveTime = 0;
    
    // Receiver State Machine
    enum ReceiveState { WAIT_START, WAIT_HEADER, WAIT_PAYLOAD, WAIT_CRC };
    ReceiveState _state = WAIT_START;
    
    uint8_t _headerBuffer[5];
    uint8_t _headerIndex = 0;
    
    static const uint16_t MAX_PAYLOAD_SIZE = 1024;
    uint8_t _payloadBuffer[MAX_PAYLOAD_SIZE];
    uint16_t _payloadIndex = 0;

    uint8_t _crcBuffer[2];
    uint8_t _crcIndex = 0;

    uint16_t calculateCRC16(const uint8_t* data, uint16_t length, uint16_t crc = 0xFFFF);
    void resetReceiver();
    void processReceivedPacket();
};

#endif
