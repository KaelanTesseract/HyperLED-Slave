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
    // Effect parameters for a Slave that renders locally. Payload (17 bytes):
    //   [effect][brightness][speed][intensity][palette][flags]
    //   [r][g][b] [r2][g2][b2] [cct] [effectStep L][effectStep H] [reserved][reserved]
    // flags: bit0 isOn, bit1 color2Enabled, bit2 whiteOnly.
    //
    // This replaces per-frame pixel streaming for effects the Slave can render itself: a 64x64
    // panel is 4096 pixels per frame, which ESP-NOW cannot carry at a usable rate and which
    // exceeds the UART payload limit outright. These parameters are sent only when something
    // changes, plus a periodic refresh so a restarted Slave recovers on its own.
    CMD_SET_SEGMENT = 0x08
};

// Length of a CMD_SET_SEGMENT payload. Kept as a constant so both sides agree.
#define HYPERBUS_SEGMENT_PAYLOAD_LEN 17

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
