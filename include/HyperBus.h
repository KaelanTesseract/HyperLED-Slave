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

enum HyperBusCommand {
    CMD_PING = 0x01,
    CMD_PONG = 0x02,
    CMD_SET_CONFIG = 0x03,
    CMD_SET_LEDS = 0x04,
    CMD_TRIGGER_UPDATE = 0x05,
    CMD_SET_LEDS_CHUNK = 0x06
};

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
    
    uint8_t* _payloadBuffer = nullptr;
    uint16_t _payloadIndex = 0;
    
    uint8_t _crcBuffer[2];
    uint8_t _crcIndex = 0;
    
    uint16_t calculateCRC16(const uint8_t* data, uint16_t length);
    void resetReceiver();
    void processReceivedPacket();
};

#endif
