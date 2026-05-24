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
#include "HyperBus.h"

HyperBusClass::HyperBusClass(HardwareSerial& serial) : _serial(serial) {
}

void HyperBusClass::begin(unsigned long baud, int8_t rxPin, int8_t txPin) {
    _serial.end(); // Force hardware reset if already initialized
    _serial.begin(baud, SERIAL_8N1, rxPin, txPin);
    resetReceiver();
}

uint16_t HyperBusClass::calculateCRC16(const uint8_t* data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)data[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

bool HyperBusClass::sendPacket(uint8_t targetId, uint8_t senderId, uint8_t command, const uint8_t* payload, uint16_t length) {
    uint8_t header[6];
    header[0] = HYPERBUS_START_BYTE;
    header[1] = targetId;
    header[2] = senderId;
    header[3] = command;
    header[4] = length & 0xFF;
    header[5] = (length >> 8) & 0xFF;
    
    // Calculate CRC over Header (excluding Start Byte) + Payload
    uint16_t crc = 0xFFFF;
    // CRC for Header
    for(int i = 1; i < 6; i++) {
        crc ^= (uint16_t)header[i];
        for (int j = 8; j != 0; j--) {
            if ((crc & 0x0001) != 0) { crc >>= 1; crc ^= 0xA001; } else { crc >>= 1; }
        }
    }
    // CRC for Payload
    if (payload && length > 0) {
        for(uint16_t i = 0; i < length; i++) {
            crc ^= (uint16_t)payload[i];
            for (int j = 8; j != 0; j--) {
                if ((crc & 0x0001) != 0) { crc >>= 1; crc ^= 0xA001; } else { crc >>= 1; }
            }
        }
    }
    
    _serial.write(header, 6);
    if (payload && length > 0) {
        _serial.write(payload, length);
    }
    uint8_t crcBytes[2] = { (uint8_t)(crc & 0xFF), (uint8_t)((crc >> 8) & 0xFF) };
    _serial.write(crcBytes, 2);
    _serial.flush();
    
    return true;
}

void HyperBusClass::resetReceiver() {
    _state = WAIT_START;
    _headerIndex = 0;
    _payloadIndex = 0;
    _crcIndex = 0;
    if (_payloadBuffer) {
        free(_payloadBuffer);
        _payloadBuffer = nullptr;
    }
}

void HyperBusClass::loop() {
    if (_state != WAIT_START && millis() - _lastReceiveTime > 50) {
        resetReceiver();
    }

    int maxBytesPerCycle = 1024;
    int bytesRead = 0;
    
    while (_serial.available() > 0 && bytesRead < maxBytesPerCycle) {
        bytesRead++;
        _lastReceiveTime = millis();
        uint8_t b = _serial.read();
        
        switch(_state) {
            case WAIT_START:
                if (b == HYPERBUS_START_BYTE) {
                    _state = WAIT_HEADER;
                    _headerIndex = 0;
                }
                break;
                
            case WAIT_HEADER:
                _headerBuffer[_headerIndex++] = b;
                if (_headerIndex == 5) {
                    uint16_t len = _headerBuffer[3] | (_headerBuffer[4] << 8);
                    if (len > 1024) { // Strict sanity limit
                        resetReceiver();
                    } else if (len == 0) {
                        _state = WAIT_CRC;
                        _crcIndex = 0;
                    } else {
                        _payloadBuffer = (uint8_t*)malloc(len);
                        if (!_payloadBuffer) {
                            resetReceiver(); // OOM
                        } else {
                            _payloadIndex = 0;
                            _state = WAIT_PAYLOAD;
                        }
                    }
                }
                break;
                
            case WAIT_PAYLOAD:
                _payloadBuffer[_payloadIndex++] = b;
                uint16_t len;
                len = _headerBuffer[3] | (_headerBuffer[4] << 8);
                if (_payloadIndex == len) {
                    _state = WAIT_CRC;
                    _crcIndex = 0;
                }
                break;
                
            case WAIT_CRC:
                _crcBuffer[_crcIndex++] = b;
                if (_crcIndex == 2) {
                    processReceivedPacket();
                    resetReceiver();
                }
                break;
        }
    }
}

void HyperBusClass::processReceivedPacket() {
    uint16_t length = _headerBuffer[3] | (_headerBuffer[4] << 8);
    uint16_t receivedCrc = _crcBuffer[0] | (_crcBuffer[1] << 8);
    
    // Calculate expected CRC
    uint16_t crc = 0xFFFF;
    for(int i = 0; i < 5; i++) {
        crc ^= (uint16_t)_headerBuffer[i];
        for (int j = 8; j != 0; j--) {
            if ((crc & 0x0001) != 0) { crc >>= 1; crc ^= 0xA001; } else { crc >>= 1; }
        }
    }
    if (_payloadBuffer && length > 0) {
        for(uint16_t i = 0; i < length; i++) {
            crc ^= (uint16_t)_payloadBuffer[i];
            for (int j = 8; j != 0; j--) {
                if ((crc & 0x0001) != 0) { crc >>= 1; crc ^= 0xA001; } else { crc >>= 1; }
            }
        }
    }
    
    if (crc == receivedCrc) {
        if (_callback) {
            HyperBusPacket packet;
            packet.targetId = _headerBuffer[0];
            packet.isWireless = false;
            packet.senderId = _headerBuffer[1];
            packet.command = _headerBuffer[2];
            packet.length = length;
            packet.payload = _payloadBuffer;
            packet.isValid = true;
            _callback(packet);
        }
    } else {
        while (_serial.available() > 0) _serial.read();
    }
}



