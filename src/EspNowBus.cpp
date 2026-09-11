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
#include "EspNowBus.h"

EspNowBusClass* EspNowBusClass::_instance = nullptr;

EspNowBusClass::EspNowBusClass() {
    _instance = this;
}

void EspNowBusClass::begin(wifi_mode_t mode, bool autoHop) {
    _autoHop = autoHop;
    
    // Ensure WiFi is in the correct mode
    if (WiFi.getMode() != mode && WiFi.getMode() != WIFI_AP_STA) {
        WiFi.mode(mode);
    }
    
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }
    
    // Register callback
    esp_now_register_recv_cb(EspNowBusClass::onDataRecv);
    
    // Register broadcast peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, _broadcastAddress, 6);
    peerInfo.channel = 0; // Use current channel
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add broadcast peer");
    }
    
    _lastPingReceived = millis();
}

void EspNowBusClass::end() {
    esp_now_unregister_recv_cb();
    esp_now_deinit();
}

void EspNowBusClass::loop() {
    if (!_autoHop) return;

    // Log only on a state change, never periodically: this board's native USB-CDC can block the
    // firmware when nothing is reading the port, so a recurring print is a liability. Transitions
    // are what matter anyway - which channel we settled on, and when the Master went quiet.
    if (_locked != _reportedLocked || (_locked && _currentChannel != _reportedChannel)) {
        _reportedLocked = _locked;
        _reportedChannel = _currentChannel;
        Serial.printf("EspNowBus: %s, channel %d (received=%lu, foreign=%lu)\n",
                      _locked ? "locked to the Master" : "scanning", _currentChannel,
                      (unsigned long)_packetsReceived, (unsigned long)_droppedForeign);
    }

    if (_locked) {
        // Locking onto the Master's channel is not permanent: if the Master reboots and
        // rejoins its router on a different channel, a permanently locked Slave would keep
        // listening on the old one and never be seen again until someone power-cycles it.
        // The Master pings every 250ms, so several seconds of silence means the channel is
        // stale - go back to scanning.
        if (millis() - _lastPingReceived > 5000) {
            Serial.println("EspNowBus: lost the Master, scanning channels again");
            _locked = false;
            _lastPingReceived = millis();
        }
        return;
    }

    // If we haven't received a PING in 500ms, switch channel
    if (millis() - _lastPingReceived > 500) {
        _currentChannel++;
        if (_currentChannel > 13) _currentChannel = 1;

        esp_err_t chErr = esp_wifi_set_channel(_currentChannel, WIFI_SECOND_CHAN_NONE);
        if (chErr != ESP_OK) _channelErrors++;
        _lastPingReceived = millis(); // Reset timer for next hop
    }
}

bool EspNowBusClass::sendPacket(uint8_t targetId, uint8_t senderId, uint8_t command, const uint8_t* payload, uint16_t length) {
    uint8_t* targetMac = _broadcastAddress;
    if (targetId != HYPERBUS_BROADCAST_ID && targetId != 254 && _peerMacs.find(targetId) != _peerMacs.end()) {
        targetMac = _peerMacs[targetId].data();
    }
    if (length <= 240) {
        // Fits in a single packet
        uint16_t packetLen = HYPERBUS_ESPNOW_HEADER + length;
        _txBuffer[0] = HYPERBUS_ESPNOW_MAGIC0;
        _txBuffer[1] = HYPERBUS_ESPNOW_MAGIC1;
        _txBuffer[2] = targetId;
        _txBuffer[3] = senderId;
        _txBuffer[4] = command;
        _txBuffer[5] = (uint8_t)length; // max 240, fits in 1 byte for espnow wrappers

        if (length > 0 && payload != nullptr) {
            memcpy(&_txBuffer[HYPERBUS_ESPNOW_HEADER], payload, length);
        }

        esp_err_t result = esp_now_send(targetMac, _txBuffer, packetLen);
        return (result == ESP_OK);
    } else if (command == CMD_SET_LEDS) {
        // Chunking for CMD_SET_LEDS
        uint16_t offset = 0;
        while (offset < length) {
            uint16_t chunkSize = length - offset;
            if (chunkSize > 240) chunkSize = 240; // 240 is exactly 48 pixels, so never mid-pixel

            // The offset travels as a PIXEL index, because that is what the receiver writes.
            // It used to be sent as the byte position, so every chunk after the first landed five
            // times too far into the strip and everything past a fifth of the way was dropped as
            // out of range - a 64x64 panel came out scrambled, while segments small enough to fit
            // in a single chunk were unaffected and hid the problem.
            uint16_t pixelOffset = offset / HYPERBUS_LED_BYTES_PER_PIXEL;

            uint16_t packetLen = HYPERBUS_ESPNOW_HEADER + 2 + chunkSize; // + offsetL, offsetH
            _txBuffer[0] = HYPERBUS_ESPNOW_MAGIC0;
            _txBuffer[1] = HYPERBUS_ESPNOW_MAGIC1;
            _txBuffer[2] = targetId;
            _txBuffer[3] = senderId;
            _txBuffer[4] = CMD_SET_LEDS_CHUNK;
            _txBuffer[5] = (uint8_t)(chunkSize + 2); // payload length includes the offset field
            _txBuffer[6] = pixelOffset & 0xFF;
            _txBuffer[7] = (pixelOffset >> 8) & 0xFF;

            memcpy(&_txBuffer[HYPERBUS_ESPNOW_HEADER + 2], &payload[offset], chunkSize);

            esp_now_send(targetMac, _txBuffer, packetLen);

            offset += chunkSize;
            delayMicroseconds(500); // Give ESP-NOW some breathing room
        }
        return true;
    }

    Serial.printf("EspNowBus: dropped %u-byte packet for cmd 0x%02X (target %u) - exceeds the 240-byte single-packet limit and chunking is only implemented for CMD_SET_LEDS\n", length, command, targetId);
    return false;
}

void EspNowBusClass::onDataRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len) {
    if (!_instance || len < HYPERBUS_ESPNOW_HEADER) return;
    if (incomingData[0] != HYPERBUS_ESPNOW_MAGIC0 || incomingData[1] != HYPERBUS_ESPNOW_MAGIC1) {
        // Not ours - some other ESP-NOW device sharing the channel. See HyperBus.h.
        _instance->_droppedForeign++;
        return;
    }
    if (!_instance->_callback) return;
    _instance->_packetsReceived++;
    
    HyperBusPacket packet;
    packet.targetId = incomingData[2];
    packet.senderId = incomingData[3];
    packet.command = incomingData[4];
    uint8_t payloadLen = incomingData[5];

    // Register the sender's MAC so replies can go out as unicast. A known ID whose MAC has
    // changed is re-learned rather than kept: an entry that could never be corrected meant a
    // Slave that once cached the wrong address kept unicasting into the void until it was
    // power-cycled, which is what made this fault look permanent.
    std::array<uint8_t, 6> mac;
    memcpy(mac.data(), esp_now_info->src_addr, 6);
    auto known = _instance->_peerMacs.find(packet.senderId);
    if (known == _instance->_peerMacs.end() || known->second != mac) {
        _instance->_peerMacs[packet.senderId] = mac;

        // Add peer to ESP-NOW
        if (!esp_now_is_peer_exist(mac.data())) {
            esp_now_peer_info_t peerInfo;
            memset(&peerInfo, 0, sizeof(peerInfo));
            memcpy(peerInfo.peer_addr, mac.data(), 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            esp_now_add_peer(&peerInfo);
        }
    }

    if (len < HYPERBUS_ESPNOW_HEADER + payloadLen) return; // Incomplete
    
    packet.length = payloadLen;
    packet.isValid = true;
    packet.isWireless = true;
    
    if (payloadLen > 0) {
        // If it's a chunk, we need to pass the offset to the payload so it can be handled
        // Actually, for CMD_SET_LEDS_CHUNK, offset is part of the first 2 bytes of payload!
        // We just pass it as is.
        packet.payload = (uint8_t*)&incomingData[HYPERBUS_ESPNOW_HEADER];
    } else {
        packet.payload = nullptr;
    }
    
    if (packet.command == CMD_PING && _instance->_autoHop) {
        // Lock onto the channel the Master names in the PING, not the one we happen to be
        // listening on. 2.4GHz channels overlap heavily, so a PING sent on e.g. channel 6 is
        // still received while we scan channel 5. Locking onto 5 then looks like success -
        // packets keep arriving - but our PONG goes out on 5 and never reaches a Master
        // listening on 6, so we receive everything and stay invisible. Older Masters send no
        // payload; fall back to the old behaviour for those.
        if (payloadLen >= 1) {
            uint8_t masterChannel = incomingData[HYPERBUS_ESPNOW_HEADER];
            if (masterChannel >= 1 && masterChannel <= 13 && masterChannel != _instance->_currentChannel) {
                _instance->_currentChannel = masterChannel;
                esp_wifi_set_channel(masterChannel, WIFI_SECOND_CHAN_NONE);
            }
        }
        _instance->_locked = true;
        _instance->_lastPingReceived = millis();
    }

    _instance->_callback(packet);
}

