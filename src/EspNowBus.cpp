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
    if (_autoHop && !_locked) {
        // If we haven't received a PING in 500ms, switch channel
        if (millis() - _lastPingReceived > 500) {
            _currentChannel++;
            if (_currentChannel > 13) _currentChannel = 1;

            esp_wifi_set_channel(_currentChannel, WIFI_SECOND_CHAN_NONE);
            _lastPingReceived = millis(); // Reset timer for next hop
        }
    }
}

bool EspNowBusClass::sendPacket(uint8_t targetId, uint8_t senderId, uint8_t command, const uint8_t* payload, uint16_t length) {
    uint8_t* targetMac = _broadcastAddress;
    if (targetId != HYPERBUS_BROADCAST_ID && targetId != 254 && _peerMacs.find(targetId) != _peerMacs.end()) {
        targetMac = _peerMacs[targetId].data();
    }
    if (length <= 240) {
        // Fits in a single packet
        uint16_t packetLen = 4 + length;
        _txBuffer[0] = targetId;
        _txBuffer[1] = senderId;
        _txBuffer[2] = command;
        _txBuffer[3] = (uint8_t)length; // max 240, fits in 1 byte for espnow wrappers

        if (length > 0 && payload != nullptr) {
            memcpy(&_txBuffer[4], payload, length);
        }

        esp_err_t result = esp_now_send(targetMac, _txBuffer, packetLen);
        return (result == ESP_OK);
    } else if (command == CMD_SET_LEDS) {
        // Chunking for CMD_SET_LEDS
        uint16_t offset = 0;
        while (offset < length) {
            uint16_t chunkSize = length - offset;
            if (chunkSize > 240) chunkSize = 240;

            uint16_t packetLen = 6 + chunkSize; // target, sender, cmd, len, offsetL, offsetH
            _txBuffer[0] = targetId;
            _txBuffer[1] = senderId;
            _txBuffer[2] = CMD_SET_LEDS_CHUNK;
            _txBuffer[3] = (uint8_t)chunkSize;
            _txBuffer[4] = offset & 0xFF;
            _txBuffer[5] = (offset >> 8) & 0xFF;

            memcpy(&_txBuffer[6], &payload[offset], chunkSize);

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
    if (!_instance || !_instance->_callback || len < 4) return;
    
    HyperBusPacket packet;
    packet.targetId = incomingData[0];
    packet.senderId = incomingData[1];
    packet.command = incomingData[2];
    uint8_t payloadLen = incomingData[3];

    // Register sender MAC for Unicast if we haven't already
    if (_instance->_peerMacs.find(packet.senderId) == _instance->_peerMacs.end()) {
        std::array<uint8_t, 6> mac;
        memcpy(mac.data(), esp_now_info->src_addr, 6);
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

    if (len < 4 + payloadLen) return; // Incomplete
    
    packet.length = payloadLen;
    packet.isValid = true;
    packet.isWireless = true;
    
    if (payloadLen > 0) {
        // If it's a chunk, we need to pass the offset to the payload so it can be handled
        // Actually, for CMD_SET_LEDS_CHUNK, offset is part of the first 2 bytes of payload!
        // We just pass it as is.
        packet.payload = (uint8_t*)&incomingData[4];
    } else {
        packet.payload = nullptr;
    }
    
    if (packet.command == CMD_PING && _instance->_autoHop) {
        _instance->_locked = true;
        _instance->_lastPingReceived = millis();
    }

    _instance->_callback(packet);
}

