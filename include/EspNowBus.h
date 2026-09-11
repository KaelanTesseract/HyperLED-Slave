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
#ifndef ESPNOWBUS_H
#define ESPNOWBUS_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "HyperBus.h" // For BusInterface and HyperBusPacket
#include <map>
#include <array>

class EspNowBusClass : public BusInterface {
    // How long the Master may stay silent before the Slave gives up its channel lock. Kept in
    // step with the Master's own 15s slave timeout (SlaveManager) so neither side drops the
    // other first over a momentary stall.
    static const unsigned long MASTER_LOST_TIMEOUT_MS = 15000;

public:
    EspNowBusClass();
    
    // mode: WIFI_STA, WIFI_AP, or WIFI_AP_STA
    // autoHop: if true, cycles channels if no PING received (for Slaves)
    void begin(wifi_mode_t mode, bool autoHop = false);
    void end();
    
    virtual void loop() override;
    virtual bool sendPacket(uint8_t targetId, uint8_t senderId, uint8_t command, const uint8_t* payload, uint16_t length) override;
    virtual void setCallback(PacketReceivedCallback cb) override { _callback = cb; }

    static EspNowBusClass* _instance;
    static void onDataRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t *incomingData, int len);

private:
    PacketReceivedCallback _callback = nullptr;
    bool _autoHop = false;
    unsigned long _lastPingReceived = 0;
    uint8_t _currentChannel = 1;
    bool _locked = false;
    // Scan diagnostics (see loop()).
    uint32_t _packetsReceived = 0;
    uint32_t _channelErrors = 0;
    uint32_t _droppedForeign = 0;
    bool _reportedLocked = false;
    uint8_t _reportedChannel = 0;
    
    uint8_t _broadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    std::map<uint8_t, std::array<uint8_t, 6>> _peerMacs;

    static const uint16_t TX_BUFFER_SIZE = 250; // Covers header(6) + max chunk(240) with headroom, ESP-NOW payload cap is 250
    uint8_t _txBuffer[TX_BUFFER_SIZE];
};

#endif

