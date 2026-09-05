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

#include <Arduino.h>
#include "Config.h"

// Drives the board's onboard WS2812 on its own NeoPixelBus instance / RMT channel, so it
// works regardless of which bus type the Slave's actual LED output uses (or whether one is
// configured at all).
//
// Unlike the Master's version this one is mostly self-driving: setCondition() reports what
// the firmware currently knows about itself, and the LED shows that. The user setting
// (pushed down from the Master via CMD_SET_STATUS_LED) only decides what "everything is
// fine" looks like - problems are always shown, even when the LED is switched off, because
// a Slave that is unreachable or unconfigured is exactly when you need to see it on the
// hardware itself.
enum SlaveLedCondition : uint8_t {
    SLED_OK = 0,           // configured and talking to the Master
    SLED_UNCONFIGURED,     // still ID 254 - waiting to be set up in the Master's WebUI
    SLED_DISCONNECTED      // no packet from the Master for a while
};

class StatusLedManagerClass {
public:
    void begin();
    void loop();

    // User setting for the "everything is fine" case (persisted).
    void setState(bool on, uint32_t color, uint8_t brightness);
    bool isOn() const { return _on; }
    uint32_t getColor() const { return _color; }
    uint8_t getBrightness() const { return _brightness; }

    // Reported by main.cpp whenever the connection/configuration state changes.
    void setCondition(SlaveLedCondition condition);

private:
    bool _on = true;
    uint32_t _color = 0x00FF00; // green = running fine
    uint8_t _brightness = 40;   // these LEDs are glaringly bright at full power

    SlaveLedCondition _condition = SLED_UNCONFIGURED;
    unsigned long _lastBlinkToggle = 0;
    bool _blinkPhaseOn = true;
    bool _dirty = true;

    void render();
    void save();
};

extern StatusLedManagerClass StatusLedManager;
