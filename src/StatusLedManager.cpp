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
#include "StatusLedManager.h"
#include <NeoPixelBus.h>
#include <Preferences.h>

StatusLedManagerClass StatusLedManager;

// RMT channel 0 - the main strip bus uses Neo800KbpsMethod, which maps to channel 1 on this
// chip, so the two never fight over the same channel.
//
// NeoRgbFeature, not the usual NeoGrbFeature: the onboard LED of this board expects plain
// RGB byte order. With GRB the red and green channels come out swapped (blue stays correct),
// which is exactly how a wrong order shows up here - so don't "fix" this back to GRB.
static NeoPixelBus<NeoRgbFeature, NeoEsp32Rmt0Ws2812xMethod>* _statusStrip = nullptr;

static const uint32_t COLOR_UNCONFIGURED = 0x0000FF; // blue
static const uint32_t COLOR_DISCONNECTED = 0xFF0000; // red
static const uint16_t BLINK_MS = 500;
static const uint8_t PROBLEM_BRIGHTNESS = 60;        // fixed, so a problem stays readable

void StatusLedManagerClass::begin() {
    Preferences prefs;
    prefs.begin("hyperled_slave", true);
    _on = prefs.getBool(PREF_STATUSLED_ON, true);
    _color = prefs.getUInt(PREF_STATUSLED_COLOR, 0x00FF00);
    _brightness = prefs.getUChar(PREF_STATUSLED_BRI, 40);
    prefs.end();

    _statusStrip = new NeoPixelBus<NeoRgbFeature, NeoEsp32Rmt0Ws2812xMethod>(1, STATUS_LED_PIN);
    _statusStrip->Begin();
    _dirty = true;
    render();
}

void StatusLedManagerClass::loop() {
    if (_condition != SLED_OK) {
        if (millis() - _lastBlinkToggle >= BLINK_MS) {
            _lastBlinkToggle = millis();
            _blinkPhaseOn = !_blinkPhaseOn;
            _dirty = true;
        }
    }
    if (_dirty) render();
}

void StatusLedManagerClass::render() {
    if (!_statusStrip) return;
    _dirty = false;

    uint32_t color = 0;
    uint8_t bri = _brightness;

    if (_condition != SLED_OK) {
        // Problem states blink and ignore the user's on/off and brightness setting - the
        // whole point is that they are visible on the board without the WebUI.
        color = _blinkPhaseOn
                    ? (_condition == SLED_DISCONNECTED ? COLOR_DISCONNECTED : COLOR_UNCONFIGURED)
                    : 0;
        bri = PROBLEM_BRIGHTNESS;
    } else if (_on) {
        color = _color;
    }

    uint8_t r = ((color >> 16) & 0xFF) * bri / 255;
    uint8_t g = ((color >> 8) & 0xFF) * bri / 255;
    uint8_t b = (color & 0xFF) * bri / 255;

    _statusStrip->SetPixelColor(0, RgbColor(r, g, b));
    _statusStrip->Show();
}

void StatusLedManagerClass::setState(bool on, uint32_t color, uint8_t brightness) {
    _on = on;
    _color = color;
    _brightness = brightness;
    _dirty = true;
    save();
}

void StatusLedManagerClass::save() {
    Preferences prefs;
    prefs.begin("hyperled_slave", false);
    prefs.putBool(PREF_STATUSLED_ON, _on);
    prefs.putUInt(PREF_STATUSLED_COLOR, _color);
    prefs.putUChar(PREF_STATUSLED_BRI, _brightness);
    prefs.end();
}

void StatusLedManagerClass::setCondition(SlaveLedCondition condition) {
    if (_condition == condition) return;
    _condition = condition;
    _blinkPhaseOn = true;
    _lastBlinkToggle = millis();
    _dirty = true;
}
