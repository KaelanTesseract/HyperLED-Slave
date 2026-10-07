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

// A NeoPixelBus "method" for the ESP32-C6.
//
// NeoPixelBus (2.8.x) has no RMT driver for the C6: NeoMethods.h leaves RMT, I2S and DMA-SPI out for
// C6 and H2, and what remains, bit banging, switches the interrupts off on a single core and with them
// the radio. This class does the one thing a method does - send the bytes that the colour feature has
// laid out in its buffer - through the RMT functions of the Arduino core (esp32-hal-rmt.h). The
// features (NeoGrbFeature, NeoWrgbTm1814Feature, NeoGrbTm1914Feature, NeoGrbwcFeature ...) stay as they
// are, so every one-wire LED type works as on the ESP32-S3.
//
// The bit timings are those NeoPixelBus uses for its own RMT methods on the other chips
// (NeoEsp32RmtSpeedWs2812x and NeoEsp32RmtSpeed400Kbps), in 50 ns ticks.

#if defined(CONFIG_IDF_TARGET_ESP32C6)

#include <Arduino.h>
#include <NeoPixelBus.h>

class NeoC6RmtSpeed800 {
public:
    static const uint16_t T0H = 8, T0L = 17;    // 400 ns, 850 ns
    static const uint16_t T1H = 16, T1L = 9;    // 800 ns, 450 ns
    static const uint16_t Reset = 6000;         // 300 us
};

class NeoC6RmtSpeed400 {
public:
    static const uint16_t T0H = 16, T0L = 34;   // 800 ns, 1700 ns
    static const uint16_t T1H = 32, T1L = 18;   // 1600 ns, 900 ns
    static const uint16_t Reset = 1000;         // 50 us
};

template<typename T_SPEED> class NeoC6RmtMethod {
public:
    typedef NeoNoSettings SettingsObject;

    NeoC6RmtMethod(uint8_t pin, uint16_t pixelCount, size_t elementSize, size_t settingsSize)
        : _sizeData(pixelCount * elementSize + settingsSize), _pin(pin) {
        _data = static_cast<uint8_t*>(calloc(_sizeData ? _sizeData : 1, 1));
        _symbolCount = _sizeData * 8 + 1;
        _symbols = static_cast<rmt_data_t*>(malloc(_symbolCount * sizeof(rmt_data_t)));
    }

    ~NeoC6RmtMethod() {
        if (_ready) rmtDeinit(_pin);
        free(_data);
        free(_symbols);
    }

    bool IsReadyToUpdate() const { return true; }  // Update() returns when the frame is out

    void Initialize() {
        if (!_data || !_symbols || _ready) return;
        _ready = rmtInit(_pin, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 20000000);  // 50 ns per tick
        if (_ready) rmtSetEOT(_pin, 0);
    }

    void Update(bool) {
        if (!_ready) return;
        size_t n = 0;
        for (size_t i = 0; i < _sizeData; i++) {
            const uint8_t v = _data[i];
            for (int bit = 7; bit >= 0; bit--) {
                const bool one = (v >> bit) & 1;
                _symbols[n].level0 = 1;
                _symbols[n].duration0 = one ? T_SPEED::T1H : T_SPEED::T0H;
                _symbols[n].level1 = 0;
                _symbols[n].duration1 = one ? T_SPEED::T1L : T_SPEED::T0L;
                n++;
            }
        }
        _symbols[n].level0 = 0;               // the latch: low, then the end marker
        _symbols[n].duration0 = T_SPEED::Reset;
        _symbols[n].level1 = 0;
        _symbols[n].duration1 = 0;
        n++;
        rmtWrite(_pin, _symbols, n, 1000);
    }

    bool AlwaysUpdate() { return false; }
    bool SwapBuffers() { return false; }
    uint8_t* getData() const { return _data; }
    size_t getDataSize() const { return _sizeData; }
    void applySettings(const SettingsObject&) {}

private:
    const size_t _sizeData;
    const uint8_t _pin;
    uint8_t* _data = nullptr;
    rmt_data_t* _symbols = nullptr;
    size_t _symbolCount = 0;
    bool _ready = false;
};

typedef NeoC6RmtMethod<NeoC6RmtSpeed800> NeoC6Rmt800KbpsMethod;
typedef NeoC6RmtMethod<NeoC6RmtSpeed400> NeoC6Rmt400KbpsMethod;

// What the strips in main.cpp are built with: the RMT method above on the C6, NeoPixelBus' own on the S3.
typedef NeoC6Rmt800KbpsMethod HyperNeo800Method;
typedef NeoC6Rmt400KbpsMethod HyperNeo400Method;

#else

#include <NeoPixelBus.h>

typedef Neo800KbpsMethod HyperNeo800Method;
typedef Neo400KbpsMethod HyperNeo400Method;

#endif
