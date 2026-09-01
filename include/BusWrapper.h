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
#include <NeoPixelBus.h>
#include <hub75.h>
#include <esp_heap_caps.h>
#include "Config.h"

class IBus {
public:
    virtual ~IBus() {}
    virtual void Begin() = 0;
    virtual void Show() = 0;
    virtual void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t w2 = 0) = 0;
    virtual const uint8_t* getBuffer() const { return nullptr; }
};

// 1-Wire Digital RGB
template<typename T_FEATURE, typename T_METHOD>
class BusDigitalRgb : public IBus {
private:
    NeoPixelBus<T_FEATURE, T_METHOD>* _bus;
    uint8_t _grouping;
public:
    BusDigitalRgb(uint16_t count, uint8_t pinData, uint8_t grouping = 1) : _grouping(grouping) {
        uint16_t busCount = count / grouping; if(busCount==0) busCount=1;
        _bus = new NeoPixelBus<T_FEATURE, T_METHOD>(busCount, pinData);
    }
    ~BusDigitalRgb() { delete _bus; }
    void Begin() override { _bus->Begin(); }
    void Show() override { _bus->Show(); }
    void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t w2) override {
        _bus->SetPixelColor(index, RgbColor(r, g, b));
    }
};

// 1-Wire Digital RGBW
template<typename T_FEATURE, typename T_METHOD>
class BusDigitalRgbw : public IBus {
private:
    NeoPixelBus<T_FEATURE, T_METHOD>* _bus;
    uint8_t _grouping;
public:
    BusDigitalRgbw(uint16_t count, uint8_t pinData, uint8_t grouping = 1) : _grouping(grouping) {
        uint16_t busCount = count / grouping; if(busCount==0) busCount=1;
        _bus = new NeoPixelBus<T_FEATURE, T_METHOD>(busCount, pinData);
    }
    ~BusDigitalRgbw() { delete _bus; }
    void Begin() override { _bus->Begin(); }
    void Show() override { _bus->Show(); }
    void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t w2) override {
        _bus->SetPixelColor(index, RgbwColor(r, g, b, w));
    }
};

// 2-Wire SPI Digital RGB
template<typename T_FEATURE, typename T_METHOD>
class BusDigitalRgbww : public IBus {
private:
    NeoPixelBus<T_FEATURE, T_METHOD>* _bus;
    uint8_t _grouping;
public:
    BusDigitalRgbww(uint16_t count, uint8_t pinData, uint8_t grouping = 1) : _grouping(grouping) {
        uint16_t busCount = count / grouping; if(busCount==0) busCount=1;
        _bus = new NeoPixelBus<T_FEATURE, T_METHOD>(busCount, pinData);
    }
    ~BusDigitalRgbww() { delete _bus; }
    void Begin() override { _bus->Begin(); }
    void Show() override { _bus->Show(); }
    void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t w2) override {
        _bus->SetPixelColor(index, RgbwwColor(r, g, b, w, w2));
    }
};

// 2-Wire SPI Digital RGB
template<typename T_FEATURE, typename T_METHOD>
class BusDigitalSpiRgb : public IBus {
private:
    NeoPixelBus<T_FEATURE, T_METHOD>* _bus;
    uint8_t _grouping;
public:
    BusDigitalSpiRgb(uint16_t count, uint8_t pinClock, uint8_t pinData, uint8_t grouping = 1) : _grouping(grouping) {
        uint16_t busCount = count / grouping; if(busCount==0) busCount=1;
        _bus = new NeoPixelBus<T_FEATURE, T_METHOD>(busCount, pinClock, pinData);
    }
    ~BusDigitalSpiRgb() { delete _bus; }
    void Begin() override { _bus->Begin(); }
    void Show() override { _bus->Show(); }
    void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t w2) override {
        _bus->SetPixelColor(index, RgbColor(r, g, b));
    }
};

// On/Off (Relay or simple digital out)
class BusOnOff : public IBus {
private:
    uint16_t _count;
    uint8_t _pin;
public:
    BusOnOff(uint16_t count, uint8_t pin) { _count = count; _pin = pin; }
    void Begin() override { if(_pin<255) { pinMode(_pin, OUTPUT); digitalWrite(_pin, LOW); } }
    void Show() override {}
    void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t w2) override {
        if(index == 0 && _pin < 255) {
            bool on = (r>0 || g>0 || b>0 || w>0);
            digitalWrite(_pin, on ? HIGH : LOW);
        }
    }
};

// PWM Analog
class BusPwm : public IBus {
private:
    uint16_t _count;
    uint8_t _pins[5];
    uint8_t _numPins;
public:
    BusPwm(uint16_t count, uint8_t numPins, uint8_t p0, uint8_t p1=255, uint8_t p2=255, uint8_t p3=255, uint8_t p4=255) {
        _count = count;
        _numPins = numPins;
        _pins[0] = p0; _pins[1] = p1; _pins[2] = p2; _pins[3] = p3; _pins[4] = p4;
    }
    
    void Begin() override {
        for(int i=0; i<_numPins; i++) {
            if(_pins[i] < 255) {
                pinMode(_pins[i], OUTPUT);
                analogWrite(_pins[i], 0);
            }
        }
    }
    
    void Show() override {}
    
    void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t w2) override {
        // In WLED, analog strips mirror the first pixel to the whole strip usually
        if (index == 0) {
            if(_numPins == 1) { // White
                uint8_t bri = w > 0 ? w : max(r, max(g,b));
                if(_pins[0] < 255) analogWrite(_pins[0], bri);
            } 
            else if(_numPins == 2) { // CCT
                if(_pins[0] < 255) analogWrite(_pins[0], r); // Warm
                if(_pins[1] < 255) analogWrite(_pins[1], b); // Cold
            } 
            else if(_numPins == 3) { // RGB
                if(_pins[0] < 255) analogWrite(_pins[0], r);
                if(_pins[1] < 255) analogWrite(_pins[1], g);
                if(_pins[2] < 255) analogWrite(_pins[2], b);
            } 
            else if(_numPins == 4) { // RGBW
                if(_pins[0] < 255) analogWrite(_pins[0], r);
                if(_pins[1] < 255) analogWrite(_pins[1], g);
                if(_pins[2] < 255) analogWrite(_pins[2], b);
                if(_pins[3] < 255) analogWrite(_pins[3], w);
            } 
            else if(_numPins == 5) { // RGB+CCT
                if(_pins[0] < 255) analogWrite(_pins[0], r);
                if(_pins[1] < 255) analogWrite(_pins[1], g);
                if(_pins[2] < 255) analogWrite(_pins[2], b);
                if(_pins[3] < 255) analogWrite(_pins[3], w); // warm
                if(_pins[4] < 255) analogWrite(_pins[4], 0); // cold (simplified)
            }
        }
    }
};

// HUB75 scan-matrix panel - identical to the Master's BusHub75 (BusWrapper.h),
// with the fixed pins coming from this project's own Config.h.
class BusHub75 : public IBus {
private:
    Hub75Driver* _driver;
    uint16_t _width;
    uint16_t _height;
    bool _ready = false;
public:
    BusHub75(uint16_t width, uint16_t height, Hub75ShiftDriver shiftDriver) : _width(width), _height(height) {
        Hub75Config config{};
        config.panel_width = width;
        config.panel_height = height;
        config.shift_driver = shiftDriver;
        // Single-buffered: internal RAM is still limited (see the OVERHEAD_FACTOR comment
        // below), and even a single frame buffer can already be substantial for larger
        // panels/bit depths (double buffering would need twice as much).
        config.double_buffer = false;
        config.brightness = 255; // the Master already scales r/g/b before sending pixel data
        config.pins.r1 = HUB75_PIN_R1; config.pins.g1 = HUB75_PIN_G1; config.pins.b1 = HUB75_PIN_B1;
        config.pins.r2 = HUB75_PIN_R2; config.pins.g2 = HUB75_PIN_G2; config.pins.b2 = HUB75_PIN_B2;
        config.pins.a = HUB75_PIN_A; config.pins.b = HUB75_PIN_B; config.pins.c = HUB75_PIN_C;
        config.pins.d = HUB75_PIN_D; config.pins.e = HUB75_PIN_E;
        config.pins.clk = HUB75_PIN_CLK; config.pins.lat = HUB75_PIN_LAT; config.pins.oe = HUB75_PIN_OE;
        _driver = new Hub75Driver(config);
    }
    ~BusHub75() { delete _driver; }
    // begin() can crash (not just return false) when the DMA buffer for the requested
    // panel/bit-depth doesn't fit in RAM - a bug in the esp-hub75 library that a return-value
    // check can't guard against on its own, since the fault can happen before begin() gets a
    // chance to return. So estimate the buffer size ourselves first and refuse to call begin()
    // at all unless there is a healthy safety margin of free DMA-capable RAM, keeping the device
    // bootable even when a panel is configured too large for the ESP32-S3's ~512KB internal RAM
    // (the GDMA backend this chip uses doesn't route HUB75 buffers into the 2MB PSRAM, so PSRAM
    // doesn't currently help here).
    //
    // The estimate below (num_rows * bit_depth * dma_width * 2 bytes) is only the BASE cost -
    // the library's real allocation also includes per-bit-plane BCM padding for grayscale, which
    // grows roughly as 2^(bit_depth-1) (not linearly with bit_depth like the base estimate does),
    // so the overhead ratio itself grows with HUB75_BIT_DEPTH (set via build_flags, see
    // platformio.ini). Measured on real hardware at the default 8-bit depth for a 64x64 panel:
    // base estimate 32768 bytes, actual requirement 549376 bytes (~16.8x, matching 2^7/8=16) -
    // that measurement was taken on the older ESP32-C6/PARLIO backend, not yet re-verified on
    // this chip's GDMA backend, so treat it as a reasonable starting estimate rather than exact.
    // OVERHEAD_FACTOR reconstructs that same ratio for whatever bit depth is configured, with a
    // +30% and +2 margin on top since exact padding also depends on an auto-tuned internal
    // parameter (lsbMsbTransitionBit) this code doesn't have access to.
    static constexpr size_t OVERHEAD_FACTOR =
        (((size_t(1) << (HUB75_BIT_DEPTH - 1)) * 13) / (HUB75_BIT_DEPTH * 10)) + 2;
    void Begin() override {
        size_t numRows = (_height + 1) / 2; // standard 1/2 scan (matches get_effective_num_rows default)
        size_t estimatedMinBytes = numRows * HUB75_BIT_DEPTH * _width * sizeof(uint16_t);
        size_t freeDmaHeap = heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (estimatedMinBytes * OVERHEAD_FACTOR > freeDmaHeap) {
            Serial.printf("BusHub75: panel %dx%d likely needs more DMA RAM than the %u bytes free - "
                          "refusing to init (would crash). Choose a smaller panel size.\n",
                          _width, _height, (unsigned)freeDmaHeap);
            _ready = false;
            return;
        }
        _ready = _driver->begin();
        if (!_ready) {
            Serial.println("BusHub75: driver init failed (panel likely too large for available RAM) - HUB75 output disabled.");
        }
    }
    void Show() override { if (_ready) _driver->flip_buffer(); }
    void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t w2) override {
        if (!_ready) return;
        uint16_t x = index % _width;
        uint16_t y = index / _width;
        _driver->set_pixel(x, y, r, g, b);
    }
};

// Virtual Bus for Master/Slave (wraps local bus and holds a large memory buffer)
class BusVirtual : public IBus {
private:
    IBus* _localBus;
    uint8_t* _buffer;
    uint16_t _localCount;
    uint16_t _totalCount;
public:
    BusVirtual(IBus* localBus, uint16_t localCount, uint16_t totalCount) {
        _localBus = localBus;
        _localCount = localCount;
        _totalCount = totalCount;
        _buffer = (uint8_t*)calloc(totalCount * 5, 1);
    }
    ~BusVirtual() {
        if (_localBus) delete _localBus;
        if (_buffer) free(_buffer);
    }
    void Begin() override { if (_localBus) _localBus->Begin(); }
    void Show() override { 
        if (_localBus) _localBus->Show(); 
    }
    void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t w2) override {
        if (index < _totalCount && _buffer) {
            _buffer[index*5] = r;
            _buffer[index*5+1] = g;
            _buffer[index*5+2] = b;
            _buffer[index*5+3] = w;
            _buffer[index*5+4] = w2;
        }
        if (index < _localCount && _localBus) {
            _localBus->SetPixelColor(index, r, g, b, w, w2);
        }
    }
    
    const uint8_t* getBuffer() const { return _buffer; }
};
