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
#include <cmath>
#include <algorithm>
#include "Config.h"

class IBus {
public:
    virtual ~IBus() {}
    virtual void Begin() = 0;
    virtual void Show() = 0;
    virtual void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t w2 = 0) = 0;
    virtual const uint8_t* getBuffer() const { return nullptr; }
    // Dims the whole output. Only a bus that can do better than scaling the pixel values
    // overrides it (HUB75); everywhere else the colours arrive already dimmed.
    virtual void setBrightness(uint8_t brightness) { (void)brightness; }
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
        // Single-buffered: internal RAM is still limited (see the sizing comment
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
        initPixelTable();
    }
    ~BusHub75() { delete _driver; }
    // begin() can crash (not just return false) when the DMA buffers for the requested
    // panel/bit-depth don't fit in RAM, so estimate the cost first and refuse to call it at all
    // unless there is a healthy margin of free DMA-capable internal RAM. That keeps the device
    // bootable when a panel is configured too large for the ESP32-S3's ~512KB of internal RAM
    // (the GDMA backend this chip uses allocates with MALLOC_CAP_DMA, so the 2MB PSRAM cannot
    // take these buffers).
    //
    // The GDMA backend allocates exactly two things: the row buffers
    // (num_rows * width * bit_depth * 2 bytes) and one descriptor chain. BCM lives in that chain
    // - the higher bit planes are repeated by re-linking descriptors, not by duplicating pixel
    // data - so the worst case is lsbMsbTransitionBit = 0 with 2^(bit_depth-1) transmissions per
    // row. A 64x64 panel at 6 bits therefore costs about 24KB + 12KB.
    //
    // An earlier version of this estimate multiplied the buffer by up to ~22x. That factor was
    // measured on the ESP32-C6/PARLIO backend, which pads the buffer itself, and does not apply
    // to GDMA - it made this guard reject panel sizes that run comfortably.
    static constexpr size_t DMA_DESCRIPTOR_BYTES = 12; // sizeof(dma_descriptor_t) on the ESP32-S3
    void Begin() override {
        size_t numRows = (_height + 1) / 2; // standard 1/2 scan (matches get_effective_num_rows default)
        size_t bufferBytes = numRows * _width * HUB75_BIT_DEPTH * sizeof(uint16_t);
        size_t descriptorBytes = numRows * (size_t(1) << (HUB75_BIT_DEPTH - 1)) * DMA_DESCRIPTOR_BYTES;
        size_t needed = bufferBytes + descriptorBytes;
        // Half again plus 8KB of headroom: the allocation must not succeed at the cost of leaving
        // the rest of the firmware - the WiFi stack above all - without internal RAM.
        size_t required = needed + needed / 2 + 8192;
        size_t freeDmaHeap = heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        Serial.printf("BusHub75: panel %dx%d at %d bits needs ~%u bytes of DMA RAM (%u free)\n",
                      _width, _height, (int)HUB75_BIT_DEPTH, (unsigned)needed, (unsigned)freeDmaHeap);
        if (required > freeDmaHeap) {
            Serial.printf("BusHub75: refusing to init - %u bytes required including margin, only %u free. "
                          "Reduce HUB75_BIT_DEPTH or the panel size.\n",
                          (unsigned)required, (unsigned)freeDmaHeap);
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
        const uint8_t* lut = _lut; // one read: the loop may swap tables while a stream is written
        _driver->set_pixel(x, y, lut[r], lut[g], lut[b]);
    }

    // Brightness for full-scale colours (0-255, same scale and feel as the segment slider).
    //
    // Scaling the pixel values does not work on this panel at low brightness. The driver runs
    // every value through a CIE1931 curve, so 8% on the slider leaves each channel one to three
    // of its 255 levels: weak channels vanish, strong ones stand alone, and pink turns purple.
    //
    // The driver can instead dim by shortening the time each row is lit, which keeps all 255
    // levels. It cannot go below about 4/63 of full that way, while the slider's perceptual curve
    // goes much lower (8% is under 1% of the light). So the time does as much as it can, and only
    // the rest is taken off the pixel values - per channel in light, not in value, so the ratio
    // between the channels, which is the colour, stays as it was.
    //
    // Pass 255 whenever the colours arrive already dimmed (a stream from the Master): that is
    // full on-time and an identity table.
    void setBrightness(uint8_t brightness) override {
        if (!_ready) return;
        if (brightness == 0) {
            _driver->set_brightness(0);
            return;
        }
        double target = cieLuminance(brightness / 255.0);           // light the slider asks for
        int wanted = (int)std::ceil(target * _maxPixels);
        uint8_t level = 1;                                           // smallest level that lights
        while (level < 255 && _pixelsAt[level] < wanted) level++;   // at least `wanted` pixels
        int pixels = _pixelsAt[level] > 0 ? _pixelsAt[level] : 1;
        double rest = target * _maxPixels / pixels;                 // what the time could not do
        if (rest > 1.0) rest = 1.0;

        uint8_t* next = (_lut == _lutA) ? _lutB : _lutA;
        for (int v = 0; v < 256; v++) {
            double y = cieLuminance(v / 255.0) * rest;
            long out = lround(cieInverse(y) * 255.0);
            next[v] = (uint8_t)(out < 0 ? 0 : (out > 255 ? 255 : out));
        }
        _lut = next;
        _driver->set_brightness(level);
    }

private:
    uint8_t _lutA[256];
    uint8_t _lutB[256];
    uint8_t* _lut = _lutA;
    // Lit pixels per row slot for each driver brightness level, and their maximum.
    uint8_t _pixelsAt[256];
    int _maxPixels = 63;

    static double cieLuminance(double x) {   // the driver's gamma table (HUB75_GAMMA_MODE 1)
        double L = x * 100.0;
        return L <= 8.0 ? L / 902.3 : std::pow((L + 16.0) / 116.0, 3.0);
    }
    static double cieInverse(double y) {
        double L = y <= 0.008856 ? y * 902.3 : 116.0 * std::cbrt(y) - 16.0;
        if (L < 0.0) L = 0.0;
        if (L > 100.0) L = 100.0;
        return L / 100.0;
    }

    // Mirrors esp-hub75 0.3.x: PlatformDma::init_brightness_coeffs(), remap_brightness() and the
    // display_pixels computation in the DMA backends. Only used to pick a level; if a later driver
    // version moves the curve, the result is a little off in brightness but still monotonic and
    // still colour-true.
    void initPixelTable() {
        _maxPixels = (int)_width - 1; // default latch_blanking of 1
        if (_maxPixels < 2) _maxPixels = 2;
        int minLevel = std::min(255, (4 * 256 + _maxPixels - 1) / _maxPixels);
        const float y1 = (float)minLevel, y2 = 128.0f, y3 = 255.0f, denom = -4096258.0f;
        const float a = (255.0f * (y2 - y1) + 128.0f * (y1 - y3) + 1.0f * (y3 - y2)) / denom;
        const float b = (255.0f * 255.0f * (y1 - y2) + 128.0f * 128.0f * (y3 - y1) + 1.0f * 1.0f * (y2 - y3)) / denom;
        const float c = (128.0f * 255.0f * (128.0f - 255.0f) * y1 + 255.0f * 1.0f * (255.0f - 1.0f) * y2 +
                         1.0f * 128.0f * (1.0f - 128.0f) * y3) / denom;
        const int32_t fa = (int32_t)(a * 65536.0f), fb = (int32_t)(b * 65536.0f), fc = (int32_t)(c * 65536.0f);
        _pixelsAt[0] = 0;
        for (int x = 1; x < 256; x++) {
            int32_t yfp = fa * x * x + fb * x + fc;
            int effective = (yfp + 32768) >> 16;
            if (effective < minLevel) effective = minLevel;
            if (effective > 255) effective = 255;
            int pixels = (_maxPixels * effective) >> 8;
            if (pixels > _maxPixels - 1) pixels = _maxPixels - 1;
            _pixelsAt[x] = (uint8_t)pixels;
        }
        for (int v = 0; v < 256; v++) _lutA[v] = (uint8_t)v;
        _lut = _lutA;
    }
public:
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
