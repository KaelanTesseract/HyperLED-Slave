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

class IBus {
public:
    virtual ~IBus() {}
    virtual void Begin() = 0;
    virtual void Show() = 0;
    virtual void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w) = 0;
    virtual const uint8_t* getBuffer() const { return nullptr; }
};

// 1-Wire Digital RGB
template<typename T_FEATURE, typename T_METHOD>
class BusDigitalRgb : public IBus {
private:
    NeoPixelBus<T_FEATURE, T_METHOD>* _bus;
public:
    BusDigitalRgb(uint16_t count, uint8_t pinData) {
        _bus = new NeoPixelBus<T_FEATURE, T_METHOD>(count, pinData);
    }
    ~BusDigitalRgb() { delete _bus; }
    void Begin() override { _bus->Begin(); }
    void Show() override { _bus->Show(); }
    void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w) override {
        _bus->SetPixelColor(index, RgbColor(r, g, b));
    }
};

// 1-Wire Digital RGBW
template<typename T_FEATURE, typename T_METHOD>
class BusDigitalRgbw : public IBus {
private:
    NeoPixelBus<T_FEATURE, T_METHOD>* _bus;
public:
    BusDigitalRgbw(uint16_t count, uint8_t pinData) {
        _bus = new NeoPixelBus<T_FEATURE, T_METHOD>(count, pinData);
    }
    ~BusDigitalRgbw() { delete _bus; }
    void Begin() override { _bus->Begin(); }
    void Show() override { _bus->Show(); }
    void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w) override {
        _bus->SetPixelColor(index, RgbwColor(r, g, b, w));
    }
};

// 2-Wire SPI Digital RGB
template<typename T_FEATURE, typename T_METHOD>
class BusDigitalSpiRgb : public IBus {
private:
    NeoPixelBus<T_FEATURE, T_METHOD>* _bus;
public:
    BusDigitalSpiRgb(uint16_t count, uint8_t pinClock, uint8_t pinData) {
        _bus = new NeoPixelBus<T_FEATURE, T_METHOD>(count, pinClock, pinData);
    }
    ~BusDigitalSpiRgb() { delete _bus; }
    void Begin() override { _bus->Begin(); }
    void Show() override { _bus->Show(); }
    void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w) override {
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
    void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w) override {
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
    
    void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w) override {
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
        _buffer = (uint8_t*)calloc(totalCount * 4, 1);
    }
    ~BusVirtual() {
        if (_localBus) delete _localBus;
        if (_buffer) free(_buffer);
    }
    void Begin() override { if (_localBus) _localBus->Begin(); }
    void Show() override { 
        if (_localBus) _localBus->Show(); 
    }
    void SetPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w) override {
        if (index < _totalCount && _buffer) {
            _buffer[index*4] = r;
            _buffer[index*4+1] = g;
            _buffer[index*4+2] = b;
            _buffer[index*4+3] = w;
        }
        if (index < _localCount && _localBus) {
            _localBus->SetPixelColor(index, r, g, b, w);
        }
    }
    
    const uint8_t* getBuffer() const { return _buffer; }
};
