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
#ifndef EFFECTENGINE_H
#define EFFECTENGINE_H

#include <Arduino.h>

// Effect rendering, decoupled from where the pixels end up.
//
// A Slave renders its own effects from parameters instead of receiving finished pixel data from
// the Master. Streaming does not scale: a 64x64 panel is 4096 pixels, which is far more per frame
// than ESP-NOW carries at a usable rate, and more than the UART payload limit allows at all. The
// parameters below are a few dozen bytes and only travel when something actually changes.
//
// The implementations are ports of the Master's own effects in src/LEDManager.cpp and must stay
// behaviourally identical, so an effect looks the same whether it runs on the Master or on a
// Slave. Effect ids match the Master's dispatch table exactly - see canRender() for which ones
// are available here; anything else still falls back to streamed pixel data.

struct EffectState {
    uint8_t effect = 0;
    uint8_t brightness = 128;
    uint8_t speed = 128;
    uint8_t intensity = 128;
    uint8_t palette = 0;
    bool isOn = true;
    uint32_t color = 0xFFFFFF;
    uint32_t color2 = 0x0000FF;
    bool color2Enabled = false;
    bool whiteOnly = false;
    uint8_t cct = 128;

    // Animation state, owned by the renderer rather than the sender.
    uint16_t effectStep = 0;
    unsigned long lastUpdate = 0;
};

// Where rendered pixels go. Implemented by the Slave over its own strip or panel.
class IEffectSink {
public:
    virtual ~IEffectSink() {}
    virtual void setPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t w2) = 0;
    virtual uint16_t pixelCount() const = 0;
    // 0 when this sink is a plain strip. 2D effects fall back to a 1D relative in that case.
    virtual uint16_t matrixWidth() const { return 0; }
    virtual uint16_t matrixHeight() const { return 0; }
};

class EffectEngine {
public:
    // Whether this effect can be rendered without the Master. Effects needing resources only the
    // Master has - the image library, the clock and weather data behind the text/clock effect -
    // are deliberately excluded and keep using streamed pixel data.
    static bool canRender(uint8_t effect) {
        switch (effect) {
            case 0: case 1: case 2: case 3: case 5: case 6:
            case 11: case 12: case 13: case 18: case 19: case 20: case 21:
                return true;
            default:
                return false;
        }
    }

    // Advances and draws the effect if it is due. Returns true when pixels changed, so the caller
    // knows whether it needs to push a frame out. Timing matches the Master's loop() exactly.
    static bool render(EffectState& st, IEffectSink& sink, unsigned long now) {
        unsigned int delayMs = 500 - ((unsigned int)st.speed * 490 / 255);
        if (st.effect == 0) delayMs = 100;
        if (now - st.lastUpdate <= delayMs) return false;
        st.lastUpdate = now;

        if (!st.isOn) {
            fill(st, sink, 0, 0, 0);
            return true;
        }

        switch (st.effect) {
            case 0:  solid(st, sink); break;
            case 1:  breathe(st, sink); break;
            case 2:  rainbow(st, sink); break;
            case 3:  chase(st, sink); break;
            case 5:  colorWipe(st, sink); break;
            case 6:  scanner(st, sink); break;
            case 11: strobe(st, sink); break;
            case 12: bounce(st, sink); break;
            case 13: paletteRainbow(st, sink); break;
            case 18: theaterChaseRainbow(st, sink); break;
            case 19: runningLights(st, sink); break;
            case 20: colorWaves(st, sink); break;
            case 21: plasma(st, sink); break;
            default: solid(st, sink); break;
        }
        return true;
    }

    static uint32_t wheel(uint8_t pos) {
        pos = 255 - pos;
        if (pos < 85) return (((uint32_t)(255 - pos * 3) << 16) | (uint32_t)(pos * 3));
        if (pos < 170) { pos -= 85; return (((uint32_t)(pos * 3) << 8) | (uint32_t)(255 - pos * 3)); }
        pos -= 170;
        return (((uint32_t)(pos * 3) << 16) | ((uint32_t)(255 - pos * 3) << 8));
    }

    static uint32_t paletteColor(uint8_t paletteId, uint8_t pos) {
        if (paletteId == 0 || paletteId >= PALETTE_COUNT) return 0;
        const PaletteDef& pal = PALETTES[paletteId];
        if (pal.stopCount == 0) return 0;
        if (pal.stopCount == 1) return pal.stops[0];

        uint16_t segWidth = 256 / (pal.stopCount - 1);
        uint8_t segIdx = pos / segWidth;
        if (segIdx >= pal.stopCount - 1) segIdx = pal.stopCount - 2;
        uint8_t localPos = pos - segIdx * segWidth;
        uint8_t blend = (uint16_t)localPos * 255 / segWidth;

        uint32_t c1 = pal.stops[segIdx];
        uint32_t c2 = pal.stops[segIdx + 1];
        uint8_t r = (((c1 >> 16) & 0xFF) * (255 - blend) + ((c2 >> 16) & 0xFF) * blend) / 255;
        uint8_t g = (((c1 >> 8) & 0xFF) * (255 - blend) + ((c2 >> 8) & 0xFF) * blend) / 255;
        uint8_t b = ((c1 & 0xFF) * (255 - blend) + (c2 & 0xFF) * blend) / 255;
        return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    }

private:
    static const uint8_t PALETTE_COUNT = 5;
    struct PaletteDef { uint8_t stopCount; uint32_t stops[4]; };
    // Same stops as the Master's PALETTE_STOPS - keep in sync.
    static const PaletteDef PALETTES[PALETTE_COUNT];

    // Cheap 0-255 triangle wave, a lightweight stand-in for a sine so effects avoid trig.
    static uint8_t triWave8(uint8_t pos) {
        return (pos < 128) ? (uint8_t)(pos * 2) : (uint8_t)(255 - (pos - 128) * 2);
    }

    // Applies the white-only/CCT handling the Master does in setSegmentPixelColor().
    static void emit(EffectState& st, IEffectSink& sink, uint16_t i, uint8_t r, uint8_t g, uint8_t b) {
        if (st.whiteOnly) {
            uint8_t bri = r > g ? (r > b ? r : b) : (g > b ? g : b);
            uint8_t w1 = (bri * (255 - st.cct)) / 255;
            uint8_t w2 = (bri * st.cct) / 255;
            sink.setPixel(i, 0, 0, 0, w1, w2);
        } else {
            sink.setPixel(i, r, g, b, 0, 0);
        }
    }

    static void fill(EffectState& st, IEffectSink& sink, uint8_t r, uint8_t g, uint8_t b) {
        uint16_t count = sink.pixelCount();
        for (uint16_t i = 0; i < count; i++) emit(st, sink, i, r, g, b);
    }

    static uint16_t bri(const EffectState& st) { return st.brightness; }

    static void scaled(uint32_t c, uint16_t b, uint8_t& r, uint8_t& g, uint8_t& bl) {
        r = (((c >> 16) & 0xFF) * b) / 255;
        g = (((c >> 8) & 0xFF) * b) / 255;
        bl = ((c & 0xFF) * b) / 255;
    }

    static void solid(EffectState& st, IEffectSink& sink) {
        uint8_t r, g, b; scaled(st.color, bri(st), r, g, b);
        fill(st, sink, r, g, b);
    }

    static void breathe(EffectState& st, IEffectSink& sink) {
        float breath = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 108.0;
        uint16_t b0 = bri(st);
        uint8_t r = (uint8_t)((((st.color >> 16) & 0xFF) * breath * b0) / 65025);
        uint8_t g = (uint8_t)((((st.color >> 8) & 0xFF) * breath * b0) / 65025);
        uint8_t b = (uint8_t)(((st.color & 0xFF) * breath * b0) / 65025);
        fill(st, sink, r, g, b);
    }

    static void rainbow(EffectState& st, IEffectSink& sink) {
        uint16_t count = sink.pixelCount();
        if (count == 0) return;
        for (uint16_t i = 0; i < count; i++) {
            uint32_t c = wheel((uint8_t)(((i * 256 / count) + st.effectStep) & 255));
            uint8_t r, g, b; scaled(c, bri(st), r, g, b);
            emit(st, sink, i, r, g, b);
        }
        st.effectStep += 5;
    }

    static void chase(EffectState& st, IEffectSink& sink) {
        uint16_t count = sink.pixelCount();
        uint8_t r, g, b; scaled(st.color, bri(st), r, g, b);
        uint8_t r2 = 0, g2 = 0, b2 = 0;
        if (st.color2Enabled) scaled(st.color2, bri(st), r2, g2, b2);
        for (uint16_t i = 0; i < count; i++) {
            if ((i + st.effectStep) % 3 == 0) emit(st, sink, i, r, g, b);
            else emit(st, sink, i, r2, g2, b2);
        }
        st.effectStep++;
    }

    static void colorWipe(EffectState& st, IEffectSink& sink) {
        uint16_t count = sink.pixelCount();
        if (count == 0) return;
        uint8_t r, g, b; scaled(st.color, bri(st), r, g, b);
        uint8_t r2 = 0, g2 = 0, b2 = 0;
        if (st.color2Enabled) scaled(st.color2, bri(st), r2, g2, b2);

        uint16_t pos = st.effectStep % (count * 2);
        for (uint16_t i = 0; i < count; i++) {
            if (pos < count) {
                if (i <= pos) emit(st, sink, i, r, g, b);
                else emit(st, sink, i, r2, g2, b2);
            } else {
                if (i <= (pos - count)) emit(st, sink, i, r2, g2, b2);
                else emit(st, sink, i, r, g, b);
            }
        }
        st.effectStep++;
    }

    static void scanner(EffectState& st, IEffectSink& sink) {
        uint16_t count = sink.pixelCount();
        if (count < 2) return;
        uint8_t r, g, b; scaled(st.color, bri(st), r, g, b);

        uint16_t span = count * 2 - 2;
        uint16_t raw = st.effectStep % span;
        uint16_t pos = raw >= count ? span - raw : raw;
        for (uint16_t i = 0; i < count; i++) {
            uint16_t dist = i > pos ? i - pos : pos - i;
            if (dist < 2) {
                uint8_t fade = 2 - dist;
                emit(st, sink, i, (r * fade) / 2, (g * fade) / 2, (b * fade) / 2);
            } else {
                emit(st, sink, i, 0, 0, 0);
            }
        }
        st.effectStep++;
    }

    static void strobe(EffectState& st, IEffectSink& sink) {
        uint8_t r = 0, g = 0, b = 0;
        if ((st.effectStep % 4) == 0) scaled(st.color, bri(st), r, g, b);
        fill(st, sink, r, g, b);
        st.effectStep++;
    }

    static void bounce(EffectState& st, IEffectSink& sink) {
        uint16_t count = sink.pixelCount();
        if (count < 2) return;
        uint8_t r, g, b; scaled(st.color, bri(st), r, g, b);

        uint16_t span = count * 2 - 2;
        uint16_t raw = st.effectStep % span;
        uint16_t pos = raw >= count ? span - raw : raw;
        uint16_t trailLen = 2 + st.intensity / 32;
        for (uint16_t i = 0; i < count; i++) {
            uint16_t dist = i > pos ? i - pos : pos - i;
            if (dist < trailLen) {
                uint8_t fade = (uint8_t)(trailLen - dist);
                emit(st, sink, i, (r * fade) / trailLen, (g * fade) / trailLen, (b * fade) / trailLen);
            } else {
                emit(st, sink, i, 0, 0, 0);
            }
        }
        st.effectStep++;
    }

    static void paletteRainbow(EffectState& st, IEffectSink& sink) {
        uint16_t count = sink.pixelCount();
        if (count == 0) return;
        for (uint16_t i = 0; i < count; i++) {
            uint8_t pos = (uint8_t)(((i * 256 / count) + st.effectStep) & 255);
            uint32_t c = (st.palette == 0) ? wheel(pos) : paletteColor(st.palette, pos);
            uint8_t r, g, b; scaled(c, bri(st), r, g, b);
            emit(st, sink, i, r, g, b);
        }
        st.effectStep += 5;
    }

    static void theaterChaseRainbow(EffectState& st, IEffectSink& sink) {
        uint16_t count = sink.pixelCount();
        for (uint16_t i = 0; i < count; i++) {
            if ((i + st.effectStep) % 3 == 0) {
                uint32_t c = wheel((uint8_t)(((i * 4) + st.effectStep) & 0xFF));
                uint8_t r, g, b; scaled(c, bri(st), r, g, b);
                emit(st, sink, i, r, g, b);
            } else {
                emit(st, sink, i, 0, 0, 0);
            }
        }
        st.effectStep++;
    }

    static void runningLights(EffectState& st, IEffectSink& sink) {
        uint16_t count = sink.pixelCount();
        if (count == 0) return;
        uint8_t baseR = (st.color >> 16) & 0xFF;
        uint8_t baseG = (st.color >> 8) & 0xFF;
        uint8_t baseB = st.color & 0xFF;
        for (uint16_t i = 0; i < count; i++) {
            uint8_t wavePos = (uint8_t)((((uint32_t)i * 512 / count) + st.effectStep) & 0xFF);
            uint16_t scale = (uint16_t)bri(st) * triWave8(wavePos) / 255;
            emit(st, sink, i, (uint8_t)((baseR * scale) / 255),
                              (uint8_t)((baseG * scale) / 255),
                              (uint8_t)((baseB * scale) / 255));
        }
        st.effectStep += 4;
    }

    static void colorWaves(EffectState& st, IEffectSink& sink) {
        uint16_t count = sink.pixelCount();
        if (count == 0) return;
        for (uint16_t i = 0; i < count; i++) {
            uint8_t basePos = (uint8_t)((((uint32_t)i * 256 / count) + st.effectStep / 3) & 0xFF);
            uint8_t wobble = triWave8((uint8_t)((i * 9 + st.effectStep) & 0xFF));
            uint8_t pos = (uint8_t)(basePos + (wobble / 4));
            uint32_t c = (st.palette == 0) ? wheel(pos) : paletteColor(st.palette, pos);
            uint8_t r, g, b; scaled(c, bri(st), r, g, b);
            emit(st, sink, i, r, g, b);
        }
        st.effectStep++;
    }

    static void plasma(EffectState& st, IEffectSink& sink) {
        uint16_t w = sink.matrixWidth();
        uint16_t h = sink.matrixHeight();
        if (w == 0 || h == 0) { colorWaves(st, sink); return; } // 1D fallback, as on the Master

        uint8_t scale = 4 + st.intensity / 16;
        for (uint16_t y = 0; y < h; y++) {
            for (uint16_t x = 0; x < w; x++) {
                uint8_t v1 = triWave8((uint8_t)(x * scale + st.effectStep));
                uint8_t v2 = triWave8((uint8_t)(y * scale + st.effectStep * 2));
                uint8_t v3 = triWave8((uint8_t)((x + y) * (scale / 2) + st.effectStep / 2));
                uint8_t pos = (uint8_t)(((uint16_t)v1 + v2 + v3) / 3);
                uint32_t c = (st.palette == 0) ? wheel(pos) : paletteColor(st.palette, pos);
                uint8_t r, g, b; scaled(c, bri(st), r, g, b);
                emit(st, sink, y * w + x, r, g, b);
            }
        }
        st.effectStep++;
    }
};

inline const EffectEngine::PaletteDef EffectEngine::PALETTES[EffectEngine::PALETTE_COUNT] = {
    {0, {}},                                            // Solid (unused)
    {4, {0xFF0000, 0xFFFF00, 0x00FF00, 0x0000FF}},      // Rainbow
    {3, {0x000000, 0xFF4500, 0xFFFF00}},                // Fire
    {3, {0x000033, 0x0077BE, 0x00FFFF}},                // Ocean
    {3, {0x013220, 0x228B22, 0x7CFC00}},                // Forest
};

#endif
