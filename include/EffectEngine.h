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
#include <vector>

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

    // Per-pixel memory for effects that carry a picture from one frame into the next. Sized
    // lazily on first use: the size depends on the sink, and only some effects need any.
    std::vector<uint8_t> fireHeat;        // Fire (pixel-sized) and Fire 2D (canvas-sized)
    std::vector<uint8_t> rippleState;     // 3 bytes (x, y, radius) per ripple
    std::vector<int16_t> matrixRainHeads; // one falling head position per column
    std::vector<uint8_t> twinkleState;    // fade level per pixel
    std::vector<uint8_t> sinelonState;    // decaying trail brightness per pixel
    std::vector<uint8_t> confettiState;   // fade level per spark
    std::vector<uint8_t> confettiHue;     // each spark's own hue, kept apart from its fade
    std::vector<uint8_t> juggleState;     // 3 bytes (r, g, b) per pixel, additively blended
    std::vector<uint8_t> fireworksState;  // 6 bytes (x, y, peakY, hue, timer, state) per rocket
    std::vector<uint8_t> starfieldState;  // 3 bytes (dx, dy, radius) per star
    std::vector<uint8_t> ballsState;      // 5 bytes (x, y, vx, vy, hue) per ball
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
    // 2D effects address pixels by coordinate. The default lays the matrix out row by row, which
    // is what a Slave panel wants. The Master overrides it to draw onto its whole virtual canvas
    // (its own matrix plus any Slave panels), so a wave pattern flows across panel boundaries
    // instead of restarting inside each segment.
    virtual void setPixelXY(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b) {
        setPixel(y * matrixWidth() + x, r, g, b, 0, 0);
    }
};

class EffectEngine {
public:
    // Whether this effect can be rendered without the Master. Effects needing resources only the
    // Master has - the image library, the clock and weather data behind the text/clock effect -
    // are deliberately excluded and keep using streamed pixel data.
    static bool canRender(uint8_t effect) {
        switch (effect) {
            // Everything except 25 (image data) and 29 (clock/text), which need resources only
            // the Master has. 10 is not an effect - it is the white-only mode handled elsewhere.
            case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8: case 9:
            case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18: case 19:
            case 20: case 21: case 22: case 23: case 24: case 26: case 27: case 28:
                return true;
            default:
                return false;
        }
    }

    // How long to wait between frames for a given speed setting. Shared so the Master's loop and
    // a Slave's own scheduling stay on the same timing.
    static unsigned int frameDelayMs(const EffectState& st) {
        if (st.effect == 0) return 100;
        return 500 - ((unsigned int)st.speed * 490 / 255);
    }

    // Advances and draws the effect if it is due. Returns true when pixels changed, so the caller
    // knows whether it needs to push a frame out. Used by Slaves, which have no scheduling of
    // their own; the Master calls draw() from inside its existing loop instead.
    static bool render(EffectState& st, IEffectSink& sink, unsigned long now) {
        if (now - st.lastUpdate <= frameDelayMs(st)) return false;
        st.lastUpdate = now;
        draw(st, sink);
        return true;
    }

    // Draws one frame unconditionally. The caller owns the timing.
    static void draw(EffectState& st, IEffectSink& sink) {
        if (!st.isOn) {
            fill(st, sink, 0, 0, 0);
            return;
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
            case 4:  fire(st, sink); break;
            case 9:  matrixRain(st, sink); break;
            case 22: ripple(st, sink); break;
            case 23: fire2D(st, sink); break;
            case 7:  twinkle(st, sink); break;
            case 8:  meteor(st, sink); break;
            case 14: sinelon(st, sink); break;
            case 15: confetti(st, sink); break;
            case 16: juggle(st, sink); break;
            case 17: bpm(st, sink); break;
            case 24: pacifica(st, sink); break;
            case 26: fireworks(st, sink); break;
            case 27: starfield(st, sink); break;
            case 28: bouncingBalls(st, sink); break;
            default: solid(st, sink); break;
        }
    }

    static uint32_t wheel(uint8_t pos) {
        pos = 255 - pos;
        if (pos < 85) return (((uint32_t)(255 - pos * 3) << 16) | (uint32_t)(pos * 3));
        if (pos < 170) { pos -= 85; return (((uint32_t)(pos * 3) << 8) | (uint32_t)(255 - pos * 3)); }
        pos -= 170;
        return (((uint32_t)(pos * 3) << 16) | ((uint32_t)(255 - pos * 3) << 8));
    }

    static uint32_t paletteColor(uint8_t paletteId, uint8_t pos) {
        if (paletteId == 0 || paletteId >= PALETTE_SLOTS) return 0;
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
    // Named PALETTE_SLOTS rather than PALETTE_COUNT: the Master defines the latter as a
    // macro, which would textually replace the member and fail to compile.
    static const uint8_t PALETTE_SLOTS = 5;
    struct PaletteDef { uint8_t stopCount; uint32_t stops[4]; };
    // Same stops as the Master's PALETTE_STOPS - keep in sync.
    static const PaletteDef PALETTES[PALETTE_SLOTS];

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

    // Rounds rather than truncates. Truncation always biases a scaled colour downwards, and it
    // does so unevenly across channels, which tints a colour as it is dimmed.
    static void scaled(uint32_t c, uint16_t b, uint8_t& r, uint8_t& g, uint8_t& bl) {
        r = (uint8_t)(((((c >> 16) & 0xFF) * b) + 127) / 255);
        g = (uint8_t)(((((c >> 8) & 0xFF) * b) + 127) / 255);
        bl = (uint8_t)((((c & 0xFF) * b) + 127) / 255);
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

    // Maps a heat value to a flame colour. Palette 0 uses the classic black-red-yellow-white
    // ramp; any other palette recolours the flame (Ocean turns it into rising bubbles).
    static void heatToColor(const EffectState& st, uint8_t heat, uint8_t& r, uint8_t& g, uint8_t& b) {
        if (st.palette == 0) {
            uint8_t t192 = (heat * 191) / 255;
            uint8_t heatramp = (t192 & 0x3F) << 2;
            if (t192 > 128) { r = 255; g = 255; b = heatramp; }
            else if (t192 > 64) { r = 255; g = heatramp; b = 0; }
            else { r = heatramp; g = 0; b = 0; }
        } else {
            uint32_t c = paletteColor(st.palette, heat);
            r = (c >> 16) & 0xFF; g = (c >> 8) & 0xFF; b = c & 0xFF;
        }
        r = (r * st.brightness) / 255;
        g = (g * st.brightness) / 255;
        b = (b * st.brightness) / 255;
    }

    static void fire(EffectState& st, IEffectSink& sink) {
        uint16_t count = sink.pixelCount();
        if (count == 0) return;
        if (st.fireHeat.size() != count) st.fireHeat.assign(count, 0);

        // intensity controls the cooling rate: higher means shorter, choppier flames
        const uint8_t cooling = 20 + (st.intensity * 80) / 255;
        const uint8_t sparking = 120;

        for (uint16_t i = 0; i < count; i++) {
            uint8_t cooldown = random(0, ((cooling * 10) / count) + 2);
            st.fireHeat[i] = (st.fireHeat[i] > cooldown) ? st.fireHeat[i] - cooldown : 0;
        }
        for (uint16_t i = count - 1; i >= 2; i--) {
            st.fireHeat[i] = (st.fireHeat[i - 1] + st.fireHeat[i - 2] + st.fireHeat[i - 2]) / 3;
        }
        if (random(0, 255) < sparking) {
            uint16_t sparkRange = count < 7 ? count : 7;
            uint16_t y = random(0, sparkRange);
            uint16_t add = random(160, 255);
            st.fireHeat[y] = (st.fireHeat[y] + add > 255) ? 255 : st.fireHeat[y] + add;
        }
        for (uint16_t i = 0; i < count; i++) {
            uint8_t r, g, b;
            heatToColor(st, st.fireHeat[i], r, g, b);
            emit(st, sink, i, r, g, b);
        }
    }

    static void fire2D(EffectState& st, IEffectSink& sink) {
        uint16_t cw = sink.matrixWidth();
        uint16_t ch = sink.matrixHeight();
        if (cw == 0 || ch == 0) { fire(st, sink); return; } // same 1D fallback as the Master

        size_t total = (size_t)cw * ch;
        if (st.fireHeat.size() != total) st.fireHeat.assign(total, 0);

        const uint8_t cooling = 20 + (st.intensity * 80) / 255;
        for (size_t i = 0; i < total; i++) {
            uint8_t cooldown = random(0, ((cooling * 10) / cw) + 2);
            st.fireHeat[i] = (st.fireHeat[i] > cooldown) ? st.fireHeat[i] - cooldown : 0;
        }
        // Heat rises within each column; y = 0 is the base row that carries the sparks.
        for (uint16_t x = 0; x < cw; x++) {
            for (uint16_t y = ch - 1; y >= 2; y--) {
                size_t idx = (size_t)y * cw + x;
                size_t b1 = (size_t)(y - 1) * cw + x;
                size_t b2 = (size_t)(y - 2) * cw + x;
                st.fireHeat[idx] = (st.fireHeat[b1] + st.fireHeat[b2] + st.fireHeat[b2]) / 3;
            }
        }
        for (uint16_t x = 0; x < cw; x++) {
            if (random(0, 255) < 120) {
                uint16_t add = random(160, 255);
                st.fireHeat[x] = (st.fireHeat[x] + add > 255) ? 255 : st.fireHeat[x] + add;
            }
        }
        for (uint16_t y = 0; y < ch; y++) {
            for (uint16_t x = 0; x < cw; x++) {
                uint8_t r, g, b;
                heatToColor(st, st.fireHeat[(size_t)y * cw + x], r, g, b);
                sink.setPixelXY(x, y, r, g, b);
            }
        }
    }

    static void matrixRain(EffectState& st, IEffectSink& sink) {
        uint16_t cw = sink.matrixWidth();
        uint16_t ch = sink.matrixHeight();
        if (cw == 0 || ch == 0) { chase(st, sink); return; } // same 1D fallback as the Master

        if (st.matrixRainHeads.size() != cw) {
            st.matrixRainHeads.assign(cw, 0);
            // Stagger the starts so the columns do not all fall in lockstep.
            for (uint16_t x = 0; x < cw; x++) st.matrixRainHeads[x] = -(int16_t)random(0, ch * 2);
        }

        for (uint16_t y = 0; y < ch; y++)
            for (uint16_t x = 0; x < cw; x++) sink.setPixelXY(x, y, 0, 0, 0);

        uint8_t r, g, b; scaled(st.color, bri(st), r, g, b);
        const int16_t trailLen = 6;
        for (uint16_t x = 0; x < cw; x++) {
            int16_t head = st.matrixRainHeads[x];
            for (int16_t t = 0; t < trailLen; t++) {
                int16_t y = head - t;
                if (y >= 0 && y < (int16_t)ch) {
                    uint8_t fade = trailLen - t; // brightest at the head, fading upward
                    sink.setPixelXY(x, y, (r * fade) / trailLen, (g * fade) / trailLen, (b * fade) / trailLen);
                }
            }
            head++;
            if (head - trailLen > (int16_t)ch) head = -(int16_t)random(0, ch);
            st.matrixRainHeads[x] = head;
        }
        st.effectStep++;
    }

    static void ripple(EffectState& st, IEffectSink& sink) {
        uint16_t cw = sink.matrixWidth();
        uint16_t ch = sink.matrixHeight();
        if (cw == 0 || ch == 0) { bounce(st, sink); return; } // same 1D fallback as the Master

        const uint8_t numRipples = 3;
        // Origin and radius are a byte each, so canvases past 255px clip - rare and accepted.
        if (st.rippleState.size() != (size_t)numRipples * 3) {
            st.rippleState.assign((size_t)numRipples * 3, 0); // radius 0 means inactive
        }

        for (uint16_t y = 0; y < ch; y++)
            for (uint16_t x = 0; x < cw; x++) sink.setPixelXY(x, y, 0, 0, 0);

        uint16_t maxRadius = (cw > ch ? cw : ch);
        uint8_t baseR = (st.color >> 16) & 0xFF;
        uint8_t baseG = (st.color >> 8) & 0xFF;
        uint8_t baseB = st.color & 0xFF;

        for (uint8_t rp = 0; rp < numRipples; rp++) {
            uint8_t base = rp * 3;
            uint8_t rx = st.rippleState[base];
            uint8_t ry = st.rippleState[base + 1];
            uint8_t radius = st.rippleState[base + 2];

            if (radius == 0) {
                // intensity sets how often a new ripple starts (roughly 2%-18% per frame)
                if (random(0, 100) < (2 + st.intensity / 16)) {
                    st.rippleState[base] = (uint8_t)random(0, cw > 255 ? 255 : cw);
                    st.rippleState[base + 1] = (uint8_t)random(0, ch > 255 ? 255 : ch);
                    st.rippleState[base + 2] = 1;
                }
                continue;
            }

            for (uint16_t y = 0; y < ch; y++) {
                for (uint16_t x = 0; x < cw; x++) {
                    int16_t dx = (int16_t)x - rx;
                    int16_t dy = (int16_t)y - ry;
                    uint16_t distSq = (uint16_t)(dx * dx + dy * dy);
                    uint16_t rSq = (uint16_t)radius * radius;
                    uint16_t rPrevSq = radius > 1 ? (uint16_t)(radius - 1) * (radius - 1) : 0;
                    if (distSq <= rSq && distSq > rPrevSq) {
                        uint8_t fade = 255 - (uint16_t)(radius * 255 / maxRadius);
                        sink.setPixelXY(x, y, (baseR * bri(st) / 255 * fade) / 255,
                                              (baseG * bri(st) / 255 * fade) / 255,
                                              (baseB * bri(st) / 255 * fade) / 255);
                    }
                }
            }
            radius++;
            st.rippleState[base + 2] = (radius >= maxRadius) ? 0 : radius;
        }
        st.effectStep++;
    }

    static void twinkle(EffectState& st, IEffectSink& sink) {
        uint16_t count = sink.pixelCount();
        if (count == 0) return;
        if (st.twinkleState.size() != count) st.twinkleState.assign(count, 0);

        // intensity controls sparkle density (roughly 2%-40% ignite chance per frame)
        if (random(0, 100) < (2 + st.intensity / 6)) st.twinkleState[random(0, count)] = 255;

        for (uint16_t i = 0; i < count; i++) {
            uint8_t& fade = st.twinkleState[i];
            if (fade == 0) { emit(st, sink, i, 0, 0, 0); continue; }
            // Each position samples its own spot in the palette, for colour variety.
            uint32_t c = (st.palette == 0) ? st.color
                                           : paletteColor(st.palette, (uint8_t)((i * 256 / count) & 0xFF));
            uint8_t r = (uint8_t)((((c >> 16) & 0xFF) * bri(st) / 255) * fade / 255);
            uint8_t g = (uint8_t)((((c >> 8) & 0xFF) * bri(st) / 255) * fade / 255);
            uint8_t b = (uint8_t)(((c & 0xFF) * bri(st) / 255) * fade / 255);
            emit(st, sink, i, r, g, b);
            fade = (fade > 14) ? fade - 14 : 0;
        }
    }

    static void meteor(EffectState& st, IEffectSink& sink) {
        uint16_t count = sink.pixelCount();
        if (count == 0) return;
        uint8_t r, g, b; scaled(st.color, bri(st), r, g, b);

        for (uint16_t i = 0; i < count; i++) emit(st, sink, i, 0, 0, 0);

        uint16_t pos = st.effectStep % count;
        uint16_t trailLen = 2 + st.intensity / 32; // intensity controls trail length (2-9 pixels)
        for (uint16_t i = 0; i < trailLen; i++) {
            if (pos >= i) {
                uint16_t pixel = pos - i;
                if (pixel < count) emit(st, sink, pixel, r / (i + 1), g / (i + 1), b / (i + 1));
            }
        }
        st.effectStep++;
    }

    static void sinelon(EffectState& st, IEffectSink& sink) {
        uint16_t count = sink.pixelCount();
        if (count == 0) return;
        if (st.sinelonState.size() != count) st.sinelonState.assign(count, 0);

        // intensity controls trail length: higher fades slower, so the trail runs longer
        uint8_t decay = 2 + (255 - st.intensity) / 16;
        for (uint16_t i = 0; i < count; i++) {
            uint8_t& v = st.sinelonState[i];
            v = (v > decay) ? v - decay : 0;
        }

        uint16_t span = count > 1 ? count * 2 - 2 : 1;
        uint16_t raw = st.effectStep % span;
        uint16_t pos = raw >= count ? span - raw : raw;
        st.sinelonState[pos] = 255;

        uint8_t baseR = (st.color >> 16) & 0xFF;
        uint8_t baseG = (st.color >> 8) & 0xFF;
        uint8_t baseB = st.color & 0xFF;
        for (uint16_t i = 0; i < count; i++) {
            uint16_t v = st.sinelonState[i];
            emit(st, sink, i, (uint8_t)((uint32_t)baseR * bri(st) / 255 * v / 255),
                              (uint8_t)((uint32_t)baseG * bri(st) / 255 * v / 255),
                              (uint8_t)((uint32_t)baseB * bri(st) / 255 * v / 255));
        }
        st.effectStep++;
    }

    static void confetti(EffectState& st, IEffectSink& sink) {
        uint16_t count = sink.pixelCount();
        if (count == 0) return;
        if (st.confettiState.size() != count) {
            st.confettiState.assign(count, 0);
            st.confettiHue.assign(count, 0);
        }

        for (uint16_t i = 0; i < count; i++) {
            uint8_t& v = st.confettiState[i];
            v = (v > 10) ? v - 10 : 0;
        }
        // intensity controls spark density (roughly 3%-45% ignite chance per frame)
        if (random(0, 100) < (3 + st.intensity / 6)) {
            uint16_t idx = random(0, count);
            st.confettiState[idx] = 255;
            st.confettiHue[idx] = (uint8_t)random(0, 256);
        }

        for (uint16_t i = 0; i < count; i++) {
            uint16_t v = st.confettiState[i];
            if (v == 0) { emit(st, sink, i, 0, 0, 0); continue; }
            uint32_t c = (st.palette == 0) ? wheel(st.confettiHue[i])
                                           : paletteColor(st.palette, st.confettiHue[i]);
            emit(st, sink, i, (uint8_t)((((c >> 16) & 0xFF) * bri(st) / 255) * v / 255),
                              (uint8_t)((((c >> 8) & 0xFF) * bri(st) / 255) * v / 255),
                              (uint8_t)(((c & 0xFF) * bri(st) / 255) * v / 255));
        }
    }

    static void juggle(EffectState& st, IEffectSink& sink) {
        uint16_t count = sink.pixelCount();
        if (count == 0) return;
        if (st.juggleState.size() != (size_t)count * 3) st.juggleState.assign((size_t)count * 3, 0);

        for (uint16_t i = 0; i < count * 3; i++) {
            uint8_t& v = st.juggleState[i];
            v = (v > 20) ? v - 20 : 0;
        }

        // Dots blend additively into the trail buffer, so overlaps brighten rather than overwrite.
        const uint8_t numDots = 4;
        uint16_t span = count > 1 ? count * 2 - 2 : 1;
        for (uint8_t d = 0; d < numDots; d++) {
            uint16_t phase = (uint16_t)(st.effectStep * (d + 1)) % span;
            uint16_t pos = phase >= count ? span - phase : phase;

            uint32_t c = wheel((uint8_t)(d * 256 / numDots));
            uint16_t base = pos * 3;
            uint16_t nr = st.juggleState[base]     + ((c >> 16) & 0xFF);
            uint16_t ng = st.juggleState[base + 1] + ((c >> 8) & 0xFF);
            uint16_t nb = st.juggleState[base + 2] + (c & 0xFF);
            st.juggleState[base]     = nr > 255 ? 255 : (uint8_t)nr;
            st.juggleState[base + 1] = ng > 255 ? 255 : (uint8_t)ng;
            st.juggleState[base + 2] = nb > 255 ? 255 : (uint8_t)nb;
        }

        for (uint16_t i = 0; i < count; i++) {
            uint16_t base = i * 3;
            emit(st, sink, i, (uint8_t)((uint32_t)st.juggleState[base] * bri(st) / 255),
                              (uint8_t)((uint32_t)st.juggleState[base + 1] * bri(st) / 255),
                              (uint8_t)((uint32_t)st.juggleState[base + 2] * bri(st) / 255));
        }
        st.effectStep++;
    }

    static void bpm(EffectState& st, IEffectSink& sink) {
        uint16_t count = sink.pixelCount();
        if (count == 0) return;

        // The palette flows across the segment while the whole thing pulses, never fully dark.
        uint8_t pulse = triWave8((uint8_t)(st.effectStep & 0xFF));
        uint16_t beatBri = 40 + ((uint16_t)pulse * (255 - 40)) / 255;

        for (uint16_t i = 0; i < count; i++) {
            uint8_t huePos = (uint8_t)(((i * 256 / count) + (st.effectStep / 4)) & 0xFF);
            uint32_t c = (st.palette == 0) ? wheel(huePos) : paletteColor(st.palette, huePos);
            uint16_t scale = (uint16_t)bri(st) * beatBri / 255;
            emit(st, sink, i, (uint8_t)((((c >> 16) & 0xFF) * scale) / 255),
                              (uint8_t)((((c >> 8) & 0xFF) * scale) / 255),
                              (uint8_t)(((c & 0xFF) * scale) / 255));
        }
        st.effectStep += 2;
    }

    static void pacifica(EffectState& st, IEffectSink& sink) {
        uint16_t cw = sink.matrixWidth();
        uint16_t ch = sink.matrixHeight();
        if (cw == 0 || ch == 0) { colorWaves(st, sink); return; } // same 1D fallback as the Master

        uint8_t speedScale = 1 + st.intensity / 32; // nudges how fast the layers drift apart
        for (uint16_t y = 0; y < ch; y++) {
            for (uint16_t x = 0; x < cw; x++) {
                uint8_t w1 = triWave8((uint8_t)(x * 6 + y * 2 + st.effectStep));
                uint8_t w2 = triWave8((uint8_t)(x * 3 - y * 4 + st.effectStep * 2 * speedScale));
                uint8_t w3 = triWave8((uint8_t)(x * 8 + y * 5 - st.effectStep / 2));
                uint8_t pos = (uint8_t)(((uint16_t)w1 + w2 + w3) / 3);
                // Defaults to the Ocean palette: a rainbow fallback would defeat the point.
                uint32_t c = (st.palette == 0) ? paletteColor(3, pos) : paletteColor(st.palette, pos);
                uint8_t r, g, b; scaled(c, bri(st), r, g, b);
                sink.setPixelXY(x, y, r, g, b);
            }
        }
        st.effectStep++;
    }

    static void fireworks(EffectState& st, IEffectSink& sink) {
        uint16_t cw = sink.matrixWidth();
        uint16_t ch = sink.matrixHeight();
        if (cw == 0 || ch == 0) { meteor(st, sink); return; } // same 1D fallback as the Master

        const uint8_t numRockets = 2;
        if (st.fireworksState.size() != (size_t)numRockets * 6) {
            st.fireworksState.assign((size_t)numRockets * 6, 0);
            for (uint8_t i = 0; i < numRockets; i++) {
                st.fireworksState[i * 6 + 4] = (uint8_t)random(0, 40); // stagger initial launches
            }
        }

        for (uint16_t y = 0; y < ch; y++)
            for (uint16_t x = 0; x < cw; x++) sink.setPixelXY(x, y, 0, 0, 0);

        uint8_t maxRadius = 10 + st.intensity / 16; // intensity controls burst size

        for (uint8_t i = 0; i < numRockets; i++) {
            uint8_t base = i * 6;
            uint8_t rx = st.fireworksState[base];
            uint8_t ry = st.fireworksState[base + 1];
            uint8_t peakY = st.fireworksState[base + 2];
            uint8_t hue = st.fireworksState[base + 3];
            uint8_t timer = st.fireworksState[base + 4];
            uint8_t state = st.fireworksState[base + 5];

            if (state == 0) {
                if (timer == 0) { // waiting for the next launch
                    rx = (uint8_t)random(0, cw > 255 ? 255 : cw);
                    ry = (uint8_t)((ch > 255 ? 255 : ch) - 1);
                    peakY = (uint8_t)random(ch / 4, (ch * 2) / 3 + 1);
                    hue = (uint8_t)random(0, 256);
                    state = 1;
                } else {
                    timer--;
                }
            } else if (state == 1) { // rising
                uint32_t c = (st.palette == 0) ? wheel(hue) : paletteColor(st.palette, hue);
                uint8_t r, g, b; scaled(c, bri(st), r, g, b);
                sink.setPixelXY(rx, ry, r, g, b);
                if (ry <= peakY || ry == 0) { state = 2; timer = 0; }
                else { ry--; }
            } else { // exploding: an expanding, fading ring, same maths as Ripple
                uint32_t c = (st.palette == 0) ? wheel(hue) : paletteColor(st.palette, hue);
                uint8_t baseR = (c >> 16) & 0xFF, baseG = (c >> 8) & 0xFF, baseB = c & 0xFF;
                uint8_t radius = timer;
                for (uint16_t y = 0; y < ch; y++) {
                    for (uint16_t x = 0; x < cw; x++) {
                        int16_t dx = (int16_t)x - rx;
                        int16_t dy = (int16_t)y - ry;
                        uint16_t distSq = (uint16_t)(dx * dx + dy * dy);
                        uint16_t rSq = (uint16_t)radius * radius;
                        uint16_t rPrevSq = radius > 1 ? (uint16_t)(radius - 1) * (radius - 1) : 0;
                        if (distSq <= rSq && distSq > rPrevSq) {
                            uint8_t fade = 255 - (uint16_t)(radius * 255 / maxRadius);
                            sink.setPixelXY(x, y, (baseR * bri(st) / 255 * fade) / 255,
                                                  (baseG * bri(st) / 255 * fade) / 255,
                                                  (baseB * bri(st) / 255 * fade) / 255);
                        }
                    }
                }
                timer++;
                if (timer >= maxRadius) {
                    state = 0;
                    timer = (uint8_t)random(20, 80); // cooldown before the next launch
                }
            }

            st.fireworksState[base] = rx;
            st.fireworksState[base + 1] = ry;
            st.fireworksState[base + 2] = peakY;
            st.fireworksState[base + 3] = hue;
            st.fireworksState[base + 4] = timer;
            st.fireworksState[base + 5] = state;
        }
        st.effectStep++;
    }

    static void starfield(EffectState& st, IEffectSink& sink) {
        uint16_t cw = sink.matrixWidth();
        uint16_t ch = sink.matrixHeight();
        if (cw == 0 || ch == 0) { twinkle(st, sink); return; } // same 1D fallback as the Master

        const uint8_t numStars = 32;
        if (st.starfieldState.size() != (size_t)numStars * 3) {
            st.starfieldState.assign((size_t)numStars * 3, 0);
            for (uint8_t i = 0; i < numStars; i++) {
                int8_t dx = 0, dy = 0;
                while (dx == 0 && dy == 0) { dx = (int8_t)random(-8, 9); dy = (int8_t)random(-8, 9); }
                st.starfieldState[i * 3] = (uint8_t)dx;
                st.starfieldState[i * 3 + 1] = (uint8_t)dy;
                st.starfieldState[i * 3 + 2] = (uint8_t)random(0, 40); // stagger initial radii
            }
        }

        for (uint16_t y = 0; y < ch; y++)
            for (uint16_t x = 0; x < cw; x++) sink.setPixelXY(x, y, 0, 0, 0);

        int16_t centerX = cw / 2;
        int16_t centerY = ch / 2;
        uint16_t maxRadius = (cw > ch ? cw : ch);
        uint8_t speed = 1 + st.intensity / 32; // intensity controls warp speed

        uint8_t baseR = (st.color >> 16) & 0xFF;
        uint8_t baseG = (st.color >> 8) & 0xFF;
        uint8_t baseB = st.color & 0xFF;

        for (uint8_t i = 0; i < numStars; i++) {
            int8_t dx = (int8_t)st.starfieldState[i * 3];
            int8_t dy = (int8_t)st.starfieldState[i * 3 + 1];
            uint8_t radius = st.starfieldState[i * 3 + 2];

            int16_t px = centerX + ((int16_t)dx * radius) / 8;
            int16_t py = centerY + ((int16_t)dy * radius) / 8;

            if (px >= 0 && px < (int16_t)cw && py >= 0 && py < (int16_t)ch) {
                uint8_t fade = (uint16_t)radius * 255 / maxRadius;
                uint32_t c = (st.palette == 0)
                    ? (((uint32_t)baseR << 16) | ((uint32_t)baseG << 8) | baseB)
                    : paletteColor(st.palette, (uint8_t)(radius * 3));
                sink.setPixelXY(px, py, (uint8_t)((((c >> 16) & 0xFF) * bri(st) / 255) * fade / 255),
                                        (uint8_t)((((c >> 8) & 0xFF) * bri(st) / 255) * fade / 255),
                                        (uint8_t)(((c & 0xFF) * bri(st) / 255) * fade / 255));
            }

            uint16_t newRadius = (uint16_t)radius + speed;
            if (newRadius > maxRadius || px < -2 || px > (int16_t)cw + 2 ||
                py < -2 || py > (int16_t)ch + 2) {
                int8_t ndx = 0, ndy = 0;
                while (ndx == 0 && ndy == 0) { ndx = (int8_t)random(-8, 9); ndy = (int8_t)random(-8, 9); }
                st.starfieldState[i * 3] = (uint8_t)ndx;
                st.starfieldState[i * 3 + 1] = (uint8_t)ndy;
                st.starfieldState[i * 3 + 2] = 0;
            } else {
                st.starfieldState[i * 3 + 2] = (uint8_t)newRadius;
            }
        }
        st.effectStep++;
    }

    static void bouncingBalls(EffectState& st, IEffectSink& sink) {
        uint16_t cw = sink.matrixWidth();
        uint16_t ch = sink.matrixHeight();
        if (cw == 0 || ch == 0) { meteor(st, sink); return; } // same 1D fallback as the Master
        if (cw < 2 || ch < 2) return;

        const uint8_t numBalls = 4;
        if (st.ballsState.size() != (size_t)numBalls * 5) {
            st.ballsState.assign((size_t)numBalls * 5, 0);
            for (uint8_t i = 0; i < numBalls; i++) {
                uint8_t base = i * 5;
                st.ballsState[base] = (uint8_t)random(0, cw > 255 ? 255 : cw);
                st.ballsState[base + 1] = (uint8_t)random(0, ch > 255 ? 255 : ch);
                int8_t vx = (int8_t)((random(0, 2) ? 1 : -1) * (1 + random(0, 2)));
                int8_t vy = (int8_t)((random(0, 2) ? 1 : -1) * (1 + random(0, 2)));
                st.ballsState[base + 2] = (uint8_t)vx;
                st.ballsState[base + 3] = (uint8_t)vy;
                st.ballsState[base + 4] = (uint8_t)random(0, 256);
            }
        }

        for (uint16_t y = 0; y < ch; y++)
            for (uint16_t x = 0; x < cw; x++) sink.setPixelXY(x, y, 0, 0, 0);

        for (uint8_t i = 0; i < numBalls; i++) {
            uint8_t base = i * 5;
            int16_t x = st.ballsState[base];
            int16_t y = st.ballsState[base + 1];
            int8_t vx = (int8_t)st.ballsState[base + 2];
            int8_t vy = (int8_t)st.ballsState[base + 3];
            uint8_t hue = st.ballsState[base + 4];

            x += vx;
            y += vy;
            if (x <= 0) { x = 0; vx = (int8_t)(-vx); }
            if (x >= (int16_t)cw - 1) { x = cw - 1; vx = (int8_t)(-vx); }
            if (y <= 0) { y = 0; vy = (int8_t)(-vy); }
            if (y >= (int16_t)ch - 1) { y = ch - 1; vy = (int8_t)(-vy); }

            uint32_t c = (st.palette == 0) ? wheel(hue) : paletteColor(st.palette, hue);
            uint8_t r, g, b; scaled(c, bri(st), r, g, b);

            // Each ball leaves a small "+"-shaped glow around it.
            sink.setPixelXY(x, y, r, g, b);
            uint8_t dimR = r / 3, dimG = g / 3, dimB = b / 3;
            if (x > 0) sink.setPixelXY(x - 1, y, dimR, dimG, dimB);
            if (x < (int16_t)cw - 1) sink.setPixelXY(x + 1, y, dimR, dimG, dimB);
            if (y > 0) sink.setPixelXY(x, y - 1, dimR, dimG, dimB);
            if (y < (int16_t)ch - 1) sink.setPixelXY(x, y + 1, dimR, dimG, dimB);

            st.ballsState[base] = (uint8_t)x;
            st.ballsState[base + 1] = (uint8_t)y;
            st.ballsState[base + 2] = (uint8_t)vx;
            st.ballsState[base + 3] = (uint8_t)vy;
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
                sink.setPixelXY(x, y, r, g, b);
            }
        }
        st.effectStep++;
    }
};

inline const EffectEngine::PaletteDef EffectEngine::PALETTES[EffectEngine::PALETTE_SLOTS] = {
    {0, {}},                                            // Solid (unused)
    {4, {0xFF0000, 0xFFFF00, 0x00FF00, 0x0000FF}},      // Rainbow
    {3, {0x000000, 0xFF4500, 0xFFFF00}},                // Fire
    {3, {0x000033, 0x0077BE, 0x00FFFF}},                // Ocean
    {3, {0x013220, 0x228B22, 0x7CFC00}},                // Forest
};

#endif
