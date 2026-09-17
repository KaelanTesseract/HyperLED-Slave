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

// How an "Uhr / Text" element is drawn - clock, date, text, image, analog clock, weather and
// Lauftext - in one place that both the Master and a HUB75 Slave use.
//
// Two identical copies exist, one per repository (include/WidgetRender.h on the Master,
// HyperLED_Slave/include/WidgetRender.h on the Slave), exactly like HyperBus.h. Keep them identical:
// the Master draws an element whenever a Slave cannot, and the two drawings must match pixel for
// pixel, or an element would change its look depending on which side happened to draw it.
//
// Nothing in here knows about segments, buses or panels. The caller passes a plot function that
// receives already-clipped coordinates and already-dimmed colours.

#include <Arduino.h>
#include <time.h>
#include <math.h>
#include "Font5x7.h"
#include "Font3x5.h"
#include "WeatherIcons.h"

namespace WidgetRender {

static const uint8_t TYPE_CLOCK = 0;
static const uint8_t TYPE_DATE = 1;
static const uint8_t TYPE_TEXT = 2;
static const uint8_t TYPE_IMAGE = 3;
static const uint8_t TYPE_ANALOG = 4;
static const uint8_t TYPE_WEATHER = 5;
static const uint8_t TYPE_MARQUEE = 6;

static const uint8_t SCALE_MAX = 8;

// How an element stays readable in front of a background effect.
static const uint8_t LEGIBLE_NONE = 0;     // drawn straight onto the effect
static const uint8_t LEGIBLE_OUTLINE = 1;  // a dark ring, one pixel wide, around every lit pixel
static const uint8_t LEGIBLE_BOX = 2;      // the effect is darkened behind the element's area
static const uint8_t LEGIBLE_MAX = 2;

// Image pixels with every channel below this are not drawn (see draw()).
static const uint8_t IMAGE_TRANSPARENT_BELOW = 12;

// One element, independent of how either side stores it.
struct Spec {
    uint8_t type;
    int16_t x;
    int16_t y;
    uint32_t color;        // 0xRRGGBB; unused by images
    uint8_t scale;         // 1..SCALE_MAX
    uint8_t format;        // clock/date layout, analog design, weather layout, marquee direction
    uint8_t font;          // 0 = 5x7, 1 = 3x5
    uint8_t speed;         // Lauftext scroll speed
    uint8_t width;         // image width, analog diameter, Lauftext window width
    uint8_t height;        // image height
    const char* text;      // custom text / Lauftext
    uint16_t textLen;
    const uint8_t* img;    // RGB triplets, width * height * 3 bytes
    size_t imgLen;
    uint8_t bri = 255;     // the element's own brightness, on top of the segment's
};

// The local wall-clock time to draw, already in the Master's time zone.
struct Clock {
    bool known;            // false: nothing to show yet (a Slave that has not been told the time)
    struct tm tm;
};

struct Weather {
    bool valid;            // false: "--`C", as the Master shows before its first forecast
    int16_t temp;          // whole degrees, already rounded
    uint8_t icon;          // WEATHER_ICON_* index
};

struct Rect {
    int16_t x;
    int16_t y;
    int16_t w;
    int16_t h;
};

static const char* const WEEKDAY_ABBR[7] = {"SO", "MO", "DI", "MI", "DO", "FR", "SA"};

// --- Time without time zones -----------------------------------------------------------------
// The Slave has no time zone rules and no NTP. It is sent the Master's *local* time as a count of
// seconds that, read as UTC, gives the local calendar fields - so gmtime_r() on the Slave yields
// exactly what localtime_r() gave on the Master, daylight saving included.

// Days since 1970-01-01 for a proleptic Gregorian date (H. Hinnant's days_from_civil).
inline int32_t daysFromCivil(int32_t y, uint32_t m, uint32_t d) {
    y -= m <= 2;
    const int32_t era = (y >= 0 ? y : y - 399) / 400;
    const uint32_t yoe = (uint32_t)(y - era * 400);
    const uint32_t doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
    const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int32_t)doe - 719468;
}

inline uint32_t localEpoch(const struct tm& t) {
    int32_t days = daysFromCivil(t.tm_year + 1900, (uint32_t)(t.tm_mon + 1), (uint32_t)t.tm_mday);
    return (uint32_t)days * 86400UL + (uint32_t)t.tm_hour * 3600UL + (uint32_t)t.tm_min * 60UL +
           (uint32_t)t.tm_sec;
}

inline void fromLocalEpoch(uint32_t epoch, struct tm& out) {
    time_t t = (time_t)epoch;
    gmtime_r(&t, &out);
}

// --- Geometry --------------------------------------------------------------------------------

inline uint8_t clampScale(uint8_t s) {
    if (s < 1) return 1;
    if (s > SCALE_MAX) return SCALE_MAX;
    return s;
}

struct Glyphs {
    uint8_t w;
    uint8_t h;
    bool (*pixel)(char, uint8_t, uint8_t);
};

inline Glyphs glyphs(uint8_t font) {
    Glyphs g;
    bool mini = (font == 1);
    g.w = mini ? FONT3X5_GLYPH_WIDTH : FONT5X7_GLYPH_WIDTH;
    g.h = mini ? FONT3X5_GLYPH_HEIGHT : FONT5X7_GLYPH_HEIGHT;
    g.pixel = mini ? font3x5_pixel : font5x7_pixel;
    return g;
}

// The text a clock or date element shows. Returns its length.
inline uint8_t clockText(const Spec& s, const struct tm& t, char* buf, size_t n) {
    if (s.type == TYPE_CLOCK) {
        switch (s.format) {
            case 1: // HH:MM:SS
                snprintf(buf, n, "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
                break;
            case 2: { // 12h AM/PM
                int h12 = t.tm_hour % 12;
                if (h12 == 0) h12 = 12;
                snprintf(buf, n, "%02d:%02d%s", h12, t.tm_min, t.tm_hour < 12 ? "AM" : "PM");
                break;
            }
            default: // HH:MM
                snprintf(buf, n, "%02d:%02d", t.tm_hour, t.tm_min);
                break;
        }
    } else {
        switch (s.format) {
            case 1: // DD.MM.YYYY
                snprintf(buf, n, "%02d.%02d.%04d", t.tm_mday, t.tm_mon + 1, t.tm_year + 1900);
                break;
            case 2: // DD.MM.YY
                snprintf(buf, n, "%02d.%02d.%02d", t.tm_mday, t.tm_mon + 1, (t.tm_year + 1900) % 100);
                break;
            case 3: // YYYY-MM-DD (ISO)
                snprintf(buf, n, "%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
                break;
            case 4: // MM/DD/YYYY (US)
                snprintf(buf, n, "%02d/%02d/%04d", t.tm_mon + 1, t.tm_mday, t.tm_year + 1900);
                break;
            case 5: // Wochentag DD.MM.
                snprintf(buf, n, "%s %02d.%02d.", WEEKDAY_ABBR[t.tm_wday % 7], t.tm_mday, t.tm_mon + 1);
                break;
            default: // DD.MM.
                snprintf(buf, n, "%02d.%02d.", t.tm_mday, t.tm_mon + 1);
                break;
        }
    }
    return (uint8_t)strlen(buf);
}

// The longest text a clock/date layout can produce - the element's area must not change size as
// the digits change, or a shorter text would leave the tail of the longer one standing.
inline uint8_t clockTextMax(const Spec& s) {
    if (s.type == TYPE_CLOCK) return s.format == 1 ? 8 : (s.format == 2 ? 7 : 5);
    switch (s.format) {
        case 1: case 3: case 4: return 10;
        case 2: return 8;
        case 5: return 9;
        default: return 6;
    }
}

static const uint8_t WEATHER_TEXT_MAX = 5; // "-40`C"

inline int16_t textWidth(uint16_t chars, const Glyphs& g, uint8_t scale) {
    if (chars == 0) return 0;
    int32_t w = (int32_t)chars * (g.w + 1) * scale - scale; // no gap after the last glyph
    return (int16_t)(w > 4096 ? 4096 : w);
}

// The area an element draws into, and the only area it ever touches.
inline Rect bounds(const Spec& s) {
    uint8_t scale = clampScale(s.scale);
    Glyphs g = glyphs(s.font);
    Rect r = {s.x, s.y, 0, 0};
    switch (s.type) {
        case TYPE_TEXT:
            r.w = textWidth(s.textLen, g, scale);
            r.h = (int16_t)(g.h * scale);
            break;
        case TYPE_MARQUEE:
            r.w = s.textLen == 0 ? 0 : (int16_t)(s.width > 0 ? s.width : 32);
            r.h = (int16_t)(g.h * scale);
            break;
        case TYPE_CLOCK:
        case TYPE_DATE:
            r.w = textWidth(clockTextMax(s), g, scale);
            r.h = (int16_t)(g.h * scale);
            break;
        case TYPE_ANALOG: {
            int16_t d = s.width > 0 ? s.width : 16;
            if (d < 8) d = 8;
            r.w = d + 1; // the rounded outline can land on x + d
            r.h = d + 1;
            break;
        }
        case TYPE_WEATHER: {
            int16_t w = 0, h = 0;
            if (s.format != 2) {
                w += (int16_t)(WEATHER_ICON_WIDTH * scale);
                h = (int16_t)(WEATHER_ICON_HEIGHT * scale);
            }
            if (s.format != 1) {
                if (s.format != 2) w += (int16_t)scale; // the gap after the icon
                w += textWidth(WEATHER_TEXT_MAX, g, scale);
                int16_t th = (int16_t)(g.h * scale);
                if (th > h) h = th;
            }
            r.w = w;
            r.h = h;
            break;
        }
        case TYPE_IMAGE:
            r.w = (int16_t)(s.width * scale);
            r.h = (int16_t)(s.height * scale);
            break;
        default:
            break;
    }
    return r;
}

// --- Drawing ---------------------------------------------------------------------------------

inline uint8_t dim(uint32_t color, uint8_t shift, uint8_t bri) {
    return (uint8_t)(((color >> shift) & 0xFF) * bri / 255);
}

// One run of text starting at (startX, y), clipped to the surface. Same loop as the Master always
// used for clock, date and custom text.
template <typename Plot>
inline void drawTextRun(const char* text, uint16_t len, int16_t startX, int16_t y, const Glyphs& g,
                        uint8_t scale, uint16_t cw, uint16_t ch, uint8_t r, uint8_t gr, uint8_t b,
                        Plot& plot) {
    const uint16_t charAdvance = (uint16_t)((g.w + 1) * scale);
    for (uint16_t i = 0; i < len; i++) {
        int16_t charX = (int16_t)(startX + i * charAdvance);
        if (charX + g.w * scale < 0 || charX >= (int16_t)cw) continue;
        for (uint8_t col = 0; col < g.w; col++) {
            for (uint8_t row = 0; row < g.h; row++) {
                if (!g.pixel(text[i], col, row)) continue;
                for (uint8_t sy = 0; sy < scale; sy++) {
                    int16_t py = y + (int16_t)row * scale + sy;
                    if (py < 0 || py >= (int16_t)ch) continue;
                    for (uint8_t sx = 0; sx < scale; sx++) {
                        int16_t px = charX + (int16_t)col * scale + sx;
                        if (px < 0 || px >= (int16_t)cw) continue;
                        plot(px, py, r, gr, b);
                    }
                }
            }
        }
    }
}

// Draws one element onto a surface of cw x ch pixels. bri dims every colour (0-255). nowMs drives
// the Lauftext scroll, which both sides compute from their own millis(): its position is not
// meant to match between devices, only its speed.
template <typename Plot>
inline void draw(const Spec& s, uint8_t bri, uint16_t cw, uint16_t ch, const Clock& clk,
                 const Weather& wx, unsigned long nowMs, Plot plot) {
    if (cw == 0 || ch == 0) return;
    if (s.bri < 255) {
        // Rounded, and never down to nothing while both are on: a dimmed element stays visible.
        uint16_t both = ((uint16_t)bri * s.bri + 127) / 255;
        bri = (both == 0 && bri > 0 && s.bri > 0) ? 1 : (uint8_t)both;
    }
    uint8_t scale = clampScale(s.scale);
    uint8_t r = dim(s.color, 16, bri);
    uint8_t g = dim(s.color, 8, bri);
    uint8_t b = dim(s.color, 0, bri);

    if (s.type == TYPE_WEATHER) {
        // tw.format: 0 = icon + temperature, 1 = icon only, 2 = temperature only.
        int16_t curX = s.x;
        if (s.format != 2) {
            uint8_t icon = wx.icon;
            for (uint8_t col = 0; col < WEATHER_ICON_WIDTH; col++) {
                for (uint8_t row = 0; row < WEATHER_ICON_HEIGHT; row++) {
                    if (!weather_icon_pixel(icon, col, row)) continue;
                    for (uint8_t sy = 0; sy < scale; sy++) {
                        int16_t py = s.y + (int16_t)row * scale + sy;
                        if (py < 0 || py >= (int16_t)ch) continue;
                        for (uint8_t sx = 0; sx < scale; sx++) {
                            int16_t px = curX + (int16_t)col * scale + sx;
                            if (px < 0 || px >= (int16_t)cw) continue;
                            plot(px, py, r, g, b);
                        }
                    }
                }
            }
            curX += (int16_t)((WEATHER_ICON_WIDTH + 1) * scale);
        }
        if (s.format != 1) {
            char wbuf[8];
            if (wx.valid) {
                snprintf(wbuf, sizeof(wbuf), "%d`C", (int)wx.temp);
            } else {
                snprintf(wbuf, sizeof(wbuf), "--`C");
            }
            Glyphs gl = glyphs(s.font);
            drawTextRun(wbuf, (uint16_t)strlen(wbuf), curX, s.y, gl, scale, cw, ch, r, g, b, plot);
        }
        return;
    }

    if (s.type == TYPE_ANALOG) {
        if (!clk.known) return;
        // Drawn with trig instead of a font; width doubles as the face diameter and format picks
        // the design: 0 = Klassisch (outline + ticks), 1 = Minimal (hands only),
        // 2 = dots + second hand, 3 = Kreuz (outline + 12/3/6/9 ticks).
        uint16_t diameter = s.width > 0 ? s.width : 16;
        if (diameter < 8) diameter = 8;
        float radius = diameter / 2.0f;
        float ccx = s.x + radius;
        float ccy = s.y + radius;

        auto plotF = [&](float px, float py) {
            int16_t ix = (int16_t)lroundf(px);
            int16_t iy = (int16_t)lroundf(py);
            if (ix < 0 || ix >= (int16_t)cw || iy < 0 || iy >= (int16_t)ch) return;
            plot(ix, iy, r, g, b);
        };
        auto plotHand = [&](float angleDeg, float len) {
            float rad = angleDeg * (float)PI / 180.0f;
            float dx = sinf(rad), dy = -cosf(rad);
            int steps = (int)len + 1;
            for (int st = 0; st <= steps; st++) {
                float t = (float)st * len / steps;
                plotF(ccx + dx * t, ccy + dy * t);
            }
        };

        if (s.format == 0 || s.format == 3) { // face outline
            for (float a = 0; a < 360.0f; a += 2.0f) {
                float rad = a * (float)PI / 180.0f;
                plotF(ccx + radius * sinf(rad), ccy - radius * cosf(rad));
            }
        }
        // Ticks are short inward spokes rather than single points on the outline, so they read
        // as marks instead of blending into it.
        if (s.format == 0) { // 12 short ticks
            for (int i = 0; i < 12; i++) {
                float rad = i * 30.0f * (float)PI / 180.0f;
                float sn = sinf(rad), cs = cosf(rad);
                for (float rr = radius * 0.75f; rr <= radius; rr += 1.0f) {
                    plotF(ccx + rr * sn, ccy - rr * cs);
                }
            }
        } else if (s.format == 2) { // 12 dots, no outline
            for (int i = 0; i < 12; i++) {
                float rad = i * 30.0f * (float)PI / 180.0f;
                plotF(ccx + radius * sinf(rad), ccy - radius * cosf(rad));
            }
        } else if (s.format == 3) { // 4 long ticks
            for (int i = 0; i < 4; i++) {
                float rad = i * 90.0f * (float)PI / 180.0f;
                float sn = sinf(rad), cs = cosf(rad);
                for (float rr = radius * 0.55f; rr <= radius; rr += 1.0f) {
                    plotF(ccx + rr * sn, ccy - rr * cs);
                }
            }
        }

        const struct tm& t = clk.tm;
        plotHand(((t.tm_hour % 12) + t.tm_min / 60.0f) * 30.0f, radius * 0.5f);
        plotHand(t.tm_min * 6.0f, radius * 0.85f);
        if (s.format == 2) plotHand(t.tm_sec * 6.0f, radius * 0.9f);
        return;
    }

    if (s.type == TYPE_IMAGE) {
        if (!s.img || s.imgLen != (size_t)s.width * s.height * 3) return;
        for (uint16_t iy = 0; iy < s.height; iy++) {
            for (uint16_t ix = 0; ix < s.width; ix++) {
                size_t off = ((size_t)iy * s.width + ix) * 3;
                // Black is see-through, and so is the near-black noise many converted pictures
                // carry: invisible on a dark panel, it would show as a solid square over a
                // background effect.
                if (s.img[off] < IMAGE_TRANSPARENT_BELOW && s.img[off + 1] < IMAGE_TRANSPARENT_BELOW &&
                    s.img[off + 2] < IMAGE_TRANSPARENT_BELOW) {
                    continue;
                }
                uint8_t pr = (uint16_t)s.img[off] * bri / 255;
                uint8_t pg = (uint16_t)s.img[off + 1] * bri / 255;
                uint8_t pb = (uint16_t)s.img[off + 2] * bri / 255;
                if (!pr && !pg && !pb) continue;
                for (uint8_t sy = 0; sy < scale; sy++) {
                    int16_t py = s.y + (int16_t)iy * scale + sy;
                    if (py < 0 || py >= (int16_t)ch) continue;
                    for (uint8_t sx = 0; sx < scale; sx++) {
                        int16_t px = s.x + (int16_t)ix * scale + sx;
                        if (px < 0 || px >= (int16_t)cw) continue;
                        plot(px, py, pr, pg, pb);
                    }
                }
            }
        }
        return;
    }

    if (s.type == TYPE_MARQUEE) {
        // The text scrolls through a fixed window (width pixels) instead of being clipped like a
        // static text; format picks the direction (0 = left, 1 = right).
        if (s.textLen == 0 || !s.text) return;
        uint16_t boxW = s.width > 0 ? s.width : 32;
        int16_t boxLeft = s.x;
        int16_t boxRight = (int16_t)(s.x + boxW);
        Glyphs gl = glyphs(s.font);
        uint16_t charAdvance = (uint16_t)((gl.w + 1) * scale);
        uint32_t textPxW = (uint32_t)s.textLen * charAdvance;
        uint32_t cycle = textPxW + boxW; // the text fully leaves the window before it repeats

        // Speed 0-255 -> ~2-30 px/s; the floor keeps the text legible even at speed 0.
        uint32_t pxPerSec = 2 + ((uint32_t)s.speed * 28) / 255;
        uint32_t offset = (uint32_t)(((uint64_t)nowMs * pxPerSec) / 1000) % cycle;
        int32_t textX = (s.format == 1)
            ? (int32_t)s.x - (int32_t)textPxW + (int32_t)offset  // scrolls right
            : (int32_t)s.x + (int32_t)boxW - (int32_t)offset;    // scrolls left

        for (uint16_t i = 0; i < s.textLen; i++) {
            int32_t charX = textX + (int32_t)(i * charAdvance);
            if (charX + gl.w * scale <= boxLeft || charX >= boxRight) continue;
            for (uint8_t col = 0; col < gl.w; col++) {
                for (uint8_t row = 0; row < gl.h; row++) {
                    if (!gl.pixel(s.text[i], col, row)) continue;
                    for (uint8_t sy = 0; sy < scale; sy++) {
                        int16_t py = s.y + (int16_t)row * scale + sy;
                        if (py < 0 || py >= (int16_t)ch) continue;
                        for (uint8_t sx = 0; sx < scale; sx++) {
                            int32_t px = charX + (int32_t)col * scale + sx;
                            if (px < boxLeft || px >= boxRight) continue;
                            if (px < 0 || px >= (int32_t)cw) continue;
                            plot((int16_t)px, py, r, g, b);
                        }
                    }
                }
            }
        }
        return;
    }

    // Clock, date, custom text.
    char buf[16];
    const char* text = s.text;
    uint16_t len = s.textLen;
    if (s.type == TYPE_CLOCK || s.type == TYPE_DATE) {
        if (!clk.known) return;
        len = clockText(s, clk.tm, buf, sizeof(buf));
        text = buf;
    } else if (s.type != TYPE_TEXT) {
        return;
    }
    if (len == 0 || !text) return;
    Glyphs gl = glyphs(s.font);
    drawTextRun(text, len, s.x, s.y, gl, scale, cw, ch, r, g, b, plot);
}

// Draws elements over a background that is already in `frame` (RGB, cw x ch). First every
// element darkens its surroundings as its legibility setting says, then all of them are drawn -
// clearing everything first keeps one element's ring from cutting into its neighbour.
// specAt(i) / legibleAt(i) give element i; `mask` is cw * ch bytes of scratch space.
template <typename SpecAt, typename LegibleAt>
inline void composeOver(uint8_t* frame, uint8_t* mask, uint16_t cw, uint16_t ch, size_t count,
                        SpecAt specAt, LegibleAt legibleAt, uint8_t bri, const Clock& clk,
                        const Weather& wx, unsigned long nowMs) {
    if (!frame || !mask || cw == 0 || ch == 0) return;
    const uint8_t LIT = 1, BLACK = 2, DIM = 4;
    size_t px = (size_t)cw * ch;
    memset(mask, 0, px);
    bool any = false;
    for (size_t i = 0; i < count; i++) {
        uint8_t legible = legibleAt(i);
        if (legible == LEGIBLE_NONE || legible > LEGIBLE_MAX) continue;
        Spec s = specAt(i);
        any = true;
        if (legible == LEGIBLE_BOX) {
            Rect r = bounds(s);
            for (int32_t y = (int32_t)r.y - 1; y <= (int32_t)r.y + r.h; y++) {
                if (y < 0 || y >= (int32_t)ch) continue;
                for (int32_t x = (int32_t)r.x - 1; x <= (int32_t)r.x + r.w; x++) {
                    if (x < 0 || x >= (int32_t)cw) continue;
                    mask[(size_t)y * cw + x] |= DIM;
                }
            }
        } else {
            auto mark = [&](int16_t x, int16_t y, uint8_t, uint8_t, uint8_t) {
                mask[(size_t)y * cw + x] |= LIT;
            };
            draw(s, 255, cw, ch, clk, wx, nowMs, mark);
        }
    }
    if (any) {
        // The ring: every neighbour of a lit pixel goes dark.
        for (uint16_t y = 0; y < ch; y++) {
            for (uint16_t x = 0; x < cw; x++) {
                if (!(mask[(size_t)y * cw + x] & LIT)) continue;
                for (int dy = -1; dy <= 1; dy++) {
                    int32_t ny = (int32_t)y + dy;
                    if (ny < 0 || ny >= (int32_t)ch) continue;
                    for (int dx = -1; dx <= 1; dx++) {
                        int32_t nx = (int32_t)x + dx;
                        if (nx < 0 || nx >= (int32_t)cw) continue;
                        mask[(size_t)ny * cw + nx] |= BLACK;
                    }
                }
            }
        }
        for (size_t p = 0; p < px; p++) {
            uint8_t m = mask[p];
            if (m & BLACK) {
                frame[p * 3] = frame[p * 3 + 1] = frame[p * 3 + 2] = 0;
            } else if (m & DIM) {
                // A box keeps a trace of the effect - an eighth - so it reads as a dark panel,
                // not a hole.
                frame[p * 3] >>= 3;
                frame[p * 3 + 1] >>= 3;
                frame[p * 3 + 2] >>= 3;
            }
        }
    }
    auto plot = [&](int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b) {
        size_t o = ((size_t)y * cw + x) * 3;
        frame[o] = r;
        frame[o + 1] = g;
        frame[o + 2] = b;
    };
    for (size_t i = 0; i < count; i++) {
        draw(specAt(i), bri, cw, ch, clk, wx, nowMs, plot);
    }
}

} // namespace WidgetRender
