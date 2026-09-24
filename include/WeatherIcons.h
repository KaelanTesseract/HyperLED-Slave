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

// The pictograms of the "Wetter" element, in two sizes and in colour.
//
// Each icon is written as pixel art, one character per pixel, so it can be read and edited
// without a converter. The characters name a colour from the palette below; '.' is transparent.
// The element can show them in these colours or in its own colour - then the palette's brightness
// becomes the shade, which keeps the depth of the drawing (see WidgetRender).
//
// Icon ids 0-4 are the ones this element has always used, so a Slave with older firmware keeps
// showing the right picture for them; 5-8 came later and such a Slave simply leaves them blank.
//
// Keep this file identical in the Master and the Slave project, like HyperBus.h.

#define WEATHER_ICON_SMALL 11   // pixels per side
#define WEATHER_ICON_LARGE 16
#define WEATHER_ICON_COUNT 9

// Kept for the element's bounding box while it has no size of its own yet.
#define WEATHER_ICON_WIDTH WEATHER_ICON_SMALL
#define WEATHER_ICON_HEIGHT WEATHER_ICON_SMALL

struct WeatherIconColor {
    char key;
    uint8_t r, g, b;
};

// Dark tones carry the outline and the shadows; on a panel they are what gives the little
// pictures their shape.
static const WeatherIconColor WEATHER_ICON_PALETTE[] = {
    {'o', 0x33, 0x40, 0x4D},  // cloud outline, dark slate
    {'c', 0x5C, 0x6B, 0x7A},  // cloud body
    {'l', 0x9A, 0xAA, 0xB8},  // cloud highlight
    {'w', 0xFF, 0xFF, 0xFF},  // snow, stars
    {'y', 0xFF, 0xD2, 0x00},  // sun, lightning
    {'a', 0xFF, 0x9E, 0x00},  // sun shadow, lightning edge
    {'b', 0x2E, 0x8B, 0xFF},  // rain
    {'d', 0x15, 0x60, 0xC0},  // rain, darker drop
    {'m', 0xE8, 0xEE, 0xF5},  // moon
    {'g', 0xB9, 0xC6, 0xD4},  // moon shadow
};
#define WEATHER_ICON_PALETTE_SIZE (sizeof(WEATHER_ICON_PALETTE) / sizeof(WEATHER_ICON_PALETTE[0]))

// 0 sun, 1 cloud, 2 rain, 3 snow, 4 thunderstorm, 5 storm, 6 moon, 7 partly cloudy (day),
// 8 partly cloudy (night). See WEATHER_ICON_* in WeatherManager.h.
static const char* const WEATHER_ICONS_LARGE[WEATHER_ICON_COUNT][WEATHER_ICON_LARGE] = {
    {   // 0 sun
        ".......yy.......",
        "................",
        "..y..........y..",
        "................",
        "......yyyy......",
        ".....yyyyyy.....",
        "....yyyyyyyy....",
        "yy..yyyyyyyy..yy",
        "yy..yyyyyyyy..yy",
        "....yyyyyaaa....",
        ".....yyyaaa.....",
        "......yaaa......",
        "................",
        "..y..........y..",
        "................",
        ".......yy.......",
    },
    {   // 1 cloud
        "................",
        "................",
        "................",
        "...oooo.........",
        "..occcco........",
        ".occccccoo......",
        ".occcccccco.....",
        ".occcccccclooo..",
        "..ooooollllllo..",
        "...ollllllllllo.",
        "...ollllllllllo.",
        "...ollllllllllo.",
        "....oooooooooo..",
        "................",
        "................",
        "................",
    },
    {   // 2 rain
        "................",
        "......oooo......",
        "....ooccllooo...",
        "..ooccccccccoo..",
        ".occcccccccccco.",
        ".occcccccccccco.",
        "..oooooooooooo..",
        "................",
        "..b...b...b...b.",
        "..b...b...b...b.",
        "..d...d...d...d.",
        "................",
        "....b...b...b...",
        "....b...b...b...",
        "....d...d...d...",
        "................",
    },
    {   // 3 snow
        "................",
        "......oooo......",
        "....ooccllooo...",
        "..ooccccccccoo..",
        ".occcccccccccco.",
        ".occcccccccccco.",
        "..oooooooooooo..",
        "................",
        "..w.w.....w.w...",
        "...w.......w....",
        "..w.w.....w.w...",
        "................",
        ".......w.w......",
        "........w.......",
        ".......w.w......",
        "................",
    },
    {   // 4 thunderstorm
        "................",
        "......oooo......",
        "....ooccllooo...",
        "..ooccccccccoo..",
        ".occcccccccccco.",
        ".occcccccccccco.",
        "..oooooooooooo..",
        "................",
        "........yya.....",
        ".......yya......",
        "......yya.......",
        ".....yyyyya.....",
        ".......yya......",
        "......yya.......",
        ".....yya........",
        "................",
    },
    {   // 5 storm
        "................",
        "......oooo......",
        "....ooccllooo...",
        "..ooccccccccoo..",
        ".occcccccccccco.",
        ".occcccccccccco.",
        "..oooooooooooo..",
        "................",
        "..lllllllllo....",
        "...........l....",
        "..llllllllo.....",
        "..........l.....",
        "..lllllllo......",
        "................",
        "...b...b...b....",
        "................",
    },
    {   // 6 moon
        "................",
        "......mmmm......",
        "....mmmmmmm.....",
        "...mmmmggg......",
        "..mmmmg.........",
        "..mmmg.......w..",
        "..mmmg..........",
        "..mmmg.....w....",
        "..mmmmg.........",
        "...mmmmggg......",
        "....mmmmmmm.....",
        "......mmmm......",
        "..w.............",
        ".........w......",
        "................",
        "................",
    },
    {   // 7 partly cloudy, day
        "...y.....y......",
        "................",
        ".....yyyy.......",
        "....yyyyyy......",
        "y...yyyyyy...y..",
        "....yyayaa......",
        ".....yaaa.......",
        "................",
        "...y.....y......",
        "......oooo......",
        "....ooccllooo...",
        "..ooccccccccoo..",
        ".occcccccccccco.",
        ".occcccccccccco.",
        "..oooooooooooo..",
        "................",
    },
    {   // 8 partly cloudy, night
        "................",
        "....mmmm....w...",
        "...mmmggg.......",
        "..mmmg..........",
        "..mmmg......w...",
        "...mmmggg.......",
        "....mmmm........",
        "................",
        "...w............",
        "......oooo......",
        "....ooccllooo...",
        "..ooccccccccoo..",
        ".occcccccccccco.",
        ".occcccccccccco.",
        "..oooooooooooo..",
        "................",
    },
};

static const char* const WEATHER_ICONS_SMALL[WEATHER_ICON_COUNT][WEATHER_ICON_SMALL] = {
    {   // 0 sun
        ".....y.....",
        "..y.....y..",
        "....yyy....",
        "...yyyyy...",
        "y..yyyyy..y",
        "y..yyyyy..y",
        "...yyyaa...",
        "....yaa....",
        "..y.....y..",
        ".....y.....",
        "...........",
    },
    {   // 1 cloud
        "...........",
        "..ooo......",
        ".occco.....",
        ".occccoo...",
        ".occcclooo.",
        "..ooolllllo",
        "..ollllllo.",
        "..ollllllo.",
        "...oooooo..",
        "...........",
        "...........",
    },
    {   // 2 rain
        "....ooo....",
        "..occlloo..",
        ".occcccclo.",
        ".occccccco.",
        "..ooooooo..",
        "...........",
        ".b..b..b..b",
        ".d..d..d..d",
        "...........",
        "..b..b..b..",
        "..d..d..d..",
    },
    {   // 3 snow
        "....ooo....",
        "..occlloo..",
        ".occcccclo.",
        ".occccccco.",
        "..ooooooo..",
        "...........",
        ".w.w...w.w.",
        "..w.....w..",
        ".w.w...w.w.",
        "....w.w....",
        ".....w.....",
    },
    {   // 4 thunderstorm
        "....ooo....",
        "..occlloo..",
        ".occcccclo.",
        ".occccccco.",
        "..ooooooo..",
        "......yya..",
        ".....yya...",
        "....yyyya..",
        ".....yya...",
        "....yya....",
        "...........",
    },
    {   // 5 storm
        "....ooo....",
        "..occlloo..",
        ".occcccclo.",
        ".occccccco.",
        "..ooooooo..",
        "...........",
        ".lllllo....",
        "......l....",
        ".llllo.....",
        "...........",
        "..b..b..b..",
    },
    {   // 6 moon
        "....mmm....",
        "...mmggg...",
        "..mmg......",
        "..mmg....w.",
        "..mmg......",
        "..mmg......",
        "...mmggg...",
        "....mmm....",
        ".w.........",
        "........w..",
        "...........",
    },
    {   // 7 partly cloudy, day
        "..y.....y..",
        "....yyy....",
        "y...yyy...y",
        "....yaa....",
        "..y.....y..",
        "....ooo....",
        "..occlloo..",
        ".occcccclo.",
        ".occccccco.",
        "..ooooooo..",
        "...........",
    },
    {   // 8 partly cloudy, night
        "...mmm...w.",
        "..mmggg....",
        "..mmg......",
        "..mmg...w..",
        "...mmggg...",
        "....ooo....",
        "..occlloo..",
        ".occcccclo.",
        ".occccccco.",
        "..ooooooo..",
        "...........",
    },
};

// The colour of one pixel. Returns false where the icon is transparent. large picks the 16x16
// set, otherwise the 11x11 one.
inline bool weather_icon_pixel(uint8_t iconId, uint8_t col, uint8_t row, bool large,
                               uint8_t& r, uint8_t& g, uint8_t& b) {
    uint8_t side = large ? WEATHER_ICON_LARGE : WEATHER_ICON_SMALL;
    if (iconId >= WEATHER_ICON_COUNT || row >= side || col >= side) return false;
    char key = large ? WEATHER_ICONS_LARGE[iconId][row][col] : WEATHER_ICONS_SMALL[iconId][row][col];
    if (key == '.') return false;
    for (size_t i = 0; i < WEATHER_ICON_PALETTE_SIZE; i++) {
        if (WEATHER_ICON_PALETTE[i].key != key) continue;
        r = WEATHER_ICON_PALETTE[i].r;
        g = WEATHER_ICON_PALETTE[i].g;
        b = WEATHER_ICON_PALETTE[i].b;
        return true;
    }
    return false;
}
