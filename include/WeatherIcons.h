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

// Small 7x7 pictograms for the "Wetter" widget's condition icon. Indices
// match WEATHER_ICON_* in WeatherManager.h. Each glyph is 7 rows of 7 bits
// (bit 6 = leftmost column, bit 0 = rightmost column).
static const uint8_t WEATHER_ICONS[][7] = {
    {0x00,0x2A,0x1C,0x7F,0x1C,0x2A,0x00}, // Sun
    {0x00,0x1C,0x3E,0x7F,0x7F,0x00,0x00}, // Cloud
    {0x1C,0x3E,0x7F,0x00,0x55,0x2A,0x55}, // Rain
    {0x1C,0x3E,0x7F,0x00,0x2A,0x41,0x2A}, // Snow
    {0x1C,0x3E,0x7F,0x08,0x18,0x30,0x40}, // Thunderstorm
};
#define WEATHER_ICON_WIDTH 7
#define WEATHER_ICON_HEIGHT 7
#define WEATHER_ICON_COUNT 5

// Returns true if the pixel at (col, row) of the given icon is lit. col/row
// are 0-based, col 0 = leftmost, row 0 = topmost. Out-of-range returns false.
inline bool weather_icon_pixel(uint8_t iconId, uint8_t col, uint8_t row) {
    if (iconId >= WEATHER_ICON_COUNT || row >= WEATHER_ICON_HEIGHT || col >= WEATHER_ICON_WIDTH) return false;
    return (WEATHER_ICONS[iconId][row] >> (WEATHER_ICON_WIDTH - 1 - col)) & 1;
}
