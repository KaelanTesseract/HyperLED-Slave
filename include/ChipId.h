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

// Which chip a firmware is built for, and the check that an image is built for it. Identical in the
// Master and the Slave repository (like HyperBus.h).

#include <Arduino.h>

// The chip names the files of a release: firmware-<chip>.bin and littlefs-<chip>.bin (see
// UpdateManager and updatefeature.md). HYPERLED_CHIP_ID is the number the header of a firmware image
// carries for the same chip (esp_chip_id_t).
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define HYPERLED_CHIP "esp32s3"
#define HYPERLED_CHIP_ID 0x0009
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
#define HYPERLED_CHIP "esp32c6"
#define HYPERLED_CHIP_ID 0x000D
#elif defined(CONFIG_IDF_TARGET_ESP32)
#define HYPERLED_CHIP "esp32"
#define HYPERLED_CHIP_ID 0x0000
#else
#error "ChipId.h: unknown chip, name it here"
#endif

// The first 16 bytes of a firmware image (esp_image_header_t) name the chip it was built for: the
// chip id sits at byte 12 and 13, little endian. A controller must never take an image for another
// chip: it would not boot, and a Slave that is only reachable over the air would be lost with it.
// This checks the magic byte and the chip; nothing else about the image.
inline bool firmwareImageIsForThisChip(const uint8_t* head, size_t len) {
    return len >= 16 && head[0] == 0xE9 && (uint16_t)(head[12] | (head[13] << 8)) == HYPERLED_CHIP_ID;
}
