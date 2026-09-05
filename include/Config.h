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

#define HYPERLED_VERSION "0.1.009"

// --- Preferences Namespaces & Keys ---
#define PREF_NAMESPACE "wled_clone"

// ABL
#define PREF_ABL_ENABLE "abl_en"
#define PREF_ABL_MA "abl_ma"

// WiFi
#define PREF_WIFI_SSID "wifi_ssid"
#define PREF_WIFI_PASS "wifi_pass"

// LED Settings
#define PREF_LED_PIN_0 "led_pin0"
#define PREF_LED_PIN_1 "led_pin1"
#define PREF_LED_PIN_2 "led_pin2"
#define PREF_LED_PIN_3 "led_pin3"
#define PREF_LED_PIN_4 "led_pin4"
#define PREF_LED_COUNT "led_count"
#define PREF_LED_TYPE "led_type"

// MQTT Settings
#define PREF_MQTT_ENABLE "mqtt_en"
#define PREF_MQTT_SERVER "mqtt_srv"
#define PREF_MQTT_PORT "mqtt_port"
#define PREF_MQTT_USER "mqtt_user"
#define PREF_MQTT_PASS "mqtt_pass"
#define PREF_MQTT_TOPIC "mqtt_top"

// LED Types
#define TYPE_WS2812_RGB 22
#define TYPE_SK6812_RGBW 30
#define TYPE_TM1814 31
#define TYPE_400KHZ 24
#define TYPE_TM1829 26
#define TYPE_UCS8903 27
#define TYPE_APA106 28
#define TYPE_TM1914 29
#define TYPE_FW1906 32
#define TYPE_UCS8904 33
#define TYPE_WS2805 34
#define TYPE_SM16825 35
#define TYPE_WS2811_W 36
#define TYPE_WS281X_WWA 37

#define TYPE_WS2801 50
#define TYPE_APA102 51
#define TYPE_LPD8806 52
#define TYPE_LPD6803 53
#define TYPE_PP9813 54

#define TYPE_ONOFF 40

#define TYPE_ANALOG_1CH 41 // White
#define TYPE_ANALOG_2CH 42 // CCT
#define TYPE_ANALOG_3CH 43 // RGB
#define TYPE_ANALOG_4CH 44 // RGBW
#define TYPE_ANALOG_5CH 45 // RGB+CCT

#define TYPE_HUB75 60

// Same fixed HUB75 pinout as the Master's Config.h - both boards are the same
// Waveshare ESP32-S3-Zero (ESP32-S3FH4R2, embedded flash + 2MB quad-SPI PSRAM).
// GPIO33-37 aren't broken out on this board (Waveshare reserves them for the
// in-package flash/PSRAM),
// GPIO0/3/45/46 are strapping pins, GPIO19/20 are native USB, GPIO21 drives
// the onboard WS2812, and GPIO43/44 are the debug UART - all avoided here.
// GPIO16/17 (UPLINK_RX/TX, main.cpp) and GPIO18/38 (DOWNLINK_RX/TX) are left
// free too, so a HUB75 Slave keeps full use of both wired HyperBus ports.
// Keep in sync with the Master's own include/Config.h - same board, same
// pins, but a separate copy (not a shared header).
#define HUB75_PIN_R1 1
#define HUB75_PIN_G1 2
#define HUB75_PIN_B1 4
#define HUB75_PIN_R2 5
#define HUB75_PIN_G2 6
#define HUB75_PIN_B2 7
#define HUB75_PIN_A 8
#define HUB75_PIN_B 9
#define HUB75_PIN_C 10
#define HUB75_PIN_D 11
#define HUB75_PIN_E 12
#define HUB75_PIN_CLK 13
#define HUB75_PIN_LAT 14
#define HUB75_PIN_OE 15

// Current State
#define PREF_STATE_ON "state_on"
#define PREF_STATE_BRI "state_bri"
#define PREF_STATE_EFFECT "state_effect"
#define PREF_STATE_COLOR "state_color" // hex string or packed uint32_t

// --- Update Configuration ---
#define SOFTWARE_VERSION HYPERLED_VERSION
#define UPDATE_JSON_URL "https://raw.githubusercontent.com/dein-user/dein-repo/main/version.json"

// --- Hardware Settings ---

// --- Default Values ---
#define DEFAULT_LED_PIN 4
#define DEFAULT_LED_COUNT 30
#define DEFAULT_LED_TYPE 0 // WS281x

#define DEFAULT_AP_SSID "HyperLED-AP"


