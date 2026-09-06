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
#include "driver/gpio.h"
#include <Arduino.h>
#include "Config.h"
#include "BusWrapper.h"
#include <Preferences.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include "HyperBus.h"
#include "EspNowBus.h"
#include "StatusLedManager.h"
#include "EffectEngine.h"

// Hardware UART Pins for the Waveshare ESP32-S3-Zero. UPLINK cross-wires to
// the Master's own GPIO16/17 pair (see include/Config.h in the root project);
// DOWNLINK is free for daisy-chaining a further Slave.
#define UPLINK_RX 16
#define UPLINK_TX 17
#define DOWNLINK_RX 18
#define DOWNLINK_TX 38

// HyperBus Instances
HyperBusClass busUp(Serial1);
HyperBusClass busDown(Serial0);
EspNowBusClass espBus;

Preferences prefs;

// Configuration state
uint8_t myId = 254; // 254 = unconfigured slave
uint8_t ledPin = 4;
uint8_t ledPin2 = 255;
uint8_t ledType = 22;
uint16_t ledCount = 0;
uint16_t matrixWidth = 16;  // TYPE_HUB75 only
uint16_t matrixHeight = 16; // TYPE_HUB75 only
uint8_t hub75ShiftDriver = 0; // TYPE_HUB75 only - same Hub75ShiftDriver enum order as the Master
String slaveName = "New Slave";
const String slaveVersion = SOFTWARE_VERSION;

// LED Strip (dynamically allocated)
IBus* strip = nullptr;

// Pending OTA state
bool pendingOta = false;
String otaSsid = "";
String otaPass = "";
String otaUrl = "";

void initLEDs() {
    if (strip) {
        // Clear the pointer before freeing it: the ESP-NOW receive callback runs in the WiFi
        // task and guards its pixel writes with "if (strip)", so it must never see a pointer
        // to an object that is already being torn down here.
        IBus* old = strip;
        strip = nullptr;
        delete old;
    }
    if ((ledCount > 0 && ledPin != 255) || ledType == TYPE_HUB75) {
        switch (ledType) {
            case TYPE_HUB75:
                // Fixed 14-pin wiring (Config.h) instead of ledPin/ledPin2; matrixWidth x
                // matrixHeight instead of ledCount (kept in sync with ledCount by the
                // CMD_SET_CONFIG handler below, so CMD_SET_LEDS's bounds-check still works).
                strip = new BusHub75(matrixWidth, matrixHeight, (Hub75ShiftDriver)hub75ShiftDriver); break;
            case TYPE_WS2812_RGB:
                strip = new BusDigitalRgb<NeoGrbFeature, Neo800KbpsMethod>(ledCount, ledPin); break;
            case TYPE_SK6812_RGBW:
                strip = new BusDigitalRgbw<NeoGrbwFeature, Neo800KbpsMethod>(ledCount, ledPin); break;
            case TYPE_TM1814:
                strip = new BusDigitalRgbw<NeoWrgbTm1814Feature, Neo800KbpsMethod>(ledCount, ledPin); break;
            case TYPE_400KHZ:
                strip = new BusDigitalRgb<NeoGrbFeature, Neo400KbpsMethod>(ledCount, ledPin); break;
            case TYPE_APA102:
                strip = new BusDigitalSpiRgb<DotStarBgrFeature, DotStarSpiMethod>(ledCount, ledPin2, ledPin); break;
            case TYPE_LPD8806:
                strip = new BusDigitalSpiRgb<Lpd8806GrbFeature, Lpd8806SpiMethod>(ledCount, ledPin2, ledPin); break;
            case TYPE_TM1914:
                // TM1914 requires a chip-specific mode-select settings header before the pixel
                // data (handled by NeoGrbTm1914Feature) - a plain NeoGrbFeature frame omits it.
                strip = new BusDigitalRgb<NeoGrbTm1914Feature, Neo800KbpsMethod>(ledCount, ledPin); break;
            case TYPE_TM1829:
            case TYPE_UCS8903:
            case TYPE_APA106:
            case TYPE_WS2811_W:
            case TYPE_WS281X_WWA:
                strip = new BusDigitalRgb<NeoGrbFeature, Neo800KbpsMethod>(ledCount, ledPin); break;
            case TYPE_FW1906:
              case TYPE_UCS8904:
                  strip = new BusDigitalRgbw<NeoGrbwFeature, Neo800KbpsMethod>(ledCount, ledPin); break;
              case TYPE_WS2805:
              case TYPE_SM16825:
                  strip = new BusDigitalRgbww<NeoGrbwcFeature, Neo800KbpsMethod>(ledCount, ledPin); break;
            case TYPE_WS2801:
                // WS2801 is plain RGB-over-SPI with no start/end frame - NOT DotStar-protocol-compatible.
                strip = new BusDigitalSpiRgb<NeoRgbFeature, Ws2801SpiMethod>(ledCount, ledPin2, ledPin); break;
            case TYPE_LPD6803:
                // LPD6803 uses 16-bit 5-5-5 words with a 1-start-bit marker per pixel - NOT DotStar-protocol-compatible.
                strip = new BusDigitalSpiRgb<Lpd6803RgbFeature, Lpd6803SpiMethod>(ledCount, ledPin2, ledPin); break;
            case TYPE_PP9813:
                // P9813 has its own checksum-byte framing per pixel - NOT DotStar-protocol-compatible.
                strip = new BusDigitalSpiRgb<P9813BgrFeature, P9813SpiMethod>(ledCount, ledPin2, ledPin); break;
            case TYPE_ONOFF:
                strip = new BusOnOff(ledCount, ledPin); break;
            case TYPE_ANALOG_1CH:
                strip = new BusPwm(ledCount, 1, ledPin); break;
            case TYPE_ANALOG_2CH:
                strip = new BusPwm(ledCount, 2, ledPin, ledPin2); break;
            // 3, 4, 5 CH analog strips require more pins, but currently Slave UI only has pin and pin2.
            // For now, map what we have.
            case TYPE_ANALOG_3CH:
                strip = new BusPwm(ledCount, 3, ledPin, ledPin2, 255); break;
            case TYPE_ANALOG_4CH:
                strip = new BusPwm(ledCount, 4, ledPin, ledPin2, 255, 255); break;
            case TYPE_ANALOG_5CH:
                strip = new BusPwm(ledCount, 5, ledPin, ledPin2, 255, 255, 255); break;
            default:
                strip = new BusDigitalRgb<NeoGrbFeature, Neo800KbpsMethod>(ledCount, ledPin); break;
        }
        
        if (strip) {
            strip->Begin();
            strip->Show();
        }
    }
}

void loadConfig() {
    prefs.begin("hyperled_slave", false);
    myId = prefs.getUInt("id", 254);
    ledPin = prefs.getUInt("pin", 4);
    ledPin2 = prefs.getUInt("pin2", 255);
    ledCount = prefs.getUInt("count", 0);
    ledType = prefs.getUInt("type", 22);
    matrixWidth = prefs.getUInt("matW", 16);
    matrixHeight = prefs.getUInt("matH", 16);
    hub75ShiftDriver = prefs.getUInt("h75sd", 0);
    slaveName = prefs.getString("name", "New Slave");
    prefs.end();

    initLEDs();
}

void saveConfig() {
    prefs.begin("hyperled_slave", false);
    prefs.putUInt("id", myId);
    prefs.putUInt("pin", ledPin);
    prefs.putUInt("pin2", ledPin2);
    prefs.putUInt("count", ledCount);
    prefs.putUInt("type", ledType);
    prefs.putUInt("matW", matrixWidth);
    prefs.putUInt("matH", matrixHeight);
    prefs.putUInt("h75sd", hub75ShiftDriver);
    prefs.putString("name", slaveName);
    prefs.end();
}

unsigned long lastUartPacket = 0;
// Last valid packet from the Master on ANY transport (lastUartPacket only tracks the wired
// one). Drives the status LED's "disconnected" state, which has to work for ESP-NOW Slaves
// too. Stays 0 until the first contact, which counts as disconnected.
unsigned long lastMasterContact = 0;
unsigned long wifiCandidateTime = 0;
bool isUartSlave = false;
uint8_t transportMode = 0;

void performOtaUpdate() {
    Serial.println("Starting WLAN-On-Demand Update...");
    
    // Disable UART to prevent HyperBus interrupts from crashing the update
    Serial1.end();
    Serial0.end();
    
    // Clear the LED strip to black just in case
    if (strip) {
        for(int i=0; i<ledCount; i++) strip->SetPixelColor(i,0,0,0,0);
        strip->Show();
        delete strip; // FREE MEMORY for SSL Handshake!
        strip = nullptr;
    }
    
    if (transportMode != 1) {
        espBus.end();
        delay(100);
        WiFi.disconnect(true, true);
        delay(100);
    }
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(otaSsid.c_str(), otaPass.c_str());
    
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 40) {
        delay(500);
        Serial.print(".");
        retries++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected! Downloading firmware...");
        WiFiClientSecure client;
        client.setInsecure();
        
        // Manually follow GitHub redirect to S3 to avoid httpUpdate bug
        String finalUrl = otaUrl;
        HTTPClient http;
        http.begin(client, otaUrl);
        const char* headerKeys[] = {"Location"};
        http.collectHeaders(headerKeys, 1);
        int httpCode = http.GET();
        if (httpCode == HTTP_CODE_FOUND || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
            String newUrl = http.header("Location");
            if (newUrl.length() > 0) {
                finalUrl = newUrl;
                Serial.println("Redirected to: " + finalUrl);
            }
        }
        http.end();
        
        httpUpdate.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        t_httpUpdate_return ret = httpUpdate.update(client, finalUrl);
        
        switch (ret) {
            case HTTP_UPDATE_FAILED:
                Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
                break;
            case HTTP_UPDATE_NO_UPDATES:
                Serial.println("HTTP_UPDATE_NO_UPDATES");
                break;
            case HTTP_UPDATE_OK:
                Serial.println("HTTP_UPDATE_OK");
                break;
        }
    } else {
        Serial.println("\nWiFi connection failed!");
    }
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("OTA Failed. Rebooting in 2s...");
    delay(2000);
    ESP.restart();
}

// A wireless PONG cannot be sent from handleUplinkPacket(): for ESP-NOW that function *is*
// the receive callback and therefore runs in the WiFi task, where blocking (the collision
// delay) and calling esp_now_send() are both forbidden. The reply is handed to loop() instead.
// The UART path is unaffected - there the callback is already driven from loop().
static volatile bool pendingWirelessPong = false;
static unsigned long pendingWirelessPongAt = 0;

// Applying a new configuration is deferred for the same reason as the PONG above: for ESP-NOW
// handleUplinkPacket() runs in the WiFi task, and initLEDs() is far too heavy for it - it deletes
// a possibly running DMA driver, allocates new DMA memory and reconfigures peripherals, while
// saveConfig() writes NVS. Doing that inside the receive callback crashed the board when a Slave
// was switched to HUB75 over the air.
static volatile bool pendingConfigApply = false;

// Local rendering: the Master sends effect parameters, this Slave draws the frames itself.
// Streaming is still supported and takes precedence - receiving pixel data switches local
// rendering back off, so effects the Slave cannot render (image, clock/text) keep working.
static EffectState localEffect;
static bool localRenderActive = false;

// Feeds EffectEngine output into whatever bus this Slave drives.
class SlaveSink : public IEffectSink {
public:
    // Sync mode: the Master runs one effect across the whole chain and tells us which slice of it
    // we are. We then render the full-length effect and keep only our window, so a scanner or
    // chase crosses the segment boundary instead of restarting at our own pixel 0.
    // windowTotal == 0 means no sync - render for our own pixel count, as usual.
    uint16_t windowOffset = 0;
    uint16_t windowTotal = 0;

    uint16_t localCount() const {
        return (ledType == TYPE_HUB75) ? (uint16_t)(::matrixWidth * ::matrixHeight) : ledCount;
    }
    void setPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t w2) override {
        if (!strip) return;
        uint16_t local = index;
        if (windowTotal > 0) {
            if (index < windowOffset) return;          // belongs to a device before us
            local = index - windowOffset;
            if (local >= localCount()) return;         // belongs to a device after us
        }
        strip->SetPixelColor(local, r, g, b, w, w2);
    }
    uint16_t pixelCount() const override { return windowTotal > 0 ? windowTotal : localCount(); }
    // A panel is driven as a plain strip while synced: the chain the Master spans is linear, so
    // 2D effects fall back to their 1D relative for the duration.
    uint16_t matrixWidth() const override {
        return (ledType == TYPE_HUB75 && windowTotal == 0) ? ::matrixWidth : 0;
    }
    uint16_t matrixHeight() const override {
        return (ledType == TYPE_HUB75 && windowTotal == 0) ? ::matrixHeight : 0;
    }
};
static SlaveSink slaveSink;

// Set when the Master drives the animation itself (sync mode) and we must draw right away
// instead of waiting for our own frame timer.
static volatile bool forceRender = false;

static void sendPong(bool wireless) {
    // PONG Payload: [LED Count L] [LED Count H] [Version Length] [Version String...] [Name...]
    uint8_t verLen = slaveVersion.length();
    uint16_t len = 2 + 1 + verLen + slaveName.length();
    uint8_t* payload = (uint8_t*)malloc(len);
    if (!payload) return;
    payload[0] = ledCount & 0xFF;
    payload[1] = (ledCount >> 8) & 0xFF;
    payload[2] = verLen;
    memcpy(&payload[3], slaveVersion.c_str(), verLen);
    memcpy(&payload[3 + verLen], slaveName.c_str(), slaveName.length());

    if (wireless) espBus.sendPacket(HYPERBUS_MASTER_ID, myId, CMD_PONG, payload, len);
    else busUp.sendPacket(HYPERBUS_MASTER_ID, myId, CMD_PONG, payload, len);

    free(payload);
}

void handleUplinkPacket(const HyperBusPacket& packet) {
    if (packet.isValid) lastMasterContact = millis();

    if (!packet.isWireless) {
        lastUartPacket = millis();
        if (transportMode != 1) {
            prefs.begin("hyperled_slave", false);
            prefs.putUInt("transport", 1);
            prefs.end();
            Serial.println("UART Transport detected. Locking in and Rebooting...");
            if (strip) { strip->SetPixelColor(0, 0, 255, 0, 0); strip->Show(); }
            delay(500);
            ESP.restart();
        }
    } else {
        if (transportMode == 0) {
            if (wifiCandidateTime == 0) {
                wifiCandidateTime = millis();
                Serial.println("Wi-Fi signal received. Waiting 3s to ensure no UART signal arrives...");
            }
        } else if (transportMode != 2) {
            prefs.begin("hyperled_slave", false);
            prefs.putUInt("transport", 2);
            prefs.end();
            Serial.println("Wi-Fi Transport detected. Locking in and Rebooting...");
            delay(500);
            ESP.restart();
        }
    }
    
    bool hasActiveUART = (transportMode == 1);

    bool isForMe = (packet.targetId == myId || packet.targetId == HYPERBUS_BROADCAST_ID || (myId == 254 && packet.targetId == 254));
    
    // Process if it's for me
    if (isForMe && packet.isValid) {
        
        if (packet.command == CMD_PING) {
            // If we have an active UART connection, ignore ESP-NOW pings to avoid transport flapping!
            if (hasActiveUART && packet.isWireless) return;

            if (packet.isWireless) {
                // Defer to loop() - see the note at sendPong(). The random offset still spreads
                // replies out so several slaves don't answer the same broadcast at once.
                if (!pendingWirelessPong) {
                    pendingWirelessPongAt = millis() + random(10, 50);
                    pendingWirelessPong = true;
                }
                return;
            }

            // Reply with PONG. Random delay if unconfigured to avoid collisions on multidrop
            delay(random(10, 50));
            sendPong(false);
        }
        else if (packet.command == CMD_SET_CONFIG) {
            // Payload: [New ID] [LED Pin] [LED Pin 2] [LED Type] [LED Count L] [LED Count H]
            //          [Matrix Width L] [Matrix Width H] [Matrix Height L] [Matrix Height H]
            //          [HUB75 Shift Driver] [Name...]
            if (packet.length >= 11) {
                myId = packet.payload[0];
                ledPin = packet.payload[1];
                ledPin2 = packet.payload[2];
                ledType = packet.payload[3];
                ledCount = packet.payload[4] | (packet.payload[5] << 8);
                matrixWidth = packet.payload[6] | (packet.payload[7] << 8);
                matrixHeight = packet.payload[8] | (packet.payload[9] << 8);
                hub75ShiftDriver = packet.payload[10];

                if (packet.length > 11) {
                    char nameBuf[64] = {0};
                    int nameLen = min((int)(packet.length - 11), 63);
                    memcpy(nameBuf, &packet.payload[11], nameLen);
                    slaveName = String(nameBuf);
                }
                pendingConfigApply = true;
            }
        }
        else if (packet.command == CMD_SET_SEGMENT) {
            if (packet.length >= HYPERBUS_SEGMENT_PAYLOAD_LEN) {
                const uint8_t* p = packet.payload;
                localEffect.effect     = p[0];
                localEffect.brightness = p[1];
                localEffect.speed      = p[2];
                localEffect.intensity  = p[3];
                localEffect.palette    = p[4];
                localEffect.isOn          = (p[5] & 0x01) != 0;
                localEffect.color2Enabled = (p[5] & 0x02) != 0;
                localEffect.whiteOnly     = (p[5] & 0x04) != 0;
                localEffect.color  = ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 8) | p[8];
                localEffect.color2 = ((uint32_t)p[9] << 16) | ((uint32_t)p[10] << 8) | p[11];
                localEffect.cct    = p[12];
                // The Master's step counter keeps a multi-panel effect roughly in phase. It is
                // only adopted on a real change, otherwise the periodic refresh would keep
                // resetting the animation the Slave is advancing on its own.
                uint16_t masterStep = p[13] | ((uint16_t)p[14] << 8);
                slaveSink.windowOffset = p[15] | ((uint16_t)p[16] << 8);
                slaveSink.windowTotal  = p[17] | ((uint16_t)p[18] << 8);

                if (slaveSink.windowTotal > 0) {
                    // Synced: the Master owns the clock. Take its step and draw immediately -
                    // advancing on our own would let the pattern drift apart at the boundary.
                    localEffect.effectStep = masterStep;
                    forceRender = true;
                } else if (!localRenderActive) {
                    localEffect.effectStep = masterStep;
                }

                // Report only on a change, never per packet: these arrive on a refresh timer and
                // this board's USB-CDC can stall the firmware if it is printed to to constantly.
                static uint8_t reportedEffect = 255;
                static bool reportedActive = false;
                static uint16_t reportedWindow = 0xFFFF;
                if (!reportedActive || reportedEffect != localEffect.effect
                    || reportedWindow != slaveSink.windowTotal) {
                    reportedActive = true;
                    reportedEffect = localEffect.effect;
                    reportedWindow = slaveSink.windowTotal;
                    Serial.printf("Rendering effect %u locally (%ux%u, window %u+%u)" "\n",
                                  localEffect.effect, slaveSink.matrixWidth(), slaveSink.matrixHeight(),
                                  slaveSink.windowOffset, slaveSink.windowTotal);
                }
                localRenderActive = true;
            }
        }
        else if (packet.command == CMD_SET_STATUS_LED) {
            // Payload: [on] [r] [g] [b] [brightness]
            if (packet.length >= 5) {
                uint32_t color = ((uint32_t)packet.payload[1] << 16) |
                                 ((uint32_t)packet.payload[2] << 8) |
                                 (uint32_t)packet.payload[3];
                StatusLedManager.setState(packet.payload[0] != 0, color, packet.payload[4]);
            }
        }
        else if (packet.command == CMD_SET_LEDS) {
              // Payload: RGBW array (5 bytes per pixel)
              localRenderActive = false; // the Master is driving the pixels again
              if (strip && packet.length >= 5) {
                  uint16_t maxLeds = min((int)ledCount, (int)(packet.length / 5));
                  for (uint16_t i = 0; i < maxLeds; i++) {
                      uint8_t r = packet.payload[i * 5];
                      uint8_t g = packet.payload[i * 5 + 1];
                      uint8_t b = packet.payload[i * 5 + 2];
                      uint8_t w = packet.payload[i * 5 + 3];
                      uint8_t w2 = packet.payload[i * 5 + 4];
                      strip->SetPixelColor(i, r, g, b, w, w2);
                  }
                  strip->Show();
              }
          }
        else if (packet.command == CMD_SET_LEDS_CHUNK) {
              // Payload: [OffsetL] [OffsetH] [RGBW array]
              localRenderActive = false; // the Master is driving the pixels again
              if (strip && packet.length >= 7) {
                  uint16_t offset = packet.payload[0] | (packet.payload[1] << 8);
                  // Reject an out-of-range offset before subtracting. (ledCount - offset) is
                  // computed as int, so an offset past the end goes negative and then wraps to
                  // ~65000 when stored in a uint16_t, and the loop reads far past the packet.
                  // The Master and the Slave disagree about ledCount for a moment whenever a
                  // Slave is reconfigured, which made this crash the board with LoadProhibited.
                  if (offset >= ledCount) return;
                  uint16_t roomLeft = ledCount - offset;
                  uint16_t inPacket = (packet.length - 2) / 5;
                  uint16_t maxLeds = roomLeft < inPacket ? roomLeft : inPacket;
                  for (uint16_t i = 0; i < maxLeds; i++) {
                      uint8_t r = packet.payload[2 + i * 5];
                      uint8_t g = packet.payload[2 + i * 5 + 1];
                      uint8_t b = packet.payload[2 + i * 5 + 2];
                      uint8_t w = packet.payload[2 + i * 5 + 3];
                      uint8_t w2 = packet.payload[2 + i * 5 + 4];
                      strip->SetPixelColor(offset + i, r, g, b, w, w2);
                  }
                  strip->Show();
              }
          }
        else if (packet.command == CMD_TRIGGER_UPDATE) {
            // Payload: JSON string with {"ssid":"...","pass":"...","url":"..."}
            char jsonBuf[512] = {0};
            int copyLen = min((int)packet.length, 511);
            memcpy(jsonBuf, packet.payload, copyLen);
            
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, jsonBuf);
            if (!err) {
                otaSsid = doc["ssid"].as<String>();
                otaPass = doc["pass"].as<String>();
                otaUrl = doc["url"].as<String>();
                
                if (otaSsid.length() > 0 && otaUrl.length() > 0) {
                    pendingOta = true;
                }
            }
        }
    }
    
    // Forward to Downlink if not exclusively for me
    // Broadcasts (255) and unconfigured (254) should be forwarded so other slaves can hear them
    if (packet.targetId != myId || packet.targetId == HYPERBUS_BROADCAST_ID || packet.targetId == 254) {
        if (!packet.isWireless) busDown.sendPacket(packet.targetId, packet.senderId, packet.command, packet.payload, packet.length);
    }
}

void handleDownlinkPacket(const HyperBusPacket& packet) {
    // Anything received from Downlink is forwarded to Uplink (Master)
    // We do NOT process packets from Downlink because the Master sends commands from Uplink.
    // The only thing on Downlink is responses from downstream slaves.
    busUp.sendPacket(packet.targetId, packet.senderId, packet.command, packet.payload, packet.length);
}

void setup() {
    // We don't use Serial (USB) so we don't break pins if they overlap, but UART0 and UART1 are safe.
    Serial.begin(115200); // Debug output over USB CDC
    Serial.println("HyperLED Slave Booting...");
    
    loadConfig();
    
    // Start UART Buses
    busUp.begin(115200, UPLINK_RX, UPLINK_TX);
    gpio_pullup_en((gpio_num_t)UPLINK_RX);
    
    busDown.begin(115200, DOWNLINK_RX, DOWNLINK_TX);
    gpio_pullup_en((gpio_num_t)DOWNLINK_RX);
    
    busUp.setCallback(handleUplinkPacket);
    busDown.setCallback(handleDownlinkPacket);
    
    prefs.begin("hyperled_slave", false);
    transportMode = prefs.getUInt("transport", 0);
    if (prefs.getUInt("nvs_reset_v2", 0) == 0) {
        prefs.putUInt("transport", 0);
        transportMode = 0;
        prefs.putUInt("nvs_reset_v2", 1);
        Serial.println("Forced NVS Reset to Auto-Sense mode!");
    }
    prefs.end();
    
    if (transportMode == 1) {
        Serial.println("Locked to UART Transport");
        isUartSlave = true;
    } else if (transportMode == 2) {
        Serial.println("Locked to Wi-Fi Transport");
        espBus.begin(WIFI_STA, true);
        espBus.setCallback(handleUplinkPacket);
    } else {
        Serial.println("Auto-Sensing Transport...");
        espBus.begin(WIFI_STA, true);
        espBus.setCallback(handleUplinkPacket);
    }
    
    StatusLedManager.begin();

    Serial.println("Slave Ready.");
}

void loop() {
    if (pendingOta) {
        pendingOta = false;
        performOtaUpdate();
    }
    
    busUp.loop();
    busDown.loop();
    if (transportMode != 1) espBus.loop();

    if (pendingWirelessPong && (long)(millis() - pendingWirelessPongAt) >= 0) {
        pendingWirelessPong = false;
        sendPong(true);
    }

    if (pendingConfigApply) {
        pendingConfigApply = false;
        saveConfig();
        initLEDs();
    }

    // Draw locally when the Master has handed us effect parameters. EffectEngine::render() does
    // its own speed timing and reports whether anything actually changed, so an idle effect
    // costs nothing and the panel is only pushed when there is a new frame.
    if (localRenderActive && strip) {
        if (forceRender) {
            forceRender = false;
            EffectEngine::draw(localEffect, slaveSink);
            strip->Show();
        } else if (EffectEngine::render(localEffect, slaveSink, millis())) {
            strip->Show();
        }
    }

    if (transportMode == 0 && wifiCandidateTime > 0 && millis() - wifiCandidateTime > 3000) {
        prefs.begin("hyperled_slave", false);
        prefs.putUInt("transport", 2);
        prefs.end();
        Serial.println("UART timeout. Locking Wi-Fi Transport and Rebooting...");
        delay(500);
        ESP.restart();
    }
    
    // Auto-Revert if locked transport is dead for 30 seconds after boot
    static unsigned long bootTime = millis();
    if (millis() - bootTime > 30000) {
        if (transportMode == 1 && lastUartPacket == 0) {
            prefs.begin("hyperled_slave", false);
            prefs.putUInt("transport", 0);
            prefs.end();
            Serial.println("UART timeout. Reverting to Auto-Sense...");
            ESP.restart();
        }
    }
    
    // Hardware Lockup Recovery. The threshold is 5s rather than 2s: a Slave that renders its own
    // effects only receives the 4 pings/s, where it used to see a constant stream of pixel data,
    // so a brief pause on the Master - reinitialising its LED bus when segments change, for
    // instance - is enough to look like a dead UART. 5s still catches a genuinely stuck
    // peripheral quickly, and matches what the wireless transport treats as losing the Master.
    if (transportMode == 1 && lastUartPacket > 0 && millis() - lastUartPacket > 5000) {
        Serial.println("UART Hardware Lockup detected! Restarting peripheral...");
        busUp.begin(115200, UPLINK_RX, UPLINK_TX);
        gpio_pullup_en((gpio_num_t)UPLINK_RX); // MUST set pullup AFTER begin!
        lastUartPacket = millis(); // Reset timer to give it time to recover
    }

    // Feed the onboard status LED. The Master pings every 250ms on both transports, so a
    // few seconds of silence means the link is really gone rather than just a dropped
    // packet. Right after boot lastMasterContact is still 0 - that counts as disconnected
    // too, which is what you want to see on a Slave nobody is talking to.
    SlaveLedCondition cond;
    if (lastMasterContact == 0 || millis() - lastMasterContact > 5000) {
        cond = SLED_DISCONNECTED;
    } else if (myId == 254) {
        cond = SLED_UNCONFIGURED;
    } else {
        cond = SLED_OK;
    }
    StatusLedManager.setCondition(cond);
    StatusLedManager.loop();
}























