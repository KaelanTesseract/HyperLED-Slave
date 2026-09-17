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
#include "WidgetRender.h"
#include <esp_rom_crc.h>
#include <esp_heap_caps.h>

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
// What a HUB75 panel's driver was last set to (see updatePanelBrightness()); 255 is what a freshly
// created driver starts with.
static uint8_t panelBrightnessApplied = 255;

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
                strip = new BusHub75(matrixWidth, matrixHeight, (Hub75ShiftDriver)hub75ShiftDriver);
                panelBrightnessApplied = 255;
                break;
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
// The brightness the Master asked for. On a HUB75 panel the effect is rendered at full scale and
// this goes to the panel driver instead (see updatePanelBrightness()).
static uint8_t localEffectBrightness = 255;

// Local rendering of the "Uhr / Text" elements the Master handed to this Slave (see
// CMD_SET_WIDGETS in HyperBus.h) - since 0.2.004 every type: clock, date, text, image, analog
// clock, weather and Lauftext, drawn with the same code the Master uses (WidgetRender.h).
// Mutually exclusive with localRenderActive, but NOT with streamed pixels: when the Master still
// streams a frame for this panel (an element that did not fit into the config), both run at once
// and each keeps to its own pixels - the elements' rectangles belong to this Slave, everything
// else to the stream.
struct LocalWidget {
    uint8_t id = 0;
    uint8_t type = 0;   // WidgetRender::TYPE_*
    int16_t x = 0;
    int16_t y = 0;
    uint32_t color = 0xFFFFFF;
    uint8_t scale = 1;
    uint8_t format = 0;
    uint8_t font = 0;   // 0 = normal 5x7, 1 = mini 3x5
    uint8_t speed = 128; // Lauftext scroll speed
    uint8_t width = 0;  // image width, analog diameter or Lauftext window
    uint8_t height = 0; // image height
    uint32_t crc = 0;   // image pixels this element should show (0 = none)
    uint8_t bri = 255;  // the element's own brightness, on top of the segment's
    uint8_t legible = WidgetRender::LEGIBLE_OUTLINE; // how it stays readable over the background
    String text;
};
static const uint8_t LOCAL_WIDGET_MAX = 12; // the Master's TEXT_WIDGET_MAX
struct WidgetRect {
    int16_t x;
    int16_t y;
    int16_t w;
    int16_t h;
};

// Owned by loop(). A CMD_SET_WIDGETS over ESP-NOW arrives in the WiFi task, so the receive path
// never touches these directly - it only stores the raw payload (see pendingWidgetPayload), and
// loop() applies it. Replacing the vector from the WiFi task while loop() iterated over it would
// free memory under that iteration.
static std::vector<LocalWidget> localWidgets;
static uint8_t localWidgetsBrightness = 255;
static bool localWidgetsOn = true;
// HYPERBUS_WIDGET_FLAG_MASTER_LAYER as last applied: whether the Master streams a frame for the rest
// of the panel, i.e. whether pixels outside the widgets are ours to clear.
static bool localWidgetsMasterLayer = false;
static volatile bool localWidgetsActive = false;
// Set when a changed widget config was applied, so the next loop() draws every widget once - a
// lone static text widget never animates and would otherwise never get drawn.
static bool localWidgetsDirty = false;

// Rectangles the widgets occupy. Streamed pixels that fall inside them are dropped: the Master's
// frame has nothing but black there, and writing it would erase the widgets. This panel is
// single-buffered, so such a write is on screen at once. Written by loop(), read by the receive
// path (the WiFi task, for ESP-NOW) - hence the lock.
static WidgetRect widgetRects[LOCAL_WIDGET_MAX];
static uint8_t widgetRectCount = 0;
static portMUX_TYPE widgetMux = portMUX_INITIALIZER_UNLOCKED;

// Latest CMD_SET_WIDGETS payload, handed from the receive path to loop() under widgetMux.
static uint8_t pendingWidgetPayload[HYPERBUS_WIDGETS_MAX_PAYLOAD];
static uint16_t pendingWidgetLen = 0;
static bool pendingWidgetConfig = false;
static volatile unsigned long lastWidgetPacketAt = 0;
// The Master refreshes the config every 2s. If it stops (the segment left "Uhr / Text" and the
// release packet got lost), stop drawing rather than overlay widgets on whatever comes next forever.
static const unsigned long LOCAL_WIDGETS_TIMEOUT_MS = 10000;

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

// --- What the elements need from the Master ------------------------------------------------
// Both are written by the receive path (the WiFi task, for ESP-NOW) and read by loop().
static portMUX_TYPE infoMux = portMUX_INITIALIZER_UNLOCKED;
static bool clockKnown = false;
static uint32_t clockEpoch = 0;        // Master's local time (see CMD_SET_TIME) ...
static unsigned long clockBaseMs = 0;  // ... as of this millis()
static WidgetRender::Weather slaveWeather = {false, 0, 1};

static WidgetRender::Clock currentClock() {
    WidgetRender::Clock c;
    portENTER_CRITICAL(&infoMux);
    bool known = clockKnown;
    uint32_t epoch = clockEpoch;
    unsigned long base = clockBaseMs;
    portEXIT_CRITICAL(&infoMux);
    c.known = known;
    memset(&c.tm, 0, sizeof(c.tm));
    if (known) WidgetRender::fromLocalEpoch(epoch + (uint32_t)((millis() - base) / 1000), c.tm);
    return c;
}

static WidgetRender::Weather currentWeather() {
    portENTER_CRITICAL(&infoMux);
    WidgetRender::Weather w = slaveWeather;
    portEXIT_CRITICAL(&infoMux);
    return w;
}

// --- Image pixels ------------------------------------------------------------------------------
// Fetched from the Master with CMD_REQUEST_WIDGET_IMAGE and kept in RAM. A restart, a lost piece or
// an edited image all end the same way: the checksum in the config does not match what is here,
// so the Slave asks (again) until it does.
struct ImagePiece {
    uint8_t id;
    uint8_t width;
    uint8_t height;
    uint32_t crc;
    uint16_t offset;
    uint16_t total;
    uint8_t len;
    uint8_t data[HYPERBUS_IMAGE_CHUNK_DATA];
};
// Pieces arrive in the WiFi task and are handed to loop() through this queue.
static QueueHandle_t imagePieceQueue = nullptr;
static const uint8_t IMAGE_PIECE_QUEUE_LEN = 24;

struct SlaveImage {
    uint8_t id = 0;
    uint32_t crc = 0;
    uint8_t width = 0;
    uint8_t height = 0;
    uint16_t total = 0;
    uint8_t* data = nullptr;
    std::vector<uint8_t> have;  // one entry per piece
    uint16_t haveCount = 0;
    bool ready = false;
    unsigned long lastRequest = 0;
};
static std::vector<SlaveImage> slaveImages; // owned by loop()
static const unsigned long IMAGE_REQUEST_RETRY_MS = 3000;
static const uint16_t IMAGE_MAX_BYTES = 64 * 64 * 3; // the Master's TEXT_WIDGET_IMG_MAX
// Which transport the widget config came over - the requests go back the same way.
static volatile bool widgetsOverEspNow = true;

static void releaseImage(SlaveImage& img) {
    if (img.data) free(img.data);
    img.data = nullptr;
    img.have.clear();
    img.haveCount = 0;
    img.total = 0;
    img.ready = false;
}

static const SlaveImage* readyImage(uint8_t id, uint32_t crc) {
    for (const auto& img : slaveImages) {
        if (img.id == id && img.crc == crc && img.ready) return &img;
    }
    return nullptr;
}

static WidgetRender::Spec specOf(const LocalWidget& tw) {
    WidgetRender::Spec spec;
    spec.type = tw.type;
    spec.x = tw.x;
    spec.y = tw.y;
    spec.color = tw.color;
    spec.scale = tw.scale;
    spec.format = tw.format;
    spec.font = tw.font;
    spec.speed = tw.speed;
    spec.width = tw.width;
    spec.height = tw.height;
    spec.text = tw.text.c_str();
    spec.textLen = (uint16_t)tw.text.length();
    spec.img = nullptr;
    spec.imgLen = 0;
    spec.bri = tw.bri;
    if (tw.type == WidgetRender::TYPE_IMAGE && tw.crc != 0) {
        const SlaveImage* img = readyImage(tw.id, tw.crc);
        if (img) {
            spec.img = img->data;
            spec.imgLen = img->total;
        }
    }
    return spec;
}

// The area an element draws into - the same geometry the drawing itself uses.
static WidgetRect widgetRect(const LocalWidget& tw) {
    WidgetRender::Rect r = WidgetRender::bounds(specOf(tw));
    WidgetRect out = {r.x, r.y, r.w, r.h};
    return out;
}

// Pixel index (row-major, as streamed) inside any of the rectangles? Only a HUB75 panel has widgets.
static bool pixelInWidgetRects(uint16_t index, const WidgetRect* rects, uint8_t count) {
    if (ledType != TYPE_HUB75 || matrixWidth == 0) return false;
    int32_t x = index % matrixWidth;
    int32_t y = index / matrixWidth;
    for (uint8_t i = 0; i < count; i++) {
        const WidgetRect& r = rects[i];
        if (x >= r.x && x < (int32_t)r.x + r.w && y >= r.y && y < (int32_t)r.y + r.h) return true;
    }
    return false;
}

// Snapshot of the widget rectangles for the receive path, which runs outside loop() over ESP-NOW.
static uint8_t copyWidgetRects(WidgetRect* out) {
    portENTER_CRITICAL(&widgetMux);
    uint8_t count = widgetRectCount;
    if (count > 0) memcpy(out, widgetRects, sizeof(WidgetRect) * count);
    portEXIT_CRITICAL(&widgetMux);
    return count;
}

static void clearWidgetRect(const WidgetRect& r) {
    if (!strip || matrixWidth == 0 || matrixHeight == 0) return;
    for (int32_t y = r.y; y < (int32_t)r.y + r.h; y++) {
        if (y < 0 || y >= (int32_t)matrixHeight) continue;
        for (int32_t x = r.x; x < (int32_t)r.x + r.w; x++) {
            if (x < 0 || x >= (int32_t)matrixWidth) continue;
            strip->SetPixelColor((uint16_t)(y * matrixWidth + x), 0, 0, 0, 0, 0);
        }
    }
}

// The elements are composed into a full RGB frame first and only pixels that differ from what
// is on the panel are written. The panel is single-buffered - every write is visible at once - so
// "clear, then draw" would flash black on every tick, and rewriting 4096 pixels twenty times a
// second for a scrolling text would be wasted work.
static uint8_t* frameRgb = nullptr;   // what the elements want
static uint8_t* shownRgb = nullptr;   // what the panel shows, as far as we wrote it
static uint16_t frameW = 0;
static uint16_t frameH = 0;
// False when something else may have written the panel since: the next flush rewrites every pixel
// we own instead of only the changed ones.
static volatile bool shownValid = false;

// The effect behind the elements (CMD_SET_BACKGROUND). Drawn only while the whole panel is ours.
// bgRgb keeps the effect's last frame - effects may leave pixels they did not change - and each
// composed frame starts from a copy of it. Owned by loop(); the receive path only hands over the
// payload (pendingBackground, under widgetMux).
static EffectState bgState;
static bool bgActive = false;
static uint8_t bgBrightness = 255;     // relative to the elements
static uint8_t* bgRgb = nullptr;
static uint8_t* composeMask = nullptr;
static uint16_t bgW = 0;
static uint16_t bgH = 0;
static unsigned long lastBgFrame = 0;
static uint8_t pendingBackground[HYPERBUS_BACKGROUND_PAYLOAD_LEN];
static bool pendingBackgroundConfig = false;
// Faster than this the composition gains nothing visible, and the panel has to be rewritten each time.
static const unsigned long BG_MIN_FRAME_MS = 20;

static uint8_t* allocPixels(size_t bytes) {
    uint8_t* p = (uint8_t*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) p = (uint8_t*)malloc(bytes);
    return p;
}

static bool ensureFrame() {
    if (matrixWidth == 0 || matrixHeight == 0) return false;
    if (frameRgb && shownRgb && frameW == matrixWidth && frameH == matrixHeight) return true;
    if (frameRgb) free(frameRgb);
    if (shownRgb) free(shownRgb);
    size_t bytes = (size_t)matrixWidth * matrixHeight * 3;
    frameRgb = allocPixels(bytes);
    shownRgb = allocPixels(bytes);
    if (!frameRgb || !shownRgb) {
        if (frameRgb) free(frameRgb);
        if (shownRgb) free(shownRgb);
        frameRgb = shownRgb = nullptr;
        frameW = frameH = 0;
        return false;
    }
    frameW = matrixWidth;
    frameH = matrixHeight;
    shownValid = false;
    return true;
}

static bool ensureBackgroundBuffers() {
    if (!ensureFrame()) return false;
    if (bgRgb && composeMask && bgW == frameW && bgH == frameH) return true;
    if (bgRgb) free(bgRgb);
    if (composeMask) free(composeMask);
    size_t px = (size_t)frameW * frameH;
    bgRgb = allocPixels(px * 3);
    composeMask = allocPixels(px);
    if (!bgRgb || !composeMask) {
        if (bgRgb) free(bgRgb);
        if (composeMask) free(composeMask);
        bgRgb = composeMask = nullptr;
        bgW = bgH = 0;
        return false;
    }
    memset(bgRgb, 0, px * 3);
    bgW = frameW;
    bgH = frameH;
    bgState = EffectState(); // its per-pixel memory belongs to the old size
    lastBgFrame = 0;
    return true;
}

// Whether a background is to be drawn right now.
static bool backgroundShown() {
    return bgActive && localWidgetsActive && localWidgetsOn && !localWidgetsMasterLayer &&
           ledType == TYPE_HUB75;
}

// Applies a CMD_SET_BACKGROUND payload. Runs in loop() only.
static void applyBackgroundConfig(const uint8_t* p) {
    uint8_t effect = p[0];
    bool active = effect != HYPERBUS_BACKGROUND_NONE && EffectEngine::canRender(effect);
    uint32_t color = ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 8) | p[8];
    uint32_t color2 = ((uint32_t)p[9] << 16) | ((uint32_t)p[10] << 8) | p[11];
    bool color2Enabled = (p[5] & 0x01) != 0;
    bool changed = active != bgActive;
    if (active) {
        changed = changed || bgState.effect != effect || bgBrightness != p[1] ||
                  bgState.speed != p[2] || bgState.intensity != p[3] || bgState.palette != p[4] ||
                  bgState.color != color || bgState.color2 != color2 ||
                  bgState.color2Enabled != color2Enabled;
    }
    if (!changed) return; // the periodic refresh

    if (active && bgState.effect != effect) {
        bgState = EffectState(); // a different effect starts from a clean state
        if (bgRgb) memset(bgRgb, 0, (size_t)bgW * bgH * 3);
        lastBgFrame = 0;
    }
    if (active) {
        bgState.effect = effect;
        bgBrightness = p[1];
        bgState.speed = p[2];
        bgState.intensity = p[3];
        bgState.palette = p[4];
        bgState.isOn = true;
        bgState.color = color;
        bgState.color2 = color2;
        bgState.color2Enabled = color2Enabled;
        bgState.whiteOnly = false;
        bgState.cct = 128;
    }
    bgActive = active;
    localWidgetsDirty = true;
    Serial.printf("Background: %s\n", active ? String(effect).c_str() : "off");
}

// Advances the background effect when it is due. Returns true when it drew a new frame.
static bool advanceBackground(unsigned long now) {
    if (!backgroundShown() || !ensureBackgroundBuffers()) return false;
    if (lastBgFrame != 0 && now - lastBgFrame < BG_MIN_FRAME_MS) return false;
    // Relative to the segment, like an element's own brightness: the whole panel is dimmed by the
    // driver on top (see updatePanelBrightness()).
    bgState.brightness = bgBrightness;
    RgbFrameSink sink;
    sink.rgb = bgRgb;
    sink.w = bgW;
    sink.h = bgH;
    if (!EffectEngine::render(bgState, sink, now)) return false;
    lastBgFrame = now;
    return true;
}

static bool inRects(int32_t x, int32_t y, const WidgetRect* rects, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        const WidgetRect& r = rects[i];
        if (x >= r.x && x < (int32_t)r.x + r.w && y >= r.y && y < (int32_t)r.y + r.h) return true;
    }
    return false;
}

// Writes one owned pixel if it differs from what the panel shows (or everything, after an
// invalidation). Returns true if it wrote.
static bool flushPixel(uint16_t x, uint16_t y, bool all) {
    size_t off = ((size_t)y * frameW + x) * 3;
    if (!all && frameRgb[off] == shownRgb[off] && frameRgb[off + 1] == shownRgb[off + 1] &&
        frameRgb[off + 2] == shownRgb[off + 2]) {
        return false;
    }
    shownRgb[off] = frameRgb[off];
    shownRgb[off + 1] = frameRgb[off + 1];
    shownRgb[off + 2] = frameRgb[off + 2];
    strip->SetPixelColor((uint16_t)(y * frameW + x), frameRgb[off], frameRgb[off + 1],
                         frameRgb[off + 2], 0, 0);
    return true;
}

// Composes and shows every element. With a Master layer only the elements' own rectangles are
// ours; otherwise the whole panel is.
static void renderLocalWidgets(unsigned long now) {
    if (!strip || ledType != TYPE_HUB75 || !ensureFrame()) return;
    bool whole = !localWidgetsMasterLayer;
    WidgetRect rects[LOCAL_WIDGET_MAX];
    uint8_t rectCount = 0;
    portENTER_CRITICAL(&widgetMux);
    rectCount = widgetRectCount;
    if (rectCount > 0) memcpy(rects, widgetRects, sizeof(WidgetRect) * rectCount);
    portEXIT_CRITICAL(&widgetMux);

    // Over a background effect: start from its frame and let each element keep itself readable.
    if (whole && backgroundShown() && ensureBackgroundBuffers()) {
        memcpy(frameRgb, bgRgb, (size_t)frameW * frameH * 3);
        WidgetRender::Clock clk = currentClock();
        WidgetRender::Weather wx = currentWeather();
        const std::vector<LocalWidget>& list = localWidgets;
        WidgetRender::composeOver(frameRgb, composeMask, frameW, frameH, list.size(),
                                  [&](size_t i) { return specOf(list[i]); },
                                  [&](size_t i) { return list[i].legible; },
                                  255, clk, wx, now);
        bool all = !shownValid;
        shownValid = true;
        uint32_t written = 0;
        for (uint16_t y = 0; y < frameH; y++) {
            for (uint16_t x = 0; x < frameW; x++) {
                if (flushPixel(x, y, all)) written++;
            }
        }
        if (written > 0) strip->Show();
        return;
    }

    // 1. clear what we own
    if (whole) {
        memset(frameRgb, 0, (size_t)frameW * frameH * 3);
    } else {
        for (uint8_t i = 0; i < rectCount; i++) {
            const WidgetRect& r = rects[i];
            for (int32_t y = r.y; y < (int32_t)r.y + r.h; y++) {
                if (y < 0 || y >= (int32_t)frameH) continue;
                for (int32_t x = r.x; x < (int32_t)r.x + r.w; x++) {
                    if (x < 0 || x >= (int32_t)frameW) continue;
                    size_t off = ((size_t)y * frameW + x) * 3;
                    frameRgb[off] = frameRgb[off + 1] = frameRgb[off + 2] = 0;
                }
            }
        }
    }

    // 2. draw the elements
    if (localWidgetsOn) {
        WidgetRender::Clock clk = currentClock();
        WidgetRender::Weather wx = currentWeather();
        auto plot = [&](int16_t x, int16_t y, uint8_t r, uint8_t g, uint8_t b) {
            if (!whole && !inRects(x, y, rects, rectCount)) return; // the stream's pixel
            size_t off = ((size_t)y * frameW + x) * 3;
            frameRgb[off] = r;
            frameRgb[off + 1] = g;
            frameRgb[off + 2] = b;
        };
        for (const auto& tw : localWidgets) {
            // Full scale when the whole panel is ours: the driver dims it (see
            // updatePanelBrightness()). Next to a stream the stream sets the driver to full, so the
            // elements dim their own pixels like the stream does.
            WidgetRender::draw(specOf(tw), whole ? 255 : localWidgetsBrightness, frameW, frameH, clk, wx, now, plot);
        }
    }

    // 3. write what changed
    bool all = !shownValid;
    shownValid = true;
    uint32_t written = 0;
    if (whole) {
        for (uint16_t y = 0; y < frameH; y++) {
            for (uint16_t x = 0; x < frameW; x++) {
                if (flushPixel(x, y, all)) written++;
            }
        }
    } else {
        for (uint8_t i = 0; i < rectCount; i++) {
            const WidgetRect& r = rects[i];
            for (int32_t y = r.y; y < (int32_t)r.y + r.h; y++) {
                if (y < 0 || y >= (int32_t)frameH) continue;
                for (int32_t x = r.x; x < (int32_t)r.x + r.w; x++) {
                    if (x < 0 || x >= (int32_t)frameW) continue;
                    if (flushPixel((uint16_t)x, (uint16_t)y, all)) written++;
                }
            }
        }
    }
    if (written > 0) strip->Show();
}

// --- Image bookkeeping (loop() only) -------------------------------------------------------------

static void sendImageRequest(uint8_t id, uint32_t crc) {
    uint8_t payload[HYPERBUS_IMAGE_REQUEST_LEN] = {
        id, (uint8_t)(crc & 0xFF), (uint8_t)((crc >> 8) & 0xFF),
        (uint8_t)((crc >> 16) & 0xFF), (uint8_t)((crc >> 24) & 0xFF)};
    if (widgetsOverEspNow) espBus.sendPacket(HYPERBUS_MASTER_ID, myId, CMD_REQUEST_WIDGET_IMAGE, payload, sizeof(payload));
    else busUp.sendPacket(HYPERBUS_MASTER_ID, myId, CMD_REQUEST_WIDGET_IMAGE, payload, sizeof(payload));
}

// Brings the image store in line with the elements just configured.
static void syncImagesWithWidgets() {
    for (auto it = slaveImages.begin(); it != slaveImages.end(); ) {
        bool wanted = false;
        for (const auto& w : localWidgets) {
            if (w.type == WidgetRender::TYPE_IMAGE && w.id == it->id && w.crc != 0) { wanted = true; break; }
        }
        if (!wanted) {
            releaseImage(*it);
            it = slaveImages.erase(it);
        } else {
            ++it;
        }
    }
    for (const auto& w : localWidgets) {
        if (w.type != WidgetRender::TYPE_IMAGE || w.crc == 0) continue;
        SlaveImage* found = nullptr;
        for (auto& img : slaveImages) {
            if (img.id == w.id) { found = &img; break; }
        }
        if (!found) {
            slaveImages.emplace_back();
            found = &slaveImages.back();
            found->id = w.id;
        }
        if (found->crc != w.crc) {
            releaseImage(*found);
            found->crc = w.crc;
            found->width = w.width;
            found->height = w.height;
            found->lastRequest = 0; // ask right away
        }
    }
}

static void requestMissingImages(unsigned long now) {
    for (auto& img : slaveImages) {
        if (img.ready || img.crc == 0) continue;
        if (img.lastRequest != 0 && now - img.lastRequest < IMAGE_REQUEST_RETRY_MS) continue;
        img.lastRequest = now;
        sendImageRequest(img.id, img.crc);
    }
}

static void processImagePieces() {
    if (!imagePieceQueue) return;
    ImagePiece piece;
    while (xQueueReceive(imagePieceQueue, &piece, 0) == pdTRUE) {
        SlaveImage* img = nullptr;
        for (auto& candidate : slaveImages) {
            if (candidate.id == piece.id && candidate.crc == piece.crc) { img = &candidate; break; }
        }
        if (!img || img->ready) continue; // not (or no longer) wanted
        uint32_t expected = (uint32_t)piece.width * piece.height * 3;
        if (expected == 0 || expected > IMAGE_MAX_BYTES || piece.total != expected) continue;
        uint16_t pieces = (uint16_t)((expected + HYPERBUS_IMAGE_CHUNK_DATA - 1) / HYPERBUS_IMAGE_CHUNK_DATA);
        if (!img->data || img->total != expected) {
            releaseImage(*img);
            img->data = allocPixels(expected);
            if (!img->data) continue;
            img->total = (uint16_t)expected;
            img->width = piece.width;
            img->height = piece.height;
            img->have.assign(pieces, 0);
        }
        if (piece.offset % HYPERBUS_IMAGE_CHUNK_DATA != 0 || piece.offset >= expected) continue;
        uint16_t index = piece.offset / HYPERBUS_IMAGE_CHUNK_DATA;
        uint16_t want = (uint16_t)min((uint32_t)HYPERBUS_IMAGE_CHUNK_DATA, expected - piece.offset);
        if (piece.len != want) continue;
        memcpy(img->data + piece.offset, piece.data, piece.len);
        if (!img->have[index]) {
            img->have[index] = 1;
            img->haveCount++;
        }
        if (img->haveCount < pieces) continue;

        uint32_t crc = esp_rom_crc32_le(0, img->data, img->total);
        if (crc == 0) crc = 1;
        if (crc == img->crc) {
            img->ready = true;
            img->have.clear();
            localWidgetsDirty = true;
            Serial.printf("Widgets: image %u received (%ux%u)\n", (unsigned)img->id,
                          (unsigned)img->width, (unsigned)img->height);
        } else {
            // Pieces from two different transfers, or a corrupted one: start over.
            uint32_t want = img->crc;
            releaseImage(*img);
            img->crc = want;
            img->lastRequest = 0;
        }
    }
}

static bool sameLocalWidgets(const std::vector<LocalWidget>& a, const std::vector<LocalWidget>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); i++) {
        const LocalWidget& p = a[i];
        const LocalWidget& q = b[i];
        if (p.id != q.id || p.type != q.type || p.x != q.x || p.y != q.y || p.color != q.color ||
            p.scale != q.scale || p.format != q.format || p.font != q.font || p.speed != q.speed ||
            p.width != q.width || p.height != q.height || p.crc != q.crc || p.bri != q.bri ||
            p.legible != q.legible || p.text != q.text) {
            return false;
        }
    }
    return true;
}

static void setWidgetRects(const WidgetRect* rects, uint8_t count) {
    portENTER_CRITICAL(&widgetMux);
    if (count > 0) memcpy(widgetRects, rects, sizeof(WidgetRect) * count);
    widgetRectCount = count;
    portEXIT_CRITICAL(&widgetMux);
}

static void deactivateLocalWidgets() {
    localWidgetsActive = false;
    shownValid = false; // whatever takes over draws over our pixels
    setWidgetRects(nullptr, 0);
    localWidgets.clear();
    syncImagesWithWidgets();
}

// Applies a CMD_SET_WIDGETS payload. Runs in loop() only.
static void applyWidgetConfig(const uint8_t* p, uint16_t length) {
    if (length < HYPERBUS_WIDGET_HEADER_LEN) return;
    uint8_t flags = p[0];
    if (flags & HYPERBUS_WIDGET_FLAG_RELEASE) {
        if (localWidgetsActive) Serial.println("Widgets: released by the Master");
        // Pixels are left as they are: whatever takes over the panel draws over them.
        deactivateLocalWidgets();
        return;
    }

    bool on = (flags & HYPERBUS_WIDGET_FLAG_ON) != 0;
    bool masterLayer = (flags & HYPERBUS_WIDGET_FLAG_MASTER_LAYER) != 0;
    bool layout2 = (flags & HYPERBUS_WIDGET_FLAG_ALL_TYPES) != 0;
    const uint16_t fixedLen = layout2 ? HYPERBUS_WIDGET_ENTRY_V2_FIXED_LEN : HYPERBUS_WIDGET_ENTRY_FIXED_LEN;
    uint8_t brightness = p[1];
    uint8_t count = p[2];

    std::vector<LocalWidget> parsed;
    uint16_t off = HYPERBUS_WIDGET_HEADER_LEN;
    for (uint8_t i = 0; i < count && parsed.size() < LOCAL_WIDGET_MAX; i++) {
        if (off + fixedLen > length) break;
        LocalWidget w;
        w.id     = p[off];
        w.type   = p[off + 1];
        w.x      = (int16_t)(p[off + 2] | (p[off + 3] << 8));
        w.y      = (int16_t)(p[off + 4] | (p[off + 5] << 8));
        w.color  = ((uint32_t)p[off + 6] << 16) | ((uint32_t)p[off + 7] << 8) | p[off + 8];
        w.scale  = p[off + 9];
        w.format = p[off + 10];
        w.font   = p[off + 11];
        w.speed  = p[off + 12];
        w.width  = p[off + 13];
        uint8_t textLen;
        if (layout2) {
            w.height = p[off + 14];
            w.crc    = (uint32_t)p[off + 15] | ((uint32_t)p[off + 16] << 8) |
                       ((uint32_t)p[off + 17] << 16) | ((uint32_t)p[off + 18] << 24);
            textLen  = p[off + 19];
        } else {
            textLen  = p[off + 14];
        }
        off += fixedLen;
        if (off + textLen > length) break;
        char textBuf[65] = {0};
        memcpy(textBuf, &p[off], textLen < 64 ? textLen : 64);
        w.text = String(textBuf);
        off += textLen;
        parsed.push_back(std::move(w));
    }
    // Each element's own brightness, behind the entries. Only as many as were parsed: a list cut
    // short above leaves the rest at full.
    if ((flags & HYPERBUS_WIDGET_FLAG_ENTRY_BRIGHTNESS) && count <= LOCAL_WIDGET_MAX &&
        parsed.size() == count && off + count <= length) {
        for (uint8_t i = 0; i < count; i++) parsed[i].bri = p[off + i];
        off += count;
        // How each element stays readable over the background, right behind the brightness.
        if ((flags & HYPERBUS_WIDGET_FLAG_ENTRY_LEGIBILITY) && off + count <= length) {
            for (uint8_t i = 0; i < count; i++) {
                uint8_t legible = p[off + i];
                parsed[i].legible = legible <= WidgetRender::LEGIBLE_MAX ? legible : WidgetRender::LEGIBLE_OUTLINE;
            }
        }
    }

    bool wasActive = localWidgetsActive;
    // The Master repeats the config every 2s. An unchanged one must change nothing on screen -
    // treating each repeat as new wiped the Master's clock and weather every two seconds.
    if (wasActive && on == localWidgetsOn && masterLayer == localWidgetsMasterLayer &&
        brightness == localWidgetsBrightness && sameLocalWidgets(parsed, localWidgets)) {
        return;
    }

    if (strip && ledType == TYPE_HUB75) {
        if (!masterLayer && (!wasActive || localWidgetsMasterLayer)) {
            // The panel is now ours alone. No black clear first: the next frame rewrites every
            // pixel with what belongs there, which leaves nothing an earlier effect drew.
            shownValid = false;
        } else if (wasActive && masterLayer) {
            // Erase what our previous elements drew. The stream's frame is black there and will
            // not resend it, since as far as the Master knows it already arrived.
            for (uint8_t i = 0; i < widgetRectCount; i++) clearWidgetRect(widgetRects[i]);
            shownValid = false;
        }
    }

    WidgetRect rects[LOCAL_WIDGET_MAX];
    uint8_t rectCount = 0;
    if (on) {
        for (const auto& w : parsed) {
            WidgetRect r = widgetRect(w);
            if (r.w > 0 && r.h > 0) rects[rectCount++] = r;
        }
    }

    localWidgets = std::move(parsed);
    localWidgetsOn = on;
    localWidgetsMasterLayer = masterLayer;
    localWidgetsBrightness = brightness;
    localRenderActive = false; // mutually exclusive with a normal locally-rendered effect
    setWidgetRects(rects, rectCount);
    localWidgetsActive = true;
    localWidgetsDirty = true;
    syncImagesWithWidgets();

    // Only on a real change (a WebUI edit), never per packet - see the USB-CDC note elsewhere.
    Serial.printf("Widgets: %u drawn locally (%s, %s)\n",
                  (unsigned)localWidgets.size(), on ? "on" : "off",
                  masterLayer ? "Master streams the rest" : "panel is ours");
}

// Set when the Master drives the animation itself (sync mode) and we must draw right away
// instead of waiting for our own frame timer.
static volatile bool forceRender = false;

// A frame arrives as dozens of chunks. Pushing the strip out from the receive callback on every
// one of them blocks that callback for the whole transfer, and the chunks arriving meanwhile are
// dropped - always the later ones, which are the lower right of a panel. loop() pushes once
// instead, so receiving stays cheap and the whole frame lands.
static volatile bool pendingShow = false;

static void sendPong(bool wireless) {
    // PONG Payload: [LED Count L] [LED Count H] [Version Length] [Version String...]
    //               [ledType] [matrixW L] [matrixW H] [matrixH L] [matrixH H] [shiftDriver]
    //               [Name...]
    //
    // The six configuration bytes were added in 0.2.001. Everything a Slave was told is stored
    // here and nowhere else, so without reporting it back the Master could not tell a HUB75
    // panel from a strip - the web UI then showed its default LED type, and saving anything on
    // that page silently pushed that default back over the real configuration. The Master keys
    // off the version string it parses first, so older Slaves that stop after the version are
    // still read correctly.
    uint8_t verLen = slaveVersion.length();
    uint16_t len = 2 + 1 + verLen + 6 + slaveName.length();
    uint8_t* payload = (uint8_t*)malloc(len);
    if (!payload) return;
    payload[0] = ledCount & 0xFF;
    payload[1] = (ledCount >> 8) & 0xFF;
    payload[2] = verLen;
    memcpy(&payload[3], slaveVersion.c_str(), verLen);
    uint16_t o = 3 + verLen;
    payload[o]     = ledType;
    payload[o + 1] = matrixWidth & 0xFF;
    payload[o + 2] = (matrixWidth >> 8) & 0xFF;
    payload[o + 3] = matrixHeight & 0xFF;
    payload[o + 4] = (matrixHeight >> 8) & 0xFF;
    payload[o + 5] = hub75ShiftDriver;
    memcpy(&payload[o + 6], slaveName.c_str(), slaveName.length());

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
                // The Master switched this segment back to a normal effect, which draws the whole
                // panel. The widget list itself belongs to loop() and is simply replaced next time.
                localWidgetsActive = false;
                shownValid = false;
                setWidgetRects(nullptr, 0);
                const uint8_t* p = packet.payload;
                localEffect.effect     = p[0];
                localEffectBrightness  = p[1];
                localEffect.brightness = (ledType == TYPE_HUB75) ? 255 : p[1];
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
                    Serial.printf("Rendering effect %u locally (%ux%u, window %u+%u, bri %u)" "\n",
                                  localEffect.effect, slaveSink.matrixWidth(), slaveSink.matrixHeight(),
                                  slaveSink.windowOffset, slaveSink.windowTotal, localEffectBrightness);
                }
                localRenderActive = true;
            }
        }
        else if (packet.command == CMD_SET_WIDGETS) {
            // Only hand the payload over - applying it touches the widget list and the panel, both
            // of which belong to loop() (see applyWidgetConfig).
            if (packet.length >= HYPERBUS_WIDGET_HEADER_LEN &&
                packet.length <= HYPERBUS_WIDGETS_MAX_PAYLOAD) {
                portENTER_CRITICAL(&widgetMux);
                memcpy(pendingWidgetPayload, packet.payload, packet.length);
                pendingWidgetLen = packet.length;
                pendingWidgetConfig = true;
                portEXIT_CRITICAL(&widgetMux);
                lastWidgetPacketAt = millis();
                widgetsOverEspNow = packet.isWireless;
            }
        }
        else if (packet.command == CMD_SET_BACKGROUND) {
            // Handed to loop() like the widget config.
            if (packet.length >= HYPERBUS_BACKGROUND_PAYLOAD_LEN) {
                portENTER_CRITICAL(&widgetMux);
                memcpy(pendingBackground, packet.payload, HYPERBUS_BACKGROUND_PAYLOAD_LEN);
                pendingBackgroundConfig = true;
                portEXIT_CRITICAL(&widgetMux);
            }
        }
        else if (packet.command == CMD_SET_TIME) {
            if (packet.length >= HYPERBUS_TIME_PAYLOAD_LEN) {
                const uint8_t* p = packet.payload;
                uint32_t epoch = (uint32_t)p[1] | ((uint32_t)p[2] << 8) |
                                 ((uint32_t)p[3] << 16) | ((uint32_t)p[4] << 24);
                uint16_t ms = (uint16_t)(p[5] | (p[6] << 8));
                if (ms > 999) ms = 999;
                portENTER_CRITICAL(&infoMux);
                clockEpoch = epoch;
                clockBaseMs = millis() - ms; // the moment that second began here
                clockKnown = true;
                portEXIT_CRITICAL(&infoMux);
            }
        }
        else if (packet.command == CMD_SET_WEATHER) {
            if (packet.length >= HYPERBUS_WEATHER_PAYLOAD_LEN) {
                const uint8_t* p = packet.payload;
                portENTER_CRITICAL(&infoMux);
                slaveWeather.valid = (p[0] & HYPERBUS_WEATHER_FLAG_VALID) != 0;
                slaveWeather.temp = (int16_t)(p[1] | (p[2] << 8));
                slaveWeather.icon = p[3];
                portEXIT_CRITICAL(&infoMux);
            }
        }
        else if (packet.command == CMD_WIDGET_IMAGE) {
            // Only queued - the image store belongs to loop().
            if (imagePieceQueue && packet.length > HYPERBUS_IMAGE_CHUNK_HEADER &&
                packet.length <= HYPERBUS_IMAGE_CHUNK_HEADER + HYPERBUS_IMAGE_CHUNK_DATA) {
                const uint8_t* p = packet.payload;
                ImagePiece piece;
                piece.id = p[0];
                piece.width = p[1];
                piece.height = p[2];
                piece.crc = (uint32_t)p[3] | ((uint32_t)p[4] << 8) |
                            ((uint32_t)p[5] << 16) | ((uint32_t)p[6] << 24);
                piece.offset = (uint16_t)(p[7] | (p[8] << 8));
                piece.total = (uint16_t)(p[9] | (p[10] << 8));
                piece.len = (uint8_t)(packet.length - HYPERBUS_IMAGE_CHUNK_HEADER);
                memcpy(piece.data, &p[HYPERBUS_IMAGE_CHUNK_HEADER], piece.len);
                xQueueSend(imagePieceQueue, &piece, 0);
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
              // Widgets stay on: in widget mode the Master streams the rest of the panel alongside
              // them, and their rectangles are kept out of the stream below.
              localRenderActive = false;  // the Master is driving the pixels again
              if (!localWidgetsMasterLayer) shownValid = false;
              WidgetRect rects[LOCAL_WIDGET_MAX];
              uint8_t rectCount = copyWidgetRects(rects);
              if (strip && packet.length >= 5) {
                  uint16_t maxLeds = min((int)ledCount, (int)(packet.length / 5));
                  for (uint16_t i = 0; i < maxLeds; i++) {
                      if (rectCount > 0 && pixelInWidgetRects(i, rects, rectCount)) continue;
                      uint8_t r = packet.payload[i * 5];
                      uint8_t g = packet.payload[i * 5 + 1];
                      uint8_t b = packet.payload[i * 5 + 2];
                      uint8_t w = packet.payload[i * 5 + 3];
                      uint8_t w2 = packet.payload[i * 5 + 4];
                      strip->SetPixelColor(i, r, g, b, w, w2);
                  }
                  pendingShow = true;
              }
          }
        else if (packet.command == CMD_SET_LEDS_CHUNK) {
              // Payload: [OffsetL] [OffsetH] [RGBW array]
              localRenderActive = false;  // the Master is driving the pixels again
              if (!localWidgetsMasterLayer) shownValid = false;
              WidgetRect rects[LOCAL_WIDGET_MAX];
              uint8_t rectCount = copyWidgetRects(rects);
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
                      if (rectCount > 0 && pixelInWidgetRects(offset + i, rects, rectCount)) continue;
                      uint8_t r = packet.payload[2 + i * 5];
                      uint8_t g = packet.payload[2 + i * 5 + 1];
                      uint8_t b = packet.payload[2 + i * 5 + 2];
                      uint8_t w = packet.payload[2 + i * 5 + 3];
                      uint8_t w2 = packet.payload[2 + i * 5 + 4];
                      strip->SetPixelColor(offset + i, r, g, b, w, w2);
                  }
                  pendingShow = true;
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

// Chooses how the HUB75 panel is dimmed. Whatever this Slave draws itself (its elements with the
// whole panel, or a local effect) goes out at full scale and the driver dims it, which keeps the
// colours right at low brightness (see BusHub75::setBrightness). A stream from the Master arrives
// already dimmed, so the driver goes to full for it.
static void updatePanelBrightness() {
    if (!strip || ledType != TYPE_HUB75) return;
    uint8_t want = 255;
    bool ownDrawing = false;
    if (localWidgetsActive) {
        if (!localWidgetsMasterLayer) {
            want = localWidgetsBrightness;
            ownDrawing = true;
        }
    } else if (localRenderActive) {
        want = localEffectBrightness;
        ownDrawing = true;
    }
    if (want == panelBrightnessApplied) return;

    if (!ownDrawing && matrixWidth > 0 && matrixHeight > 0) {
        // Back to a stream: what is on the panel was drawn at full scale for a dimmed driver and
        // would flash up at full brightness until the stream has repainted it. Blank it first.
        uint32_t count = (uint32_t)matrixWidth * matrixHeight;
        for (uint32_t i = 0; i < count; i++) strip->SetPixelColor((uint16_t)i, 0, 0, 0, 0, 0);
    }
    strip->setBrightness(want);
    panelBrightnessApplied = want;
    // The part of the dimming done in the pixel values changed with it: redraw everything.
    shownValid = false;
    forceRender = true;
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

    imagePieceQueue = xQueueCreate(IMAGE_PIECE_QUEUE_LEN, sizeof(ImagePiece));
    if (!imagePieceQueue) Serial.println("Widgets: could not create the image queue");

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
    // Push a streamed frame once, outside the receive callback (see pendingShow).
    if (pendingShow && strip) {
        pendingShow = false;
        strip->Show();
    }

    updatePanelBrightness();

    if (localRenderActive && strip) {
        if (forceRender) {
            forceRender = false;
            EffectEngine::draw(localEffect, slaveSink);
            strip->Show();
        } else if (EffectEngine::render(localEffect, slaveSink, millis())) {
            strip->Show();
        }
    }

    // Apply the latest widget config the receive path handed over.
    {
        uint8_t widgetPayload[HYPERBUS_WIDGETS_MAX_PAYLOAD];
        uint16_t widgetLen = 0;
        portENTER_CRITICAL(&widgetMux);
        if (pendingWidgetConfig) {
            widgetLen = pendingWidgetLen;
            memcpy(widgetPayload, pendingWidgetPayload, widgetLen);
            pendingWidgetConfig = false;
        }
        portEXIT_CRITICAL(&widgetMux);
        if (widgetLen > 0) applyWidgetConfig(widgetPayload, widgetLen);

        uint8_t bgPayload[HYPERBUS_BACKGROUND_PAYLOAD_LEN];
        bool bgPending = false;
        portENTER_CRITICAL(&widgetMux);
        if (pendingBackgroundConfig) {
            memcpy(bgPayload, pendingBackground, sizeof(bgPayload));
            pendingBackgroundConfig = false;
            bgPending = true;
        }
        portEXIT_CRITICAL(&widgetMux);
        if (bgPending) applyBackgroundConfig(bgPayload);
    }

    if (localWidgetsActive) {
        // Read the receive time before taking `now`: the other way round a packet landing in
        // between makes now - last wrap around and look like a timeout.
        unsigned long lastPacket = lastWidgetPacketAt;
        if ((long)(millis() - lastPacket) > (long)LOCAL_WIDGETS_TIMEOUT_MS) {
            Serial.println("Widgets: no config from the Master for 10s, stopping");
            deactivateLocalWidgets();
        }
    }

    if (localWidgetsActive && strip) {
        processImagePieces();
        unsigned long now = millis();
        requestMissingImages(now);

        // A Lauftext needs a steady ~20 fps to scroll smoothly. Everything else changes at most once
        // a second (clock, weather) or on an event (image arrived, config changed); four times a
        // second is plenty for those, and a frame with nothing new writes nothing.
        bool animated = false;
        if (localWidgetsOn) {
            for (const auto& w : localWidgets) {
                if (w.type == WidgetRender::TYPE_MARQUEE) { animated = true; break; }
            }
        }
        static unsigned long lastWidgetFrame = 0;
        unsigned long interval = animated ? 50 : 250;
        bool backgroundMoved = advanceBackground(now);
        if (localWidgetsDirty || backgroundMoved || now - lastWidgetFrame >= interval) {
            localWidgetsDirty = false;
            lastWidgetFrame = now;
            renderLocalWidgets(now);
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























