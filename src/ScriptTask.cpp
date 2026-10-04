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
#include "ScriptTask.h"

#include <esp_heap_caps.h>
#include <esp_rom_crc.h>

namespace Script {

namespace {
constexpr unsigned long FAILURE_WINDOW_MS = 60000;
}

// The frame a script draws: a plain buffer, in PSRAM when there is some (a 64x64 frame is 12 KB),
// and a published copy the other tasks read, so nobody ever sees half a frame.
class Task::FrameCanvas : public Canvas {
public:
    FrameCanvas(uint16_t w, uint16_t h) : _w(w), _h(h) {
        size_t bytes = (size_t)w * h * 3;
        _pixels = alloc(bytes);
        _copy = alloc(bytes);
    }
    ~FrameCanvas() {
        heap_caps_free(_pixels);
        heap_caps_free(_copy);
    }
    bool ok() const { return _pixels && _copy; }
    int width() const override { return _w; }
    int height() const override { return _h; }
    void setPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b) override {
        if (x < 0 || y < 0 || x >= _w || y >= _h) return;
        uint8_t* p = &_pixels[((size_t)y * _w + x) * 3];
        p[0] = r; p[1] = g; p[2] = b;
    }
    void fill(uint8_t r, uint8_t g, uint8_t b) override {
        for (size_t i = 0; i < (size_t)_w * _h; i++) { _pixels[i * 3] = r; _pixels[i * 3 + 1] = g; _pixels[i * 3 + 2] = b; }
    }
    size_t bytes() const { return (size_t)_w * _h * 3; }
    void publish() { memcpy(_copy, _pixels, bytes()); }
    const uint8_t* published() const { return _copy; }
    uint32_t crc() const { return esp_rom_crc32_le(0, _copy, bytes()); }

private:
    static uint8_t* alloc(size_t bytes) {
        uint8_t* p = (uint8_t*)heap_caps_calloc(bytes, 1, MALLOC_CAP_SPIRAM);
        if (!p) p = (uint8_t*)heap_caps_calloc(bytes, 1, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        return p;
    }
    uint16_t _w, _h;
    uint8_t* _pixels = nullptr;
    uint8_t* _copy = nullptr;
};

bool Task::start(uint16_t width, uint16_t height, const Limits& limits) {
    if (_task || width == 0 || height == 0) return false;
    if (!_lock) _lock = xSemaphoreCreateRecursiveMutex();
    if (!_lock) return false;
    FrameCanvas* canvas = new FrameCanvas(width, height);
    if (!canvas->ok()) {
        delete canvas;
        return false;
    }
    _width = width;
    _height = height;
    _limits = limits;
    _canvas = canvas;
    _stopRequested = false;
    _pendingText.clear();
    _hasPendingScript = false;
    _unloadRequested = false;
    _requestedCrc = 0;
    _valuesNew = false;
    _on = true;
    _frameReady = false;
    _active = false;
    _status = Wire::Status();
    _loaded = false;
    _blanked = false;
    _haveValues = false;
    _settings.clear();
    _values.clear();
    _consecutiveFailures = 0;
    _failuresInWindow = 0;
    _failureWindowStart = 0;
    _frameTimeSum = 0;
    _frameCount = 0;
    // Core 0 next to the WiFi stack and below it in priority: a script cannot hold up the main loop.
    TaskHandle_t handle = nullptr;
    if (xTaskCreatePinnedToCore(&Task::entry, "script", limits.stackBytes, this, 1, &handle, 0) != pdPASS) {
        delete _canvas;
        _canvas = nullptr;
        return false;
    }
    _task = handle;
    return true;
}

void Task::stop() {
    if (!_task) return;
    _stopRequested = true;
    // The task closes the script, frees the buffers and ends itself; a frame takes at most the time budget.
    for (int i = 0; i < 400 && _task; i++) vTaskDelay(pdMS_TO_TICKS(5));
}

void Task::entry(void* self) {
    static_cast<Task*>(self)->loop();
}

void Task::setScript(const uint8_t* text, size_t length, uint32_t crc) {
    if (!_task) return;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    if (crc != 0 && crc == _requestedCrc) {
        xSemaphoreGiveRecursive(_lock);
        return;
    }
    _pendingText.assign(text, text + length);
    _hasPendingScript = true;
    _unloadRequested = false;
    _requestedCrc = crc;
    xSemaphoreGiveRecursive(_lock);
}

void Task::unload() {
    if (!_task) return;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    _unloadRequested = true;
    _hasPendingScript = false;
    _pendingText.clear();
    _requestedCrc = 0;
    xSemaphoreGiveRecursive(_lock);
}

void Task::setValues(const std::vector<Item>& settings, const std::vector<Item>& values) {
    if (!_task) return;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    _pendingSettings = settings;
    _pendingValues = values;
    _valuesNew = true;
    xSemaphoreGiveRecursive(_lock);
}

void Task::setOn(bool on) {
    if (!_task) return;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    _on = on;
    xSemaphoreGiveRecursive(_lock);
}

bool Task::takeFrame(uint8_t* rgb, size_t bytes) {
    if (!_task) return false;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    bool ready = _frameReady && _canvas && _canvas->bytes() <= bytes;
    if (ready) {
        memcpy(rgb, _canvas->published(), _canvas->bytes());
        _frameReady = false;
    }
    xSemaphoreGiveRecursive(_lock);
    return ready;
}

uint32_t Task::crc() {
    if (!_task) return 0;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    uint32_t c = _requestedCrc;
    xSemaphoreGiveRecursive(_lock);
    return c;
}

Wire::Status Task::status() {
    Wire::Status s;
    if (!_task) return s;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    s = _status;
    xSemaphoreGiveRecursive(_lock);
    return s;
}

// ---------------------------------------------------------------------------------------------
// The task

void Task::closeHost(Wire::State state, uint8_t result, const String& message) {
    String text = message;  // may be the host's own message, which closing could drop
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    _host.close();
    _loaded = false;
    _active = false;
    _frameReady = false;
    _consecutiveFailures = 0;
    _frameTimeSum = 0;
    _frameCount = 0;
    _status.state = state;
    _status.result = result;
    _status.message = text;
    _status.fps = 0;
    _status.frameCrc = 0;
    _status.memoryKb = 0;
    _status.frameUs10 = 0;
    xSemaphoreGiveRecursive(_lock);
}

void Task::load(const std::vector<uint8_t>& text) {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    _status.state = Wire::STATE_LOADING;
    xSemaphoreGiveRecursive(_lock);
    // A new script starts on a dark canvas, not on what the one before it left behind.
    _canvas->fill(0, 0, 0);
    Result r = _host.load((const char*)text.data(), text.size(), *_canvas, _limits);
    if (r != Result::Ok) {
        closeHost(Wire::STATE_FAILED, (uint8_t)r, _host.message());
        return;
    }
    _loaded = true;
    _blanked = false;
    _nextFrameAt = millis();
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    _status.state = Wire::STATE_RUNNING;
    _status.result = 0;
    _status.message = "";
    xSemaphoreGiveRecursive(_lock);
    if (_haveValues) applyValues();
}

void Task::applyValues() {
    Result r = _host.setValues(_settings, _values);
    if (r == Result::Ok) return;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    _status.message = _host.message();
    _status.result = (uint8_t)r;
    xSemaphoreGiveRecursive(_lock);
    if (++_consecutiveFailures >= 3) closeHost(Wire::STATE_FAILED, (uint8_t)r, _host.message());
}

void Task::runFrame() {
    unsigned long now = millis();
    if ((long)(now - _nextFrameAt) < 0) return;
    _nextFrameAt = now + 1000 / _host.fps();
    Result r = _host.frame(now);
    if (r == Result::Ok) {
        _consecutiveFailures = 0;
        _frameTimeSum += _host.lastCallUs();
        _frameCount++;
        // Everything other tasks read is updated together, under the lock; the Lua state itself is
        // never touched outside this task.
        xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
        _canvas->publish();
        _status.frameCrc = _canvas->crc();
        _status.fps = (uint8_t)_host.fps();
        _status.memoryKb = (uint8_t)(_host.memoryUsed() / 1024);
        _status.frameUs10 = (uint16_t)min<unsigned long>(65535, _frameTimeSum / _frameCount / 100);
        _frameReady = true;
        _active = true;
        xSemaphoreGiveRecursive(_lock);
        return;
    }
    // A failed frame is skipped, the previous one stays. Three in a row, or ten in a minute, end the script.
    _consecutiveFailures++;
    if (now - _failureWindowStart > FAILURE_WINDOW_MS) {
        _failureWindowStart = now;
        _failuresInWindow = 0;
    }
    _failuresInWindow++;
    if (_consecutiveFailures >= 3 || _failuresInWindow >= 10) {
        closeHost(Wire::STATE_FAILED, (uint8_t)r, _host.message());
    } else {
        xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
        _status.message = _host.message();
        _status.result = (uint8_t)r;
        xSemaphoreGiveRecursive(_lock);
    }
}

void Task::loop() {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5));

        xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
        bool stopNow = _stopRequested;
        bool unload = _unloadRequested;
        _unloadRequested = false;
        std::vector<uint8_t> text;
        bool haveScript = _hasPendingScript;
        if (haveScript) {
            text.swap(_pendingText);
            _hasPendingScript = false;
        }
        bool valuesNew = _valuesNew;
        if (valuesNew) {
            _settings.swap(_pendingSettings);
            _values.swap(_pendingValues);
            _valuesNew = false;
            _haveValues = true;
        }
        bool on = _on;
        xSemaphoreGiveRecursive(_lock);

        if (stopNow) break;
        if (unload) closeHost(Wire::STATE_NONE, 0, "");
        if (haveScript) {
            closeHost(Wire::STATE_NONE, 0, "");
            load(text);
        } else if (valuesNew && _loaded) {
            applyValues();
        }
        if (!_loaded) continue;

        if (!on) {
            // Switched off: the script stays loaded, the frame goes dark once.
            if (!_blanked) {
                xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
                _canvas->fill(0, 0, 0);
                _canvas->publish();
                _status.frameCrc = _canvas->crc();
                _frameReady = true;
                _active = true;
                xSemaphoreGiveRecursive(_lock);
                _blanked = true;
            }
            continue;
        }
        _blanked = false;
        runFrame();
    }

    // Leaving: close the script and free the buffers before the task ends. Nobody reads the buffer
    // any more by then, but the pointer is taken away under the lock all the same.
    closeHost(Wire::STATE_NONE, 0, "");
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    FrameCanvas* old = _canvas;
    _canvas = nullptr;
    xSemaphoreGiveRecursive(_lock);
    delete old;
    _task = nullptr;
    vTaskDelete(nullptr);
}

}  // namespace Script
