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

// Runs one script in a task of its own and draws it into a buffer. Used by the Master (one per
// script-driven segment) and by the Slave (one for its output); this file and ScriptTask.cpp are
// identical copies in both repositories - change both together.
//
// The Lua state belongs to the task; whatever other tasks read - the finished frame, the state - is
// kept under this class's lock. A slow script therefore holds up nobody else.

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <vector>
#include "ScriptHost.h"
#include "ScriptWire.h"

namespace Script {

class Task {
public:
    Task() {}
    ~Task() {
        stop();
        if (_lock && !_task) vSemaphoreDelete(_lock);  // the task is gone, nobody holds it
    }
    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;

    // Allocates the frame buffers (PSRAM when there is some) and starts the task. False: no memory.
    bool start(uint16_t width, uint16_t height, const Limits& limits);
    void stop();  // ends the task and frees everything
    bool running() const { return _task != nullptr; }
    uint16_t width() const { return _width; }
    uint16_t height() const { return _height; }

    // Commands: copied under the lock, carried out by the task. Safe from any task.
    // The same checksum again is ignored - that is what makes it cheap to hand the script over on
    // every pass - and a script that failed is not tried again until a different one is handed over
    // or unload() is called.
    void setScript(const uint8_t* text, size_t length, uint32_t crc);
    void unload();
    void setValues(const std::vector<Item>& settings, const std::vector<Item>& values);
    void setOn(bool on);  // off: the frame is darkened once and the script rests

    // True when a frame was finished since the last call; copies it (r, g, b per pixel, row by row).
    bool takeFrame(uint8_t* rgb, size_t bytes);
    // A script is loaded and has drawn. Lock-free, so it is safe from a receive callback.
    bool active() const { return _task != nullptr && _active; }
    // How much of the task's stack has never been used, in bytes: shows how close a script comes to
    // overflowing it (deep nesting costs C stack).
    uint32_t stackFree() const { return _task ? (uint32_t)uxTaskGetStackHighWaterMark(_task) : 0; }
    uint32_t crc();         // checksum of the script last handed over; 0 when there is none
    Wire::Status status();  // state, last error, frame rate and time, memory, checksum of the last frame

private:
    class FrameCanvas;
    static void entry(void* self);
    void loop();
    void closeHost(Wire::State state, uint8_t result, const String& message);
    void publishMessage();
    void load(const std::vector<uint8_t>& text);
    void applyValues();
    void runFrame();

    SemaphoreHandle_t _lock = nullptr;
    TaskHandle_t volatile _task = nullptr;
    volatile bool _stopRequested = false;
    uint16_t _width = 0, _height = 0;
    Limits _limits;

    // Commands, under _lock.
    std::vector<uint8_t> _pendingText;
    bool _hasPendingScript = false;
    bool _unloadRequested = false;
    uint32_t _requestedCrc = 0;
    std::vector<Item> _pendingSettings, _pendingValues;
    bool _valuesNew = false;
    bool _on = true;

    // Results, under _lock (_active is also read without it).
    bool _frameReady = false;
    volatile bool _active = false;
    Wire::Status _status;

    // Task only.
    Host _host;
    FrameCanvas* _canvas = nullptr;
    bool _loaded = false;
    bool _blanked = false;
    std::vector<Item> _settings, _values;  // kept, so a script that is loaded later still gets them
    bool _haveValues = false;
    unsigned long _nextFrameAt = 0;
    uint8_t _consecutiveFailures = 0;
    unsigned long _failureWindowStart = 0;
    uint8_t _failuresInWindow = 0;
    unsigned long _frameTimeSum = 0;
    uint32_t _frameCount = 0;
};

}  // namespace Script
