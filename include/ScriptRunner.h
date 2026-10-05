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
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <vector>
#include "ScriptTask.h"
#include "ScriptWire.h"

// The Slave's side of the script commands: it assembles what the Master sends, hands it to a
// Script::Task (which runs Lua in a task of its own and draws into a buffer) and reports back. The
// receive path (a WiFi or UART callback) only copies bytes in; service() in loop() does the rest, and
// loop() also copies finished frames to the panel. See
// docs/en/11_Plugin_Skripte.md of the Master repository.
class ScriptRunnerClass {
public:
    void begin();
    void setGeometry(uint16_t width, uint16_t height);

    // From the receive path.
    void onConfig(const uint8_t* payload, size_t length);
    void onChunk(const uint8_t* payload, size_t length);
    void onValues(const uint8_t* payload, size_t length);
    void onData(const uint8_t* payload, size_t length);

    // From loop(): carries out what arrived (starts and ends the task, hands over the script and its
    // values, ends a script the Master has stopped talking about).
    void service();

    // A script owns the output right now. Lock-free: also called from the receive path.
    bool active() const { return _task.active(); }
    uint8_t brightness() const { return _brightness; }
    // Copies the latest finished frame (r, g, b per pixel, row by row) if there is a new one.
    bool takeFrame(uint8_t* rgb, size_t bytes, uint16_t& width, uint16_t& height);
    bool wantRequest(uint32_t& crc);                // the Slave should ask the Master for this script
    bool wantStatus(Script::Wire::Status& status);  // a status packet is due
    void stop();                                    // the main loop gives the output back

private:
    SemaphoreHandle_t _lock = nullptr;
    Script::Task _task;

    // geometry
    uint16_t _width = 0, _height = 0;

    // What the Master said (written by the receive path, read by service()), under _lock.
    Script::Wire::Config _config;
    bool _haveConfig = false;
    unsigned long _configAt = 0;
    std::vector<uint8_t> _text;  // the script as received
    uint16_t _textTotal = 0;
    uint32_t _textCrc = 0;
    size_t _textHave = 0;
    bool _textHanded = false;    // the complete, intact text is with the task
    std::vector<uint8_t> _values;  // the latest CMD_SET_SCRIPT_VALUES payload
    bool _valuesNew = false;
    // CMD_SET_SCRIPT_DATA packets as they came in (the receive path only copies them): a ring, a
    // packet that finds it full pushes out the oldest, the Master repeats everything anyway.
    static const uint8_t DATA_RING = 16;
    uint8_t _dataRing[DATA_RING][Script::Wire::VALUES_MAX];
    uint16_t _dataRingLen[DATA_RING];
    uint8_t _dataHead = 0, _dataCount = 0;
    bool _releaseRequested = false;
    bool _statusDirty = true;
    volatile uint8_t _brightness = 255;
    unsigned long _lastRequestAt = 0;

    // loop() only.
    uint8_t _valuesSequenceApplied = 0xFF;
    // The parts of the list of each kind (settings, values) that are being collected, and the lists
    // the script has now.
    struct DataAssembly {
        uint8_t sequence = 0xFF;
        uint8_t parts = 0;
        uint16_t have = 0;  // bit per part
        std::vector<Script::Item> part[Script::Wire::DATA_PARTS];
    };
    DataAssembly _assembly[2];
    uint8_t _dataApplied[2] = {0xFF, 0xFF};
    std::vector<Script::Item> _kept[2];
    void resetData();
    bool takeData();
    bool _startFailed = false;
    unsigned long _startFailedAt = 0;
    unsigned long _lastStatusAt = 0;
    uint8_t _lastState = 0xFF, _lastResult = 0xFF;
    String _lastMessage;
};

extern ScriptRunnerClass ScriptRunner;
