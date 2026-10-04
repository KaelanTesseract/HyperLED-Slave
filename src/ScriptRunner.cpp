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
#include "ScriptRunner.h"

#include <esp_rom_crc.h>

ScriptRunnerClass ScriptRunner;

namespace {

// The Master stops saying what should run, or sends nothing at all: after this the script ends by
// itself, like the element lists do.
constexpr unsigned long CONFIG_TIMEOUT_MS = 10000;
constexpr unsigned long REQUEST_INTERVAL_MS = 1000;
constexpr unsigned long STATUS_INTERVAL_MS = 5000;
constexpr unsigned long START_RETRY_MS = 2000;

}  // namespace

void ScriptRunnerClass::begin() {
    _lock = xSemaphoreCreateRecursiveMutex();
}

void ScriptRunnerClass::setGeometry(uint16_t width, uint16_t height) {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    if (width != _width || height != _height) {
        _width = width;
        _height = height;
        _releaseRequested = true;  // service() ends the script; the next one starts on the new size
    }
    xSemaphoreGiveRecursive(_lock);
}

// ---------------------------------------------------------------------------------------------
// Receive path: copy and return

void ScriptRunnerClass::onConfig(const uint8_t* payload, size_t length) {
    Script::Wire::Config config;
    if (!Script::Wire::decodeConfig(payload, length, config)) return;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    if (config.flags & Script::Wire::FLAG_RELEASE) {
        _releaseRequested = true;
        _haveConfig = false;
    } else {
        // A different script (or none so far): the text received so far is of no use.
        if (!_haveConfig || config.crc != _textCrc || config.length != _textTotal) {
            _text.assign(config.length, 0);
            _textTotal = config.length;
            _textCrc = config.crc;
            _textHave = 0;
            _textHanded = false;
            _statusDirty = true;
        }
        _config = config;
        _haveConfig = true;
        _configAt = millis();
        _brightness = config.brightness;
    }
    xSemaphoreGiveRecursive(_lock);
}

void ScriptRunnerClass::onChunk(const uint8_t* payload, size_t length) {
    Script::Wire::Chunk chunk;
    if (!Script::Wire::decodeChunk(payload, length, chunk)) return;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    if (_haveConfig && chunk.crc == _textCrc && chunk.total == _textTotal) {
        memcpy(&_text[chunk.offset], chunk.data, chunk.len);
        // Pieces arrive in order; a missing one leaves _textHave short, or the CRC check fails,
        // which makes the Slave ask again.
        if ((size_t)chunk.offset + chunk.len > _textHave) _textHave = (size_t)chunk.offset + chunk.len;
    }
    xSemaphoreGiveRecursive(_lock);
}

void ScriptRunnerClass::onValues(const uint8_t* payload, size_t length) {
    if (length < 2 || length > Script::Wire::VALUES_MAX) return;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    _values.assign(payload, payload + length);
    _valuesNew = true;
    xSemaphoreGiveRecursive(_lock);
}

// ---------------------------------------------------------------------------------------------
// Main loop side

void ScriptRunnerClass::service() {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    bool release = _releaseRequested;
    _releaseRequested = false;
    bool timedOut = _haveConfig && (millis() - _configAt > CONFIG_TIMEOUT_MS);
    if (timedOut) _haveConfig = false;
    bool have = _haveConfig;
    Script::Wire::Config config = _config;
    uint16_t w = _width, h = _height;
    xSemaphoreGiveRecursive(_lock);

    // The Master let go, went quiet, or the geometry changed: the script and the buffers go.
    if (release || timedOut) {
        _task.unload();
        _task.stop();
        _valuesSequenceApplied = 0xFF;
        _startFailed = false;
        return;
    }
    if (!have) return;

    if (!_task.running()) {
        if (w == 0 || h == 0) return;
        if (_startFailed && millis() - _startFailedAt < START_RETRY_MS) return;
        Script::Limits limits;
        limits.memoryBytes = 32768;
        // The budget is for one call into the script, in the task of its own (the main loop and the bus
        // never wait for it): it cuts off an endless loop and bounds how late a frame can be. A big panel
        // takes longer to fill - a 64x64 plasma is about 68 ms - so it gets as much as on the Master.
        limits.budgetUs = (uint32_t)w * h > 1024 ? 120000 : 40000;
        limits.useSpiram = true;  // measured: no slower, and it keeps the internal RAM for the panel driver
        _startFailed = !_task.start(w, h, limits);
        _startFailedAt = millis();
        if (_startFailed) return;
        _valuesSequenceApplied = 0xFF;
    }

    // The text is handed over once it is complete and intact.
    bool handText = false;
    std::vector<uint8_t> text;
    bool valuesNew = false;
    std::vector<uint8_t> values;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    if (!_textHanded && _textTotal > 0 && _textHave >= _textTotal) {
        if (esp_rom_crc32_le(0, _text.data(), _text.size()) == _textCrc) {
            handText = true;
            text = _text;
            _textHanded = true;
        } else {
            _textHave = 0;  // a piece was damaged or missing: start again, the Slave asks for it
        }
    }
    if (_valuesNew) {
        valuesNew = true;
        values.swap(_values);
        _valuesNew = false;
    }
    xSemaphoreGiveRecursive(_lock);

    if (handText) _task.setScript(text.data(), text.size(), config.crc);
    if (valuesNew) {
        std::vector<Script::Item> settings, vals;
        uint8_t sequence = 0;
        // The Master repeats the values every 2 s; the script is only told when they changed.
        if (Script::Wire::decodeItems(values.data(), values.size(), sequence, settings, vals) &&
            sequence != _valuesSequenceApplied) {
            _valuesSequenceApplied = sequence;
            _task.setValues(settings, vals);
        }
    }
    _task.setOn((config.flags & Script::Wire::FLAG_ON) != 0);
}

bool ScriptRunnerClass::takeFrame(uint8_t* rgb, size_t bytes, uint16_t& width, uint16_t& height) {
    if (!_task.takeFrame(rgb, bytes)) return false;
    width = _task.width();
    height = _task.height();
    return true;
}

bool ScriptRunnerClass::wantRequest(uint32_t& crc) {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    bool want = _haveConfig && !_textHanded && _textTotal > 0 && _textHave < _textTotal &&
                millis() - _lastRequestAt >= REQUEST_INTERVAL_MS;
    if (want) {
        crc = _textCrc;
        _lastRequestAt = millis();
    }
    xSemaphoreGiveRecursive(_lock);
    return want;
}

bool ScriptRunnerClass::wantStatus(Script::Wire::Status& status) {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    bool have = _haveConfig;
    bool dirty = _statusDirty;
    xSemaphoreGiveRecursive(_lock);
    if (!have) return false;

    Script::Wire::Status current;
    if (_task.running()) {
        current = _task.status();
    } else if (_startFailed) {
        current.state = Script::Wire::STATE_FAILED;
        current.result = (uint8_t)Script::Result::OutOfMemory;
        current.message = "Kein Speicher für das Bild";
    }
    unsigned long now = millis();
    bool changed = dirty || current.state != _lastState || current.result != _lastResult || current.message != _lastMessage;
    if (!changed && now - _lastStatusAt < STATUS_INTERVAL_MS) return false;

    status = current;
    _lastState = current.state;
    _lastResult = current.result;
    _lastMessage = current.message;
    _lastStatusAt = now;
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    _statusDirty = false;
    xSemaphoreGiveRecursive(_lock);
    return true;
}

void ScriptRunnerClass::stop() {
    xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
    _releaseRequested = true;
    _haveConfig = false;
    xSemaphoreGiveRecursive(_lock);
}
