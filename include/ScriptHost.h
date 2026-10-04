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

// The script host: one Lua state behind a small interface, used by the Master and by the Slaves.
// This file and ScriptHost.cpp are identical copies in both repositories - change both together.
//
// A script sees only W, H, N, settings and v (read-only data) and a handful of drawing functions
// that write into the Canvas it was given. It has a memory ceiling and a time budget for every
// call into it; it cannot reach files, the network, or anything else of the device.
// See docs/en/11_Plugin_Skripte.md of the Master repository.

#include <Arduino.h>
#include <vector>

struct lua_State;

namespace Script {

enum class Result : uint8_t {
    Ok,
    SyntaxError,   // the script does not compile; message() names the line
    RuntimeError,  // an error while running; message() says what
    Timeout,       // a call took longer than its time budget
    OutOfMemory,   // the script wanted more than its memory ceiling
    Missing        // the script has no frame() function
};

// Where the pixels go. The host never touches a display itself.
class Canvas {
public:
    virtual ~Canvas() {}
    virtual int width() const = 0;
    virtual int height() const = 0;  // 1 for a strip
    // The host does not clip: an implementation ignores coordinates outside the segment.
    virtual void setPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b) = 0;
    virtual void fill(uint8_t r, uint8_t g, uint8_t b) = 0;
};

struct Limits {
    uint32_t memoryBytes = 32768;  // everything the script has allocated, in total
    uint32_t budgetUs = 40000;     // for each call: load, init, update, frame
    bool useSpiram = false;        // keep the Lua heap in PSRAM instead of internal RAM
    uint32_t stackBytes = 16384;   // the stack of the task that runs the script (see Script::Task). The
                                   // deepest nesting Lua allows here (LUAI_MAXCCALLS, see PATCHES.md) needs
                                   // up to 13.3 KB of it - measured on the device
    uint32_t hookEvery = 0;        // measurements only: also install a debug hook every n instructions (the time limit itself
                                   // is checked by the patched VM, see lib/lua54/PATCHES.md)
};

// One value handed to the script, by name. Nil items are left out of the table.
struct Item {
    enum Kind : uint8_t { Nil, Number, Text, Bool };
    String name;
    Kind kind = Nil;
    double number = 0;  // also holds a Bool as 0 or 1
    String text;

    static Item num(const String& name, double value) {
        Item i; i.name = name; i.kind = Number; i.number = value; return i;
    }
    static Item txt(const String& name, const String& value) {
        Item i; i.name = name; i.kind = Text; i.text = value; return i;
    }
    static Item flag(const String& name, bool value) {
        Item i; i.name = name; i.kind = Bool; i.number = value ? 1 : 0; return i;
    }
};

struct Env;  // the allocator's and the drawing functions' view of a host

class Host {
public:
    Host() {}
    ~Host() { close(); }
    Host(const Host&) = delete;
    Host& operator=(const Host&) = delete;

    // Compiles the script, runs its top level and init(). Only text is accepted, never
    // precompiled code. On any failure nothing stays open and message() says why.
    Result load(const char* source, size_t length, Canvas& canvas, const Limits& limits);

    // Hands the script new settings and values (tables `settings` and `v`) and calls update().
    Result setValues(const std::vector<Item>& settings, const std::vector<Item>& values);

    // Calls frame(t, dt). t is the milliseconds since the first frame, dt since the one before.
    Result frame(uint32_t nowMs);

    void close();
    bool isOpen() const { return _state != nullptr; }

    // The frame rate the script asked for through the global `fps`, 1..60, 30 by default.
    int fps() const;

    // The last log() text, or the text of the last error. At most 80 characters for log().
    const String& message() const { return _message; }
    uint32_t memoryUsed() const;
    uint32_t lastCallUs() const { return _lastCallUs; }

    // Compiles without running, in a throwaway state: is this script's text valid?
    static Result check(const char* source, size_t length, String& error);

private:
    lua_State* _state = nullptr;
    Env* _env = nullptr;
    String _message;
    uint32_t _startMs = 0;
    uint32_t _lastMs = 0;
    bool _started = false;
    uint32_t _lastCallUs = 0;

    Result call(const char* name, int args, bool required);
    Result finish(int status);
};

}  // namespace Script
