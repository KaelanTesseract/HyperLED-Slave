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
#include "ScriptHost.h"

#include <esp_heap_caps.h>
#include <math.h>

extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

namespace Script {

// What the allocator and the drawing functions need to know about their host. It lives as long as
// the Lua state does: Lua hands the pointer back on every allocation.
struct Env {
    uint32_t used = 0;
    uint32_t limit = 0;
    bool spiram = false;
    bool oom = false;       // an allocation was refused
    bool timedOut = false;  // the time limit cut a call short
    uint32_t deadline = 0;  // micros() at which the current call runs out of time
    uint32_t budgetUs = 0;
    Canvas* canvas = nullptr;
    String* message = nullptr;
    uint32_t lastLogMs = 0;
    bool logged = false;
};

Env* envOf(lua_State* L) {
    void* ud = nullptr;
    lua_getallocf(L, &ud);
    return static_cast<Env*>(ud);
}

namespace {

// Every byte a script gets goes through here, so the ceiling cannot be dodged.
void* allocate(void* ud, void* ptr, size_t osize, size_t nsize) {
    Env* e = static_cast<Env*>(ud);
    if (nsize == 0) {
        if (ptr) {
            e->used -= (uint32_t)osize;
            heap_caps_free(ptr);
        }
        return nullptr;
    }
    // For a new block osize is a type tag, not a size.
    size_t old = ptr ? osize : 0;
    if ((size_t)e->used - old + nsize > e->limit) {
        e->oom = true;
        return nullptr;  // Lua turns this into a "not enough memory" error
    }
    void* p = nullptr;
    if (e->spiram) p = heap_caps_realloc(ptr, nsize, MALLOC_CAP_SPIRAM);
    if (!p) p = heap_caps_realloc(ptr, nsize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!p) {
        e->oom = true;
        return nullptr;
    }
    e->used = (uint32_t)((size_t)e->used - old + nsize);
    return p;
}

// Measurements only (Limits::hookEvery > 0): the debug hook that the patched VM makes unnecessary.
// It costs about a third of the speed, which is why the time limit does not use it.
void hook(lua_State* L, lua_Debug*) {
    Env* e = envOf(L);
    if ((int32_t)(micros() - e->deadline) >= 0) {
        e->timedOut = true;
        luaL_error(L, "Zeitbudget ueberschritten");
    }
}

int toByte(lua_State* L, int arg) {
    float v = (float)luaL_checknumber(L, arg);
    if (v < 0) return 0;
    if (v > 255) return 255;
    return (int)v;
}

int toCoord(lua_State* L, int arg) {
    return (int)floorf((float)luaL_checknumber(L, arg));
}

int l_px(lua_State* L) {
    Env* e = envOf(L);
    e->canvas->setPixel(toCoord(L, 1), toCoord(L, 2), (uint8_t)toByte(L, 3), (uint8_t)toByte(L, 4), (uint8_t)toByte(L, 5));
    return 0;
}

int l_fill(lua_State* L) {
    envOf(L)->canvas->fill((uint8_t)toByte(L, 1), (uint8_t)toByte(L, 2), (uint8_t)toByte(L, 3));
    return 0;
}

int l_clear(lua_State* L) {
    envOf(L)->canvas->fill(0, 0, 0);
    return 0;
}

// hsv(h, s, v): hue 0..359 (any integer, it wraps), saturation and value 0..255 -> r, g, b
int l_hsv(lua_State* L) {
    int h = ((int)floorf((float)luaL_checknumber(L, 1)) % 360 + 360) % 360;
    int s = toByte(L, 2);
    int v = toByte(L, 3);
    int region = h / 60;
    int rem = (h % 60) * 255 / 60;
    int p = v * (255 - s) / 255;
    int q = v * (255 - s * rem / 255) / 255;
    int t = v * (255 - s * (255 - rem) / 255) / 255;
    int r, g, b;
    switch (region) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
    }
    lua_pushinteger(L, r);
    lua_pushinteger(L, g);
    lua_pushinteger(L, b);
    return 3;
}

// A message for the author. At most one a second, at most 80 characters: it is for reading, not
// for output, and it must not cost the device anything.
int l_log(lua_State* L) {
    Env* e = envOf(L);
    uint32_t now = millis();
    if (e->logged && now - e->lastLogMs < 1000) return 0;
    e->logged = true;
    e->lastLogMs = now;
    String text;
    int n = lua_gettop(L);
    for (int i = 1; i <= n; i++) {
        size_t len = 0;
        const char* s = luaL_tolstring(L, i, &len);
        if (i > 1) text += ' ';
        if (text.length() < 80) text += String(s).substring(0, 80);
        lua_pop(L, 1);
    }
    if (text.length() > 80) text = text.substring(0, 80);
    *e->message = text;
    return 0;
}

void openLibs(lua_State* L) {
    static const luaL_Reg libs[] = {
        {LUA_GNAME, luaopen_base}, {LUA_MATHLIBNAME, luaopen_math},
        {LUA_STRLIBNAME, luaopen_string}, {LUA_TABLIBNAME, luaopen_table}, {nullptr, nullptr}};
    for (const luaL_Reg* lib = libs; lib->func; lib++) {
        luaL_requiref(L, lib->name, lib->func, 1);
        lua_pop(L, 1);
    }
    // What the script must not reach. pcall and xpcall go too: inside a protected call a script
    // could swallow the time-budget error and carry on forever.
    static const char* const removed[] = {"dofile", "loadfile", "load", "collectgarbage", "warn", "pcall", "xpcall"};
    for (const char* name : removed) {
        lua_pushnil(L);
        lua_setglobal(L, name);
    }
    lua_getglobal(L, LUA_STRLIBNAME);
    lua_pushnil(L);
    lua_setfield(L, -2, "dump");
    lua_pop(L, 1);
}

void pushItems(lua_State* L, const std::vector<Item>& items) {
    lua_createtable(L, 0, (int)items.size());
    for (const Item& item : items) {
        switch (item.kind) {
            case Item::Number: lua_pushnumber(L, (lua_Number)item.number); break;
            case Item::Text: lua_pushlstring(L, item.text.c_str(), item.text.length()); break;
            case Item::Bool: lua_pushboolean(L, item.number != 0); break;
            default: continue;
        }
        lua_setfield(L, -2, item.name.c_str());
    }
}

}  // namespace

}  // namespace Script

// Used by the patched Lua VM (see lib/lua54/PATCHES.md). The counter is bumped at every backward
// jump, call and pattern-match step; every 32nd time the VM asks whether the running call has used
// up its time budget. Two tasks may bump it at once - it only needs to be roughly right.
extern "C" volatile unsigned int hyperled_poll_ticks = 0;

extern "C" int hyperled_script_expired(lua_State* L) {
    Script::Env* e = Script::envOf(L);
    if ((int32_t)(micros() - e->deadline) >= 0) {
        e->timedOut = true;
        return 1;
    }
    return 0;
}

namespace Script {

Result Host::load(const char* source, size_t length, Canvas& canvas, const Limits& limits) {
    close();
    _message = "";
    _started = false;
    _env = new Env;
    _env->limit = limits.memoryBytes;
    _env->spiram = limits.useSpiram;
    _env->budgetUs = limits.budgetUs;
    _env->canvas = &canvas;
    _env->message = &_message;

    _state = lua_newstate(allocate, _env);
    if (!_state) {
        _message = "Kein Speicher für das Skript";
        close();
        return Result::OutOfMemory;
    }
    if (limits.hookEvery > 0) lua_sethook(_state, hook, LUA_MASKCOUNT, (int)limits.hookEvery);
    openLibs(_state);

    lua_pushinteger(_state, canvas.width());
    lua_setglobal(_state, "W");
    lua_pushinteger(_state, canvas.height());
    lua_setglobal(_state, "H");
    lua_pushinteger(_state, canvas.width() * canvas.height());
    lua_setglobal(_state, "N");
    lua_pushinteger(_state, 30);
    lua_setglobal(_state, "fps");
    lua_pushcfunction(_state, l_px);
    lua_setglobal(_state, "px");
    lua_pushcfunction(_state, l_fill);
    lua_setglobal(_state, "fill");
    lua_pushcfunction(_state, l_clear);
    lua_setglobal(_state, "clear");
    lua_pushcfunction(_state, l_hsv);
    lua_setglobal(_state, "hsv");
    lua_pushcfunction(_state, l_log);
    lua_setglobal(_state, "log");
    lua_pushcfunction(_state, l_log);
    lua_setglobal(_state, "print");
    pushItems(_state, std::vector<Item>());
    lua_setglobal(_state, "settings");
    pushItems(_state, std::vector<Item>());
    lua_setglobal(_state, "v");

    // Text only: a precompiled chunk could hold anything.
    int status = luaL_loadbufferx(_state, source, length, "=script", "t");
    if (status != LUA_OK) {
        Result r = finish(status);
        close();
        return r == Result::RuntimeError ? Result::SyntaxError : r;
    }
    _env->deadline = micros() + _env->budgetUs;
    _env->timedOut = false;
    _env->oom = false;
    status = lua_pcall(_state, 0, 0, 0);
    if (status != LUA_OK) {
        Result r = finish(status);
        close();
        return r;
    }

    lua_getglobal(_state, "frame");
    bool hasFrame = lua_isfunction(_state, -1);
    lua_pop(_state, 1);
    if (!hasFrame) {
        _message = "Das Skript hat keine Funktion frame()";
        close();
        return Result::Missing;
    }
    Result r = call("init", 0, false);
    if (r != Result::Ok) {
        close();
        return r;
    }
    return Result::Ok;
}

// Turns the status of a failed call into a Result and keeps the message.
Result Host::finish(int status) {
    const char* text = lua_tostring(_state, -1);
    _message = text ? String(text) : String("Unbekannter Fehler");
    lua_pop(_state, 1);
    if (_env->timedOut) return Result::Timeout;
    if (status == LUA_ERRMEM || _env->oom) return Result::OutOfMemory;
    return Result::RuntimeError;
}

// Calls the global function `name` with `args` arguments already on the stack (pushed after the
// function: the caller pushes them first, so they are moved here).
Result Host::call(const char* name, int args, bool required) {
    lua_getglobal(_state, name);
    if (!lua_isfunction(_state, -1)) {
        lua_pop(_state, 1 + args);
        return required ? Result::Missing : Result::Ok;
    }
    if (args > 0) lua_insert(_state, -(args + 1));
    _env->deadline = micros() + _env->budgetUs;
    _env->timedOut = false;
    _env->oom = false;
    uint32_t start = micros();
    int status = lua_pcall(_state, args, 0, 0);
    _lastCallUs = micros() - start;
    if (status != LUA_OK) return finish(status);
    return Result::Ok;
}

Result Host::setValues(const std::vector<Item>& settings, const std::vector<Item>& values) {
    if (!_state) return Result::Missing;
    pushItems(_state, settings);
    lua_setglobal(_state, "settings");
    pushItems(_state, values);
    lua_setglobal(_state, "v");
    return call("update", 0, false);
}

Result Host::frame(uint32_t nowMs) {
    if (!_state) return Result::Missing;
    if (!_started) {
        _started = true;
        _startMs = nowMs;
        _lastMs = nowMs;
    }
    uint32_t t = (nowMs - _startMs) & 0x7FFFFFFF;
    uint32_t dt = nowMs - _lastMs;
    _lastMs = nowMs;
    lua_pushinteger(_state, (lua_Integer)t);
    lua_pushinteger(_state, (lua_Integer)(dt & 0x7FFFFFFF));
    return call("frame", 2, true);
}

void Host::close() {
    if (_state) {
        // Closing runs the script's finalizers (__gc); they get a time budget too.
        _env->deadline = micros() + _env->budgetUs;
        lua_close(_state);
        _state = nullptr;
    }
    delete _env;
    _env = nullptr;
}

int Host::fps() const {
    if (!_state) return 30;
    lua_getglobal(_state, "fps");
    int f = 30;
    if (lua_isnumber(_state, -1)) {
        float v = (float)lua_tonumber(_state, -1);
        f = v < 1 ? 1 : (v > 60 ? 60 : (int)v);
    }
    lua_pop(_state, 1);
    return f;
}

uint32_t Host::memoryUsed() const {
    return _env ? _env->used : 0;
}

Result Host::check(const char* source, size_t length, String& error) {
    Env env;
    env.limit = 65536;
    env.spiram = true;  // the check on installation must not touch the scarce internal RAM
    String message;
    env.message = &message;
    lua_State* L = lua_newstate(allocate, &env);
    if (!L) {
        error = "Kein Speicher für die Prüfung";
        return Result::OutOfMemory;
    }
    int status = luaL_loadbufferx(L, source, length, "=script", "t");
    Result r = Result::Ok;
    if (status != LUA_OK) {
        const char* text = lua_tostring(L, -1);
        error = text ? String(text) : String("Unbekannter Fehler");
        r = (status == LUA_ERRMEM || env.oom) ? Result::OutOfMemory : Result::SyntaxError;
    }
    lua_close(L);
    return r;
}

}  // namespace Script
