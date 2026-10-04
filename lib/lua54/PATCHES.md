# Changes to the Lua sources

Lua 5.4.7, MIT licence (see `LICENSE-LUA.txt`, unchanged). Obtained from the PlatformIO package
`fischer-simon/Esp32Lua @ 5.4.7`; `luaconf.h` there already defines `LUA_32BITS` (32-bit integers and
floats), which stays. `lua.c`, `luac.c` and `onelua.c` (stand-alone programs with their own `main`)
are left out.

HyperLED changes three files for two reasons: a cheap time limit for scripts (`lvm.c`, `lstrlib.c`), and a lower nesting depth (`llimits.h`, `lstrlib.c`). The functions that
need it live in `ScriptHost.cpp`: the counter `hyperled_poll_ticks`, and `hyperled_script_expired(lua_State*)`,
which returns non-zero when the running call has used up its time budget.

Why not a debug hook? Any hook switches the Lua 5.4 VM into a slower tracing mode for every
instruction; measured on the ESP32-S3 that costs about 35 % (a 64x64 plasma script: 86 ms instead of
61 ms per frame). Asking the clock only where a script can run without end, and only at every 32nd such place, costs a few percent.

`lvm.c` (time limit)
- after `#define updatetrap`: declarations, the macro `hyperled_tick()` (counts, true at every 32nd call) and
  the macro `hyperled_poll(L)`, which raises "Zeitbudget ueberschritten" when the call is out of time
- `OP_JMP`: `if (GETARG_sJ(i) < 0) ProtectNT(hyperled_poll(L));` (a backward jump is a loop: `while`,
  `repeat`, `goto`)
- `donextjump`: the same check for a backward jump that follows a test, which Lua runs inside the
  test instruction instead of `OP_JMP` (the closing jump of `repeat ... until`)
- `OP_FORLOOP`: `ProtectNT(hyperled_poll(L));` replaces the lone `updatetrap(ci)` at the end
- `OP_TFORLOOP`: `ProtectNT(hyperled_poll(L));` after the jump back (generic `for`)
- `OP_CALL` and `OP_TAILCALL`: `hyperled_poll(L);` after the `savepc` (endless recursion and endless
  tail calls)

`lstrlib.c`
- `match()`: asks `hyperled_script_expired` at the start of every step, so a pattern that backtracks
  for ever (`string.find`, `gsub`, ...) is cut off although it runs no Lua code

`llimits.h`
- `LUAI_MAXCCALLS` 200 -> 32 (nested C calls and nested syntax levels). A script is untrusted input, and
  every level costs C stack in the task that runs it: measured on the ESP32-S3, one level costs about
  350 bytes in the worst case (a `__tostring` or `__index` metamethod that calls itself; nested function
  bodies are about 300). With 200 levels such a script overflowed even a 48 KB stack and rebooted the
  controller. With 32 the worst case is 13.3 KB, and the task has 16 KB (`Limits::stackBytes`); a script
  that nests deeper gets the error "C stack overflow" instead. Real scripts nest far less.

`lstrlib.c`
- `MAXCCALLS` 200 -> 100 (recursion depth of the pattern matcher, about 40 bytes a level).

When Lua is updated, these changes must be applied again, in the Master and in the Slave repository
(the two copies of `lib/lua54/` stay identical).
