"""
NeoPixelBus 2.8.4's ESP32-S3 "LCD" driving method (NeoEsp32LcdXMethod.h) fails to compile
against ESP-IDF 5.5+, which renamed gpio_hal_iomux_func_sel() - a known upstream bug
(https://github.com/Makuna/NeoPixelBus/issues/895), not yet released in a tagged version.
This is unrelated to HyperLED's own code; it just needs the vendored dependency patched
on every fresh fetch, since .pio/libdeps is not part of the repo. Idempotent - safe to
run on every build.
"""
Import("env")
import re

libdeps_dir = env.subst("$PROJECT_LIBDEPS_DIR")
pioenv = env["PIOENV"]
target_file = f"{libdeps_dir}/{pioenv}/NeoPixelBus/src/internal/methods/NeoEsp32LcdXMethod.h"

OLD_INCLUDE = "#include <hal/gpio_hal.h>\n"
NEW_INCLUDE = "#include <hal/gpio_hal.h>\n#include <esp_idf_version.h>\n"

OLD_LINE = "        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);\n"
NEW_LINES = (
    "#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)\n"
    "        gpio_iomux_out(pin, PIN_FUNC_GPIO, false);\n"
    "#else\n"
    "        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);\n"
    "#endif\n"
)

try:
    with open(target_file, "r") as f:
        content = f.read()
except FileNotFoundError:
    # Library not fetched yet on this run (e.g. very first dependency-resolution pass) -
    # PlatformIO re-invokes extra_scripts after lib_deps are installed, so nothing to do here.
    content = None

if content is not None and OLD_LINE in content:
    content = content.replace(OLD_INCLUDE, NEW_INCLUDE, 1)
    content = content.replace(OLD_LINE, NEW_LINES)
    with open(target_file, "w") as f:
        f.write(content)
    print(f"patch_neopixelbus.py: patched {target_file} for ESP-IDF 5.5+ compatibility")
