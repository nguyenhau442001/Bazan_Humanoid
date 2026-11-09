"""Vendor the STM32 HAL, CMSIS and FreeRTOS sources at pinned commits, like STM32CubeMX copies them.

    python firmware/stm32f411/tools/fetch_drivers.py           # (re)write Drivers/ and FreeRTOS/
    python firmware/stm32f411/tools/fetch_drivers.py --check   # fetch again and compare byte for byte (CI)

Everything is pinned by commit SHA (the tag is only a label). To upgrade, change the SHA, run the script,
build, and record why in the commit message.
"""
import filecmp
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

FW = Path(__file__).resolve().parents[1]

# HAL modules this firmware uses (keep in step with Core/Inc/stm32f4xx_hal_conf.h)
HAL_MODULES = ["cortex", "rcc", "gpio", "dma", "i2c", "uart", "adc", "tim", "iwdg", "pwr", "flash", "exti"]
HAL_HEADERS = ["hal", "hal_def"] + [f"hal_{m}" for m in HAL_MODULES] + [
    "hal_rcc_ex", "hal_gpio_ex", "hal_dma_ex", "hal_i2c_ex", "hal_adc_ex", "hal_tim_ex", "hal_pwr_ex",
    "hal_flash_ex", "hal_flash_ramfunc"]

# (folder, repo, tag, commit, [(glob in the repo, destination under FW)])
PINS = [
    ("hal", "STMicroelectronics/stm32f4xx-hal-driver", "v1.8.6", "6ef30cf0ef6cc29ad518f37b9e6de89b6b815f90", [
        *[(f"Inc/stm32f4xx_{m}.h", "Drivers/STM32F4xx_HAL_Driver/Inc") for m in HAL_HEADERS],
        ("Inc/stm32f4xx_ll_adc.h", "Drivers/STM32F4xx_HAL_Driver/Inc"),   # included by hal_adc.h
        ("Inc/Legacy/stm32_hal_legacy.h", "Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"),
        *[(f"Src/stm32f4xx_{m}.c", "Drivers/STM32F4xx_HAL_Driver/Src") for m in (
            "hal", "hal_cortex", "hal_rcc", "hal_rcc_ex", "hal_gpio", "hal_dma", "hal_dma_ex", "hal_i2c",
            "hal_i2c_ex", "hal_uart", "hal_adc", "hal_adc_ex", "hal_tim", "hal_tim_ex", "hal_iwdg", "hal_pwr",
            "hal_pwr_ex", "hal_flash", "hal_flash_ex", "hal_flash_ramfunc")],
        ("LICENSE.md", "Drivers/STM32F4xx_HAL_Driver"),
    ]),
    ("device", "STMicroelectronics/cmsis-device-f4", "v2.6.9", "3b64b7b0b546cefe84f13c65508126eea58b97d8", [
        ("Include/stm32f4xx.h", "Drivers/CMSIS/Device/ST/STM32F4xx/Include"),
        ("Include/stm32f411xe.h", "Drivers/CMSIS/Device/ST/STM32F4xx/Include"),
        ("Include/system_stm32f4xx.h", "Drivers/CMSIS/Device/ST/STM32F4xx/Include"),
        ("Source/Templates/gcc/startup_stm32f411xe.s", "Core/Startup"),
        ("Source/Templates/system_stm32f4xx.c", "Core/Src"),
        ("LICENSE.md", "Drivers/CMSIS/Device/ST/STM32F4xx"),
    ]),
    ("cmsis", "ARM-software/CMSIS_5", "5.9.0", "2b7495b8535bdcb306dac29b9ded4cfb679d7e5c", [
        *[(f"CMSIS/Core/Include/{h}", "Drivers/CMSIS/Include") for h in (
            "core_cm4.h", "cmsis_gcc.h", "cmsis_compiler.h", "cmsis_version.h", "mpu_armv7.h")],
        ("LICENSE.txt", "Drivers/CMSIS"),
    ]),
    ("freertos", "FreeRTOS/FreeRTOS-Kernel", "V11.1.0", "dbf70559b27d39c1fdb68dfb9a32140b6a6777a0", [
        *[(f"{c}.c", "FreeRTOS/Source")
          for c in ("tasks", "queue", "list", "timers", "event_groups", "stream_buffer")],
        ("include/*.h", "FreeRTOS/Source/include"),
        ("portable/GCC/ARM_CM4F/*", "FreeRTOS/Source/portable/GCC/ARM_CM4F"),
        ("portable/MemMang/heap_4.c", "FreeRTOS/Source/portable/MemMang"),
        # POSIX port: runs the same tasks on the host for the integration tests
        ("portable/ThirdParty/GCC/Posix/*.c", "FreeRTOS/Source/portable/ThirdParty/GCC/Posix"),
        ("portable/ThirdParty/GCC/Posix/*.h", "FreeRTOS/Source/portable/ThirdParty/GCC/Posix"),
        ("portable/ThirdParty/GCC/Posix/utils/*", "FreeRTOS/Source/portable/ThirdParty/GCC/Posix/utils"),
        ("LICENSE.md", "FreeRTOS"),
    ]),
]
OWNED = ["Drivers", "FreeRTOS/Source", "FreeRTOS/LICENSE.md", "Core/Startup/startup_stm32f411xe.s",
         "Core/Src/system_stm32f4xx.c"]


def clone(repo, tag, sha, into):
    subprocess.run(["git", "-c", "advice.detachedHead=false", "clone", "-q", "--depth", "1", "--branch", tag,
                    f"https://github.com/{repo}.git", str(into)], check=True)
    head = subprocess.run(["git", "-C", str(into), "rev-parse", "HEAD"], capture_output=True, text=True,
                          check=True).stdout.strip()
    if head != sha:
        sys.exit(f"{repo} {tag} is {head}, pinned {sha}: the tag moved, refusing to vendor")


def vendor(root):
    with tempfile.TemporaryDirectory() as tmp:
        for name, repo, tag, sha, items in PINS:
            src = Path(tmp) / name
            clone(repo, tag, sha, src)
            for pattern, dst in items:
                files = sorted(p for p in src.glob(pattern) if p.is_file())
                if not files:
                    sys.exit(f"{repo}: nothing matches {pattern}")
                for f in files:
                    out = root / dst / f.name
                    out.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(f, out)
    (root / "Drivers" / "VERSIONS.md").write_text(
        "# Vendored sources\n\nWritten by `tools/fetch_drivers.py`; do not edit by hand.\n\n"
        "| Source | Tag | Commit |\n| --- | --- | --- |\n"
        + "".join(f"| [{r}](https://github.com/{r}) | {t} | `{s}` |\n" for _, r, t, s, _ in PINS))


def tree(root):
    out = {}
    for o in OWNED:
        p = root / o
        for f in ([p] if p.is_file() else sorted(p.rglob("*"))):
            if f.is_file():
                out[f.relative_to(root).as_posix()] = f
    return out


def main():
    if "--check" not in sys.argv:
        for o in OWNED:
            p = FW / o
            if p.is_dir():
                shutil.rmtree(p)
            elif p.exists():
                p.unlink()
        vendor(FW)
        print(f"vendored {len(tree(FW))} files")
        return
    with tempfile.TemporaryDirectory() as tmp:
        fresh = Path(tmp) / "fw"
        vendor(fresh)
        a, b = tree(FW), tree(fresh)
        bad = sorted(set(a) ^ set(b)) + [k for k in sorted(set(a) & set(b)) if not filecmp.cmp(a[k], b[k], False)]
        if bad:
            print("vendored sources differ from the pinned commits:", *bad[:20], sep="\n  ")
            sys.exit(1)
        print(f"{len(a)} vendored files match the pinned commits")


if __name__ == "__main__":
    main()
