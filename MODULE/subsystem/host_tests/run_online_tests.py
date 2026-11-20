"""用真实 Subsystem 实现验证设备掉线诊断。"""

from pathlib import Path
import os
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[3]
    with tempfile.TemporaryDirectory(prefix="subsystem_online_") as directory:
        work = Path(directory)
        (work / "rtos_utility.hpp").write_text(
            "#pragma once\n"
            "#define taskENTER_CRITICAL() ((void)0)\n"
            "#define taskEXIT_CRITICAL() ((void)0)\n"
            "inline void* xTaskGetCurrentTaskHandle() {\n"
            "    static int task; return &task;\n"
            "}\n",
            encoding="utf-8",
        )
        executable = work / ("online_tests.exe" if os.name == "nt" else "online_tests")
        source = work / "online_tests.cpp"
        source.write_bytes(Path(__file__).with_name("online_tests.cpp.in").read_bytes())
        subprocess.run(
            [
                "g++", "-std=c++23", "-Wall", "-Wextra", "-Werror",
                "-fno-rtti", "-fno-exceptions", "-fno-threadsafe-statics",
                "-I", str(work),
                "-I", str(root / "MODULE/subsystem"),
                "-I", str(root / "MODULE/fault"),
                str(root / "MODULE/subsystem/subsystem.cpp"),
                str(source),
                "-o", str(executable),
            ],
            check=True,
        )
        subprocess.run([str(executable)], check=True)
    print("Online diagnostics tests passed")


if __name__ == "__main__":
    main()
