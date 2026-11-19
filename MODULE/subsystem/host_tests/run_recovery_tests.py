"""用真实状态树、状态回调和子系统实现验证恢复模式。"""

from pathlib import Path
import os
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[3]
    tests = Path(__file__).parent
    hfsm = root / "APP/robot/robot_hfsm"
    robot = (root / "APP/robot/robot.hpp").read_text(encoding="utf-8")
    context_and_tree = robot[robot.index("struct Robot_Context {"):]
    with tempfile.TemporaryDirectory(prefix="subsystem_recovery_") as directory:
        work = Path(directory)
        (work / "rtos_utility.hpp").write_text(
            "#pragma once\n"
            "#define taskENTER_CRITICAL() ((void)0)\n"
            "#define taskEXIT_CRITICAL() ((void)0)\n"
            "inline void* xTaskGetCurrentTaskHandle() { static int task; return &task; }\n",
            encoding="utf-8",
        )
        fixture = (tests / "recovery_robot.hpp.in").read_text(encoding="utf-8")
        (work / "robot.hpp").write_text(fixture + context_and_tree, encoding="utf-8")
        source = work / "recovery_tests.cpp"
        source.write_bytes((tests / "recovery_tests.cpp.in").read_bytes())
        executable = work / ("recovery_tests.exe" if os.name == "nt" else "recovery_tests")
        command = [
            "g++", "-std=c++23", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter",
            "-fno-rtti", "-fno-exceptions", "-fno-threadsafe-statics",
        ]
        for include in (work, hfsm, root / "Hfsm", root / "MODULE/subsystem", root / "MODULE/fault"):
            command.extend(["-I", str(include)])
        command.extend(str(path) for path in sorted((hfsm / "peer_root").rglob("*.cpp")))
        command.extend([str(root / "MODULE/subsystem/subsystem.cpp"), str(source), "-o", str(executable)])
        subprocess.run(command, check=True)
        for scenario in ("manual", "test", "outside_fault", "retry", "guard"):
            subprocess.run([str(executable), scenario], check=True)
        subprocess.run(command + ["-DEMPTY_TEST_SET"], check=True)
        subprocess.run([str(executable), "empty"], check=True)
    print("Recovery tests passed: manual, test, outside fault, retry, guard, empty set")


if __name__ == "__main__":
    main()
