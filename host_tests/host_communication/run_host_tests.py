from pathlib import Path
import os
import shutil
import subprocess


def main():
    tests = Path(__file__).resolve().parent
    root = tests.parents[1]
    output = root / "build" / "host_communication_tests"
    output.mkdir(parents=True, exist_ok=True)
    compiler = shutil.which(os.environ.get("CXX", "g++"))
    if compiler is None:
        raise SystemExit("找不到宿主机 g++，请设置 CXX")
    common = [compiler, "-std=c++23", "-Wall", "-Wextra", "-Werror", "-pedantic",
              "-fno-exceptions", "-fno-rtti", "-g"]
    includes = [tests / "stubs", root / "MODULE" / "host", root / "MODULE" / "frame_codec",
                root / "BSP" / "usb_port", root / "Algorithm"]
    for directory in includes:
        common += ["-I", str(directory)]
    cases = {
        "protocol": [tests / "protocol_test.cpp"],
        "usb_port": [tests / "usb_port_test.cpp", root / "BSP" / "usb_port" / "usb_port.cpp"],
    }
    for name, sources in cases.items():
        executable = output / (name + (".exe" if os.name == "nt" else ""))
        subprocess.run(common + [str(source) for source in sources] + ["-o", str(executable)],
                       check=True)
        subprocess.run([str(executable)], check=True)
        print(f"PASS: {name}", flush=True)


if __name__ == "__main__":
    main()
