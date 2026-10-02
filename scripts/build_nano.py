"""Build the RISC-V collector using the installed WSL distribution."""

import pathlib
import subprocess
import json
import sys

if sys.stdout is not None:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if sys.stderr is not None:
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

ROOT = pathlib.Path(__file__).resolve().parents[1]
DISTRO = "Ubuntu-22.04"


def run(*args):
    print(" ".join(map(str, args)), flush=True)
    process = subprocess.Popen(
        list(map(str, args)),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        creationflags=subprocess.CREATE_NO_WINDOW,
    )
    stdout, stderr = process.communicate()
    for output in (stdout, stderr):
        if output:
            print(output.decode("utf-8", "replace").replace(chr(0), ""), flush=True)
    if process.returncode:
        raise subprocess.CalledProcessError(process.returncode, args)


def main():
    result = subprocess.run(
        ["wsl", "-d", DISTRO, "--exec", "wslpath", "-a", str(ROOT)],
        check=True,
        capture_output=True,
    )
    root = result.stdout.decode("utf-8").strip()
    compiler = json.loads(
        (ROOT / "config" / "toolchain.json").read_text(encoding="utf-8-sig")
    )["compiler"]
    prefix = ["wsl", "-d", DISTRO, "--exec"]
    run(
        *prefix,
        "cmake",
        "-S",
        root + "/nano",
        "-B",
        root + "/build/nano-riscv64",
        "-DCMAKE_SYSTEM_NAME=Linux",
        "-DCMAKE_SYSTEM_PROCESSOR=riscv64",
        "-DCMAKE_C_COMPILER=" + compiler,
        "-DSTATIC_BUILD=ON",
    )
    run(*prefix, "cmake", "--build", root + "/build/nano-riscv64", "--parallel", "4")
    run(*prefix, "file", root + "/build/nano-riscv64/drone-nano")


if __name__ == "__main__":
    main()
