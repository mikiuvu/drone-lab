"""@file build_nano.py
@brief 在 Ubuntu-22.04 WSL 中交叉编译 Nano 的 RISC-V 静态采集器。
@note 读取 config/toolchain.json，生成或更新 build/nano-riscv64；不上传到设备。
"""

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
    """@brief 启动参数列表指定的进程，输出 UTF-8 日志并检查退出码。
    @param args 进程及参数，不经过 Shell 拼接。
    @note 非零退出抛 CalledProcessError。
    """
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
    """@brief 用 wslpath 转换项目路径，读取编译器路径后配置、构建并检查产物类型。
    """
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
