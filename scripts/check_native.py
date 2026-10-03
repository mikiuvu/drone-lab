"""@file check_native.py
@brief 在 WSL 编译并运行 C 编码契约，再用 Python 验证 JSON 字段、字节翻转与无效目标过滤。
@note 导入即执行检查，更新 build/native/nano-contract；输入来自测试替身，不是传感器。
"""
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from app.data import validate_record, valid_distances

wsl = ["wsl", "-d", "Ubuntu-22.04", "--exec"]
root = (
    subprocess.run(wsl + ["wslpath", "-a", str(ROOT)], check=True, capture_output=True)
    .stdout.decode()
    .strip()
)
subprocess.run(wsl + ["mkdir", "-p", root + "/build/native"], check=True)
subprocess.run(
    wsl
    + [
        "gcc",
        "-std=c11",
        "-O2",
        "-I" + root + "/nano",
        "-I" + root + "/vendor/stm32-vl53l5cx/modules",
        "-isystem",
        root + "/vendor/mavlink",
        root + "/tests/nano_contract.c",
        root + "/nano/platform.c",
        root + "/vendor/stm32-vl53l5cx/modules/vl53l5cx_api.c",
        "-lm",
        "-o",
        root + "/build/native/nano-contract",
    ],
    check=True,
)
raw = subprocess.run(
    wsl + [root + "/build/native/nano-contract"], check=True, capture_output=True
).stdout
item = validate_record(json.loads(raw))
assert (
    item["sensor"] == 1
    and item["source_time_us"] == 1234567
    and len(valid_distances(item)) == 63
)
assert len(item["signal_kcps_spad"]) == 64 and len(item["sigma_mm"]) == 64
print("PASS: real C JSON encoder -> Python validator; endian swap; invalid target mask")
