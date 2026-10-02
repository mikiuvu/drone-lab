"""Double-click launcher: keep a persistent error log without a console window."""

import ctypes
import os
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parent
os.chdir(ROOT)
sys.path.insert(0, str(ROOT))
(ROOT / "logs").mkdir(exist_ok=True)
log = (ROOT / "logs" / "desktop-errors.log").open("a", encoding="utf-8", buffering=1)
sys.stderr = log


def report(error_type, value, trace):
    traceback.print_exception(error_type, value, trace, file=log)
    ctypes.windll.user32.MessageBoxW(
        None,
        "程序遇到错误，详细信息已保存到 logs/desktop-errors.log。",
        "Drone Lab",
        0x10,
    )


sys.excepthook = report
from app.main import main

main()
