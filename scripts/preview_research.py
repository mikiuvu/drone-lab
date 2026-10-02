import sys
import time
import faulthandler
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from PySide6.QtWidgets import QApplication
from PySide6.QtTest import QTest
from PySide6.QtGui import QFont
from app.main import LabWindow

app = QApplication([])
app.setStyle("Fusion")
app.setFont(QFont("Microsoft YaHei UI", 10))
trace = (ROOT / "logs" / "research-preview-trace.txt").open("w", encoding="utf-8")
faulthandler.dump_traceback_later(12, file=trace)
window = LabWindow()
window.show()
window.tabs.setCurrentIndex(5)
panel = window.research_panel
for choice, name in [
    ("任务调度与图像排队", "compute"),
    ("双 ToF 几何与可观测性", "geometry"),
    ("时间错配与接口门控", "integration"),
    ("I²C 与串口带宽预算", "bandwidth"),
    ("失效与恢复状态机", "control"),
]:
    panel.choice.setCurrentText(choice)
    if name == "compute":
        panel.spins["npu_ms"].setValue(60)
        panel.spins["cpu_pre_ms"].setValue(6)
        panel.spins["cpu_post_ms"].setValue(4)
    panel.start()
    deadline = time.monotonic() + 20
    while panel.worker is not None and time.monotonic() < deadline:
        time.sleep(0.03)
        app.processEvents()
    if panel.worker is not None:
        raise RuntimeError("Preview timed out")
    if "未完成" in panel.status.text():
        raise RuntimeError(panel.status.text())
    QTest.qWait(100)
    window.grab().save(str(ROOT / "docs" / "research" / ("ui-" + name + ".png")))
panel.report()
QTest.qWait(120)
window.grab().save(str(ROOT / "docs" / "research" / "ui-overview.png"))
window.close()
faulthandler.cancel_dump_traceback_later()
trace.close()
