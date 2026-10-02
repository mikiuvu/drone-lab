import sys
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
window = LabWindow()
window.show()
window.start_demo()
QTest.qWait(450)
window.grab().save(str(ROOT / "docs" / "preview.png"))
for index in (1, 2, 3, 4):
    window.tabs.setCurrentIndex(index)
    QTest.qWait(80)
    window.grab().save(str(ROOT / "docs" / f"preview-{index}.png"))
window.stop_sources()
window.close()
