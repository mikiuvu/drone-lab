import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from PySide6.QtWidgets import QApplication
from PySide6.QtTest import QTest
from PySide6.QtGui import QFont
from app.main import LabWindow

app=QApplication([]);app.setStyle('Fusion');app.setFont(QFont('Microsoft YaHei UI',10))
window=LabWindow();window.show();window.tabs.setCurrentIndex(6)
QTest.qWait(900);window.gazebo_panel.refresh();QTest.qWait(100)
window.grab().save(str(ROOT/'simulation/docs/gui-preview.png'))
window.close()
