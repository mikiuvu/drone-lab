import os
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PySide6.QtWidgets import QApplication
from app.research_panel import ResearchPanel, result_html, MODELS
import pytest


def test_integration_gui_result_saved(tmp_path):
    app = QApplication.instance() or QApplication([])
    panel = ResearchPanel(tmp_path)
    panel.choice.setCurrentText("时间错配与接口门控")
    panel.start()
    for _ in range(200):
        time.sleep(0.03)
        app.processEvents()
        if panel.worker is None:
            break
    assert panel.worker is None
    assert "23.6" in panel.view.toPlainText()
    files = list((tmp_path / "research" / "results").glob("integration*.json"))
    assert len(files) == 1
    assert panel.run_button.isEnabled()
    panel.close()


def test_report_text_is_escaped():
    output = result_html(
        {
            "summary": "<img src=x>",
            "metrics": {"<a>": "<script>"},
            "assumptions": ["<b>"],
        }
    )
    assert "<script>" not in output and "&lt;script&gt;" in output


@pytest.mark.parametrize("choice", list(MODELS))
def test_each_gui_model_runs_without_hardware(tmp_path, choice):
    app = QApplication.instance() or QApplication([])
    panel = ResearchPanel(tmp_path)
    panel.choice.setCurrentText(choice)
    panel.start()
    deadline = time.monotonic() + 15
    while panel.worker is not None and time.monotonic() < deadline:
        time.sleep(0.03)
        app.processEvents()
    assert panel.worker is None
    assert "已保存" in panel.status.text(), panel.status.text()
    assert len(list((tmp_path / "research" / "results").glob("*.json"))) == 1
    panel.close()
