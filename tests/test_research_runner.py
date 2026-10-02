import hashlib
import json
import pytest
from research.runner import execute


def test_runner_preserves_parameters_and_source(tmp_path):
    import research.integration as module

    path = tmp_path / "result.json"
    result = execute("integration", {"delay_ms": 10}, path, plot=False)
    assert result["result"]["metrics"]["translation_age_error_m"] == 0.01
    assert json.loads(path.read_text(encoding="utf-8"))["input_config"] == {
        "delay_ms": 10
    }
    from pathlib import Path

    assert (
        result["source_sha256"]
        == hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()
    )


def test_plot_failure_keeps_numerical_evidence(tmp_path, monkeypatch):
    import research.plots as plots

    def broken(*args):
        raise RuntimeError("injected image backend failure")

    monkeypatch.setattr(plots, "save_plot", broken)
    path = tmp_path / "result.json"
    result = execute("integration", {}, path)
    assert result["plot_error"] == "injected image backend failure"
    assert (
        json.loads(path.read_text(encoding="utf-8"))["result"]["metrics"][
            "translation_age_error_m"
        ]
        == 0.05
    )


def test_unknown_model_does_not_execute(tmp_path):
    with pytest.raises(ValueError):
        execute("../../app.main", {}, tmp_path / "bad.json", False)
