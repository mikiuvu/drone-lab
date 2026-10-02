import hashlib
import pytest
from scripts.build_research_report import verify_sources, numeric


def test_report_refuses_stale_source(tmp_path):
    source = tmp_path / "model.py"
    source.write_text("version=1", encoding="utf-8")
    bundle = {
        "runs": {
            "model": {
                "source": "model.py",
                "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            }
        }
    }
    verify_sources(bundle, tmp_path)
    source.write_text("version=2", encoding="utf-8")
    with pytest.raises(ValueError, match="source changed"):
        verify_sources(bundle, tmp_path)


def test_no_samples_not_zero_latency():
    assert numeric(None) == "无完成样本"
    assert numeric(0) == "0.0"
