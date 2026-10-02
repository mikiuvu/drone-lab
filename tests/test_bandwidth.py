import pytest
from research.bandwidth import i2c_read_bits, run_bandwidth


def test_single_and_chunked_read_bytes():
    assert i2c_read_bits(1, 256) == 45
    assert i2c_read_bits(256, 256) == 9 * 260
    assert i2c_read_bits(257, 256) == 9 * 265
    with pytest.raises(ValueError):
        i2c_read_bits(0, 256)


def test_full_duplex_not_summed():
    result = run_bandwidth()["metrics"]
    assert result["i2c_read_bits_per_frame"] == 13716
    assert result["shared_bus_lower_bound_utilization"] > 1
    assert result["fc_to_nano_rx_utilization"] < 1
    assert result["nano_to_fc_tx_utilization"] < 1


def test_1mhz_budget_changes_not_payload():
    a = run_bandwidth()["metrics"]
    b = run_bandwidth({"i2c_khz": 1000})["metrics"]
    assert b["i2c_read_bits_per_frame"] == a["i2c_read_bits_per_frame"]
    assert b["i2c_read_minimum_ms"] == pytest.approx(a["i2c_read_minimum_ms"] * 0.4)
