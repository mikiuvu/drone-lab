"""绘图负载上限、全量统计和单槽快照交换回归。"""

import numpy as np

from app.flight_plot_data import LatestSnapshot, aligned_errors, envelope, prepare_plot, EVENT_LIMIT
from simulation.flight_data import aligned_samples, export_csv


def test_vector_errors_match_original_alignment_with_gaps():
    times = np.arange(500) * 0.05
    times = np.delete(times, np.arange(100, 150))
    truth = np.column_stack((times, np.sin(times), np.cos(times), times * 0, times * 0))
    estimate_times = np.arange(700) * 0.04 - 0.11
    estimates = np.column_stack((estimate_times, np.sin(estimate_times) + 0.01,
                                 np.cos(estimate_times) - 0.03))
    expected = np.array([row[2] for row in aligned_samples(estimates, truth)])
    np.testing.assert_allclose(aligned_errors(estimates, truth), expected, equal_nan=True, atol=1e-12)


def test_envelope_bounds_points_keeps_peaks_and_breaks():
    x = np.arange(50000)
    values = np.zeros(len(x))
    values[123] = 100
    values[999] = -20
    values[9000:9100] = np.nan
    tx, y = envelope(x, values, 400)
    assert len(tx) <= 400 and tx[0] == 0 and tx[-1] == x[-1]
    assert 100 in y and -20 in y and np.isnan(y).any()
    np.testing.assert_array_equal(y, values[tx])


def make_snapshot(count=20000):
    times = np.arange(count) * 0.05
    estimates = np.zeros((count, 11))
    estimates[:, 0] = times
    estimates[:, 1] = np.sin(times)
    estimates[:, 3] = 1
    estimates[:, 4] = 0.5
    estimates[:, 8:] = 1
    truth = np.column_stack((times, estimates[:, 1] + 0.1, times * 0, times * 0 + 1, times * 0))
    return dict(ids=[0], estimates={0: estimates.tolist()}, truths={0: truth.tolist()},
                phases=[(i * 0.2, "CONTACT", 0) for i in range(2000)],
                impacts=[(i, 0) for i in range(1000)])


def test_display_preparation_preserves_full_stats_and_csv(tmp_path):
    snapshot = make_snapshot()
    prepared = prepare_plot(snapshot)
    assert len(snapshot["estimates"][0]) == 20000
    assert len(prepared["full"][0][0]) == 20000
    assert len(prepared["curves"][0]["height"][0]) <= 1600
    np.testing.assert_allclose(prepared["stats"][0], [0.1, 0.1, 0.1], atol=1e-12)
    assert len(prepared["phases"]) <= EVENT_LIMIT
    assert len(prepared["impacts"]) <= EVENT_LIMIT
    target = tmp_path / "all.csv"
    export_csv(snapshot, target)
    assert len(target.read_text(encoding="utf-8-sig").splitlines()) == 20001


def test_latest_snapshot_overwrites_pending_notifications():
    slot = LatestSnapshot()
    for revision in range(100):
        slot.publish({"revision": revision})
    assert slot.take() == {"revision": 99}
    assert slot.take() is None


def test_prepare_stops_before_work_when_cancelled():
    assert prepare_plot(make_snapshot(1), cancelled=lambda: True) is None
