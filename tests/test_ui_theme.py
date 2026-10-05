"""@file test_ui_theme.py
@brief 界面主题、自绘控件与指标格式化的回归测试。
覆盖两类内容：视觉令牌与配色映射这类纯函数，以及曾经出过缺陷的绘制与显示逻辑
（色标控件误成顶层窗口、失联机号列表重复累积、平滑函数把缺失值抹平）。
"""

import math
import unittest

from app import theme
from app.data import demonstration

try:
    from PySide6.QtWidgets import QApplication

    from app.flight_dashboard import PHASE_COLORS, PHASE_NAMES, smooth
    from app.simulation_panel import GazeboPanel
    from app.widgets import AttitudeView, Heatmap, SensorCard, Trend
except ModuleNotFoundError:
    QApplication = None
    smooth = None
    GazeboPanel = None


class ThemeTokenTests(unittest.TestCase):
    def test_light_and_dark_define_the_same_keys(self):
        self.assertEqual(set(theme.LIGHT), set(theme.DARK))

    def test_every_token_is_a_hex_color(self):
        for name, value in theme.tokens().items():
            if name in ("radius", "space", "dark"):
                continue
            self.assertRegex(value, r"^#[0-9a-f]{6}$", name)

    def test_categorical_palettes_have_equal_length_and_unique_entries(self):
        light = theme.categorical(False)
        dark = theme.categorical(True)
        self.assertEqual(len(light), len(dark))
        self.assertEqual(len(set(light)), len(light))
        self.assertEqual(len(set(dark)), len(dark))

    def test_lerp_color_hits_endpoints_and_clamps(self):
        stops = theme.DISTANCE_STOPS
        self.assertEqual(
            theme.hex_of(theme.lerp_color(stops, 0.0)), stops[0].lower()
        )
        self.assertEqual(
            theme.hex_of(theme.lerp_color(stops, 1.0)), stops[-1].lower()
        )
        # 超出 0–1 的位置必须截断到端点，而不是插值出非法颜色。
        self.assertEqual(
            theme.hex_of(theme.lerp_color(stops, -3.0)), stops[0].lower()
        )
        self.assertEqual(
            theme.hex_of(theme.lerp_color(stops, 7.0)), stops[-1].lower()
        )

    def test_lerp_color_stays_in_gamut_across_the_ramp(self):
        for step in range(101):
            rgb = theme.lerp_color(theme.DISTANCE_STOPS, step / 100)
            self.assertTrue(all(0 <= channel <= 255 for channel in rgb), step)

    def test_readable_on_flips_at_the_luminance_threshold(self):
        self.assertEqual(theme.readable_on((255, 255, 255)), "#10161f")
        self.assertEqual(theme.readable_on((0, 0, 0)), "#ffffff")
        # 距离色标的两端必须给出相反的文字色，否则近端或远端会读不清。
        bright = theme.lerp_color(theme.DISTANCE_STOPS, 0.0)
        dark = theme.lerp_color(theme.DISTANCE_STOPS, 1.0)
        self.assertNotEqual(theme.readable_on(bright), theme.readable_on(dark))

    def test_stylesheet_escapes_braces_and_uses_theme_colors(self):
        light = theme.stylesheet(False)
        dark = theme.stylesheet(True)
        for name, sheet in (("light", light), ("dark", dark)):
            self.assertNotIn("{t[", sheet, name)
            self.assertNotIn("{{", sheet, name)
            self.assertIn("QGroupBox", sheet, name)
        self.assertIn(theme.LIGHT["accent"], light)
        self.assertIn(theme.DARK["accent"], dark)
        self.assertNotEqual(light, dark)

    def test_phase_color_falls_back_to_neutral_for_unknown_names(self):
        self.assertEqual(
            theme.phase_color("CONTACT"), theme.PHASE_COLORS["CONTACT"]
        )
        self.assertEqual(
            theme.phase_color("CONTACT", True), theme.PHASE_COLORS_DARK["CONTACT"]
        )
        # 未收录的阶段必须退化为中性灰，而不是抛错或借用别的阶段颜色。
        self.assertEqual(theme.phase_color("NOT_A_PHASE"), theme.PHASE_COLORS["WAIT_READY"])
        self.assertEqual(
            theme.phase_color("NOT_A_PHASE", True), theme.PHASE_COLORS_DARK["WAIT_READY"]
        )
        self.assertEqual(theme.phase_name("COMPLETE"), "完成")
        self.assertEqual(theme.phase_name(""), "等待任务阶段")
        self.assertIn("NOT_A_PHASE", theme.phase_name("NOT_A_PHASE"))

    def test_every_colored_phase_has_a_display_name(self):
        # 阶段条按名称配色，配色表必须是名称表的子集，否则会出现有颜色无中文名的阶段。
        for table in (theme.PHASE_COLORS, theme.PHASE_COLORS_DARK):
            self.assertTrue(set(table) <= set(theme.PHASE_NAMES))

    def test_phase_tables_cover_the_same_names_in_both_themes(self):
        self.assertEqual(set(theme.PHASE_COLORS), set(theme.PHASE_COLORS_DARK))

    def test_matplotlib_rc_is_applied_inside_the_context(self):
        # rc_context.__getitem__ 返回进入上下文之前的旧值，必须直接读 rcParams 才能验证生效结果。
        from matplotlib import rc_context, rcParams

        for dark in (False, True):
            with rc_context(theme.matplotlib_rc(dark)):
                self.assertTrue(rcParams["axes.grid"])
                self.assertFalse(rcParams["axes.spines.top"])
                self.assertEqual(
                    rcParams["figure.facecolor"], theme.tokens(dark)["surface"]
                )

    def test_mpl_context_restores_previous_settings(self):
        from matplotlib import rcParams

        before = rcParams["figure.facecolor"]
        with theme.mpl_context(True):
            self.assertEqual(rcParams["figure.facecolor"], theme.DARK["surface"])
        self.assertEqual(rcParams["figure.facecolor"], before)


@unittest.skipUnless(QApplication is not None, "PySide6 is required for UI checks")
class FlightThemeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_flight_dashboard_uses_theme_background(self):
        from app.flight_dashboard import FlightDashboard

        window = FlightDashboard()
        window.set_theme(True)
        expected = tuple(
            int(theme.DARK["surface"][index : index + 2], 16) / 255
            for index in (1, 3, 5)
        )
        self.assertEqual(window.figure.get_facecolor()[:3], expected)
        window.close()


@unittest.skipUnless(QApplication is not None, "PySide6 is required for UI checks")
class WidgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_colour_scale_is_a_child_of_the_heatmap(self):
        # 色标控件如果没挂到父控件上会变成独立顶层窗口，既不随热图移动也不显示。
        view = Heatmap()
        self.assertIs(view.scale.parent(), view)

    def test_zone_at_matches_the_flipped_paint_mapping(self):
        view = Heatmap()
        view.resize(400, 430)
        rect = view.grid_rect()
        cell = rect.width() / 8
        for flip in (True, False):
            view.flip = flip
            for row in range(8):
                for col in range(8):
                    x = rect.left() + (col + 0.5) * cell
                    y = rect.top() + (row + 0.5) * cell
                    expected = row * 8 + col
                    if flip:
                        expected = 63 - expected
                    self.assertEqual(view.zone_at(x, y), expected, (flip, row, col))

    def test_zone_at_returns_none_outside_the_grid(self):
        view = Heatmap()
        view.resize(400, 430)
        self.assertIsNone(view.zone_at(-5, -5))
        self.assertIsNone(view.zone_at(view.width() + 5, view.height() + 5))

    def test_value_range_follows_frame_extremes_only_in_auto_mode(self):
        view = Heatmap()
        view.frame = demonstration(0)[0]
        low, high = view.value_range()
        self.assertLess(low, high)
        self.assertGreater(low, 1000)
        self.assertLess(high, 2500)
        view.auto_range = False
        view.maximum = 4000
        self.assertEqual(view.value_range(), (0.0, 4000.0))

    def test_value_range_survives_a_frame_without_valid_zones(self):
        view = Heatmap()
        frame = demonstration(0)[0]
        frame["status"] = [0] * 64
        view.frame = frame
        self.assertEqual(view.value_range(), (0.0, float(view.maximum)))

    def test_value_range_expands_a_single_valued_frame(self):
        # 全部测到同一距离时量程宽度为零，颜色映射会除零，必须人为展开。
        view = Heatmap()
        frame = demonstration(0)[0]
        frame["distance_mm"] = [1500] * 64
        frame["status"] = [5] * 64
        view.frame = frame
        low, high = view.value_range()
        self.assertLess(low, high)
        self.assertLessEqual(low, 1500)
        self.assertGreaterEqual(high, 1500)

    def test_widgets_paint_without_data(self):
        for widget in (Heatmap(), SensorCard(0), AttitudeView(), Trend()):
            with self.subTest(widget=type(widget).__name__):
                widget.resize(360, 320)
                self.assertFalse(widget.grab().isNull())
                widget.deleteLater()

    def test_theme_switch_repaints_custom_widgets(self):
        card = SensorCard(0)
        card.set_theme(True)
        self.assertTrue(card.map.dark)
        card.set_theme(False)
        self.assertFalse(card.map.dark)
        card.deleteLater()

    def test_sensor_card_tracks_sequence_gaps(self):
        card = SensorCard(0)
        first = demonstration(0)[0]
        card.receive(dict(first, sequence=1))
        card.receive(dict(first, sequence=4))
        self.assertEqual(card.missed, 2)
        self.assertEqual(card.total, 2)
        card.deleteLater()

    def test_trend_reset_clears_both_series(self):
        view = Trend()
        view.values[0].append(1500)
        view.values[1].append(1600)
        view.reset()
        self.assertEqual(len(view.values[0]), 0)
        self.assertEqual(len(view.values[1]), 0)


@unittest.skipUnless(QApplication is not None, "PySide6 is required for UI checks")
class FleetDisplayTests(unittest.TestCase):
    """失联机号曾经在历史记录里重复累积，把状态栏刷成几千字符，这里锁住去重行为。"""

    def test_id_list_dedupes_and_sorts(self):
        self.assertEqual(GazeboPanel._id_list([3, 1, 3, 0, 1, 0]), "0、1、3")

    def test_id_list_ignores_non_integer_and_negative_entries(self):
        self.assertEqual(GazeboPanel._id_list([2, "3", -1, None, 2.5, 0]), "0、2")

    def test_id_list_collapses_long_lists(self):
        text = GazeboPanel._id_list(list(range(40)))
        self.assertLess(len(text), 60)
        self.assertIn("等 40 架", text)

    def test_id_list_handles_missing_and_empty_values(self):
        self.assertEqual(GazeboPanel._id_list(None), "无")
        self.assertEqual(GazeboPanel._id_list([]), "无")

    def test_phase_text_stays_short_despite_duplicate_skipped_entries(self):
        # 模拟旧固件反复失联/恢复产生的重复记录：界面必须去重后再显示。
        team = {
            "phase": "TRANSIT",
            "flight_elapsed_s": 12.5,
            "active_vehicle": 1,
            "completed": [0],
            "skipped": [0, 1, 2, 3] * 400,
        }
        text = GazeboPanel._phase_text(team)
        self.assertIn("跳过机号：0、1、2、3", text)
        self.assertLess(len(text), 120)

    def test_phase_text_handles_a_missing_team_dictionary(self):
        text = GazeboPanel._phase_text({})
        self.assertIn("等待任务阶段", text)

    def test_phase_text_reports_counters(self):
        text = GazeboPanel._phase_text(
            {"phase": "CONTACT", "telemetry_errors": 3, "send_errors": 1}
        )
        self.assertIn("状态写入重试 3 次", text)
        self.assertIn("指令发送失败 1 次", text)


@unittest.skipUnless(smooth is not None, "PySide6 is required for smoothing checks")
class SmoothingTests(unittest.TestCase):
    def test_window_below_two_returns_the_input_unchanged(self):
        values = [1.0, 2.0, 3.0]
        self.assertIs(smooth(values, 1), values)

    def test_leading_gap_is_not_smeared_into_later_samples(self):
        import numpy as np

        # 前三个样本全部缺失：前三个输出必须仍是缺失，不能被后面的值补平。
        result = smooth(np.array([np.nan, np.nan, np.nan, 1.0]), 3)
        self.assertEqual(len(result), 4)
        self.assertTrue(all(math.isnan(v) for v in result[:3]))
        self.assertAlmostEqual(result[3], 1.0)

    def test_nan_is_excluded_from_the_mean(self):
        import numpy as np

        # 窗口 [1, NaN, 3] 的有效均值应为 2，而不是把 NaN 当零算成 1.33。
        result = smooth(np.array([1.0, np.nan, 3.0]), 3)
        self.assertAlmostEqual(result[-1], 2.0)

    def test_output_length_matches_input(self):
        import numpy as np

        values = np.arange(20.0)
        self.assertEqual(len(smooth(values, 5)), len(values))

    def test_smoothing_reduces_the_peak_of_a_spike(self):
        import numpy as np

        values = np.zeros(20)
        values[10] = 10.0
        self.assertLess(smooth(values, 5).max(), values.max())

    def test_smoothing_a_short_series_does_not_raise(self):
        import numpy as np

        self.assertEqual(len(smooth(np.array([1.0]), 10)), 1)
        self.assertEqual(len(smooth(np.array([]), 4)), 0)


class PhaseAliasTests(unittest.TestCase):
    def test_dashboard_phase_aliases_point_at_the_shared_tables(self):
        self.assertIs(PHASE_NAMES, theme.PHASE_NAMES)
        self.assertIs(PHASE_COLORS, theme.PHASE_COLORS)


if __name__ == "__main__":
    unittest.main()
