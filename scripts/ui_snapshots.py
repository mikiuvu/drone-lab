"""@file ui_snapshots.py
@brief 把界面各页签与飞行数据窗口渲染成 PNG，用于改动后的视觉回归检查。
@note 使用 WA_DontShowOnScreen 让窗口不受屏幕尺寸限制，因此可以在任意分辨率下出图；
截图只读取控件自身，不依赖桌面环境，可在 CI 或无显示器机器上运行。
用法示例：
    python scripts/ui_snapshots.py --out docs/_shots
    python scripts/ui_snapshots.py --dark --run-dir simulation/runtime/run_20261004_101934
"""

import argparse
import os
import sys
import time
from pathlib import Path
from unittest.mock import patch

from PySide6.QtCore import QCoreApplication, QEvent, Qt
from PySide6.QtGui import QFontDatabase
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from app import theme  # noqa: E402
from app.flight_dashboard import FlightDashboard  # noqa: E402
from app.main import LabWindow  # noqa: E402


def snap(window, path):
    """@brief 把窗口当前外观保存为 PNG。
    @param window 待截图的顶层窗口。
    @param path 输出文件路径。
    """
    # processEvents 不负责清理所有延迟删除对象；先释放换页/换主题时淘汰的控件。
    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    QApplication.processEvents()
    if not window.grab().save(str(path)):
        raise OSError(f"无法保存快照：{path}")


def capture_lab(app, out, dark, size):
    """@brief 渲染主窗口的全部页签，实时页在演示数据下出图。
    @param app QApplication 实例。
    @param out 输出目录。
    @param dark 为真时使用深色主题。
    @param size 窗口尺寸 (宽, 高)，单位像素。
    """
    window = LabWindow()
    window.apply_theme(dark)
    window.setAttribute(Qt.WidgetAttribute.WA_DontShowOnScreen, True)
    window.resize(*size)
    window.show()
    suffix = "-dark" if dark else ""
    # 先喂入演示数据，让热图、趋势和姿态都有内容，否则截到的全是空状态。
    window.start_demo()
    QTest.qWait(3000)
    snap(window, out / f"01-live{suffix}.png")
    for index in range(window.tabs.count()):
        window.tabs.setCurrentIndex(index)
        QTest.qWait(100)
        name = window.tabs.tabText(index).replace(" ", "_")
        snap(window, out / f"{index:02d}-{name}{suffix}.png")
    window.close()


def capture_flight(app, out, dark, size, folder):
    """@brief 渲染飞行数据窗口的全部机与单机两种视图。
    @param app QApplication 实例。
    @param out 输出目录。
    @param dark 为真时使用深色主题。
    @param size 窗口尺寸 (宽, 高)，单位像素。
    @param folder 含 companion 日志的仿真运行目录。
    """
    suffix = "-dark" if dark else ""
    window = FlightDashboard(folder)
    window.set_theme(dark)
    window.setAttribute(Qt.WidgetAttribute.WA_DontShowOnScreen, True)
    window.resize(*size)
    window.show()
    try:
        # 等后台读完已有日志再绘制；超时应报错，不能把空图当成成功快照。
        deadline = time.monotonic() + 30
        while not window.snapshot or window.snapshot["backlog"]:
            if time.monotonic() >= deadline:
                raise TimeoutError(f"飞行日志未加载完成：{folder}")
            QTest.qWait(100)
        window.canvas.draw()
        snap(window, out / f"10-flight-all{suffix}.png")
        if window.vehicle.count() > 1:
            window.vehicle.setCurrentIndex(1)
            window.canvas.draw()
            snap(window, out / f"11-flight-single{suffix}.png")
    finally:
        window.close()


def resolve_run_dir(explicit):
    """@brief 决定飞行数据窗口要读取的运行目录。
    @param explicit 命令行给出的目录，可为空。
    @return 运行目录 Path；找不到可用目录时返回 None。
    """
    if explicit:
        return Path(explicit)
    from simulation.localization_metrics import resolve_run_dir as resolve

    try:
        return resolve(ROOT)
    except (OSError, ValueError):
        return None


def main():
    """@brief 解析参数并依次渲染主窗口与飞行数据窗口。"""
    parser = argparse.ArgumentParser(description="渲染界面快照")
    parser.add_argument("--out", type=Path, default=ROOT / "docs" / "_shots")
    parser.add_argument("--dark", action="store_true", help="使用深色主题")
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--run-dir", type=Path, help="飞行数据窗口读取的运行目录")
    parser.add_argument("--skip-flight", action="store_true", help="只渲染主窗口")
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    app = QApplication(sys.argv[:1])
    # Windows 的 offscreen 插件不枚举系统字体，显式注册已安装字体以显示中文。
    if sys.platform == "win32" and app.platformName() == "offscreen":
        fonts = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"
        for name in ("msyh.ttc", "msyhbd.ttc", "segoeui.ttf"):
            path = fonts / name
            if path.is_file():
                QFontDatabase.addApplicationFont(str(path))
    theme.apply_theme(app, args.dark)
    size = (args.width, args.height)
    # 快照里的演示模式和主题只用于渲染，保留用户保存的连接与界面配置。
    with patch.object(LabWindow, "save_settings", return_value=None):
        capture_lab(app, args.out, args.dark, size)
    if not args.skip_flight:
        folder = resolve_run_dir(args.run_dir)
        if folder is None:
            print("跳过飞行数据窗口：没有可用的仿真运行目录，请用 --run-dir 指定。")
        else:
            capture_flight(app, args.out, args.dark, size, folder)
    print("快照已保存到", args.out)


if __name__ == "__main__":
    main()
