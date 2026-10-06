"""验证实验台按钮到 WSL Gazebo GUI 的完整启动链路。"""

import argparse
import json
import sys
import time
from pathlib import Path

from PySide6.QtTest import QTest
from PySide6.QtGui import QFontDatabase
from PySide6.QtWidgets import QApplication, QPushButton

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from app.simulation_panel import GazeboPanel
from app import theme


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--action', choices=('start', 'gui'), default='start')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    app = QApplication.instance() or QApplication([])
    for name in ('msyh.ttc', 'msyhbd.ttc', 'msyhl.ttc'):
        font = Path('C:/Windows/Fonts') / name
        if font.exists():
            QFontDatabase.addApplicationFont(str(font))
    theme.apply_theme(app, False)
    panel = GazeboPanel(ROOT)
    panel.resize(1450, 1000)
    panel.fly.setChecked(False)
    panel.mission_choice.setCurrentIndex(panel.mission_choice.findData('hover'))
    panel.show()
    captions = {'start': '▶ 启动多机环境', 'gui': '打开 Gazebo'}
    buttons = [button for button in panel.findChildren(QPushButton)
               if button.text() == captions[args.action]]
    if len(buttons) != 1:
        raise RuntimeError('无法唯一定位 Gazebo 操作按钮')
    buttons[0].click()
    deadline = time.monotonic() + 180
    while panel.process is not None and time.monotonic() < deadline:
        QTest.qWait(50)
    if panel.process is not None:
        raise TimeoutError('实验台 Gazebo 命令未结束')
    if panel.command_error:
        raise RuntimeError(panel.command_error + chr(10) + panel.log.toPlainText())
    # 等待后台快照把地图和相机显示出来，验证真实 UI 的输出内容。
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline:
        QTest.qWait(100)
        if panel.snapshot and all(camera.pixmap() is not None and not camera.pixmap().isNull()
                                  for camera in panel.cameras):
            break
    args.output.parent.mkdir(parents=True, exist_ok=True)
    panel.grab().save(str(args.output.with_suffix('.png')))
    report = {'action': args.action, 'status': panel.status.text(),
              'camera_count': len(panel.cameras),
              'camera_images': sum(camera.pixmap() is not None and not camera.pixmap().isNull()
                                   for camera in panel.cameras),
              'run_dir': str(panel.current_folder), 'error': panel.command_error}
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    panel.close()
    deadline = time.monotonic() + 10
    while panel.refresh_worker is not None and time.monotonic() < deadline:
        QTest.qWait(50)
    print(json.dumps(report, ensure_ascii=False))
    if report['camera_images'] != report['camera_count']:
        raise RuntimeError('部分机载相机未显示')


if __name__ == '__main__':
    main()
