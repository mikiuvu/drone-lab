"""在实际 Gazebo 图像上验证地图装甲入口及生产 PnP 检测器。"""

import argparse
import json
import os
import shlex
import signal
import subprocess
import sys
import threading
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))


def main():
    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node

    from simulation.launch_linux import environment
    from simulation.moving_armor import add_moving_armor

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, default=ROOT / 'maps/rmuc_2027.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--build', type=Path, default=Path('/home/devcontainers/drone-sim/build-companion'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    config = json.loads(args.map.read_text(encoding='utf-8'))
    tree = ET.parse(args.map.parent / config['world_file'])
    world = tree.getroot().find('world')
    add_moving_armor(world, config, args.output, (args.map.parent / config['terrain_triangles_file']).resolve())
    target, dive = config['team_mission']['target'], config['team_mission']['dive_attack']
    model = ET.SubElement(world, 'model', name='armor_probe')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = (
        f'{target["x"] - dive["entry_standoff_m"]} {target["y"]} '
        f'{target["z"] + dive["entry_height_m"]} 0 0 0')
    link = ET.SubElement(model, 'link', name='camera')
    sensor = ET.SubElement(link, 'sensor', name='camera', type='camera')
    ET.SubElement(sensor, 'topic').text = '/operator/armor_probe'
    ET.SubElement(sensor, 'update_rate').text = '10'
    camera = ET.SubElement(sensor, 'camera')
    ET.SubElement(camera, 'horizontal_fov').text = '1.5707963267948966'
    image = ET.SubElement(camera, 'image')
    for key, value in [('width', '640'), ('height', '360'), ('format', 'R8G8B8')]:
        ET.SubElement(image, key).text = value
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '.02'
    ET.SubElement(clip, 'far').text = '8'
    tree.write(args.output / 'world.sdf', encoding='utf-8', xml_declaration=True)
    env = environment()
    env['GZ_PARTITION'] = f'rmuc2027-armor-{os.getpid()}'
    os.environ.update(env)
    done, node = threading.Event(), Node()
    image_path = args.output / 'armor.ppm'

    def received(message):
        if done.is_set() or len(message.data) != message.width * message.height * 3:
            return
        image_path.write_bytes(f'P6\n{message.width} {message.height}\n255\n'.encode() + message.data)
        done.set()

    node.subscribe(Image, '/operator/armor_probe', received)
    with (args.output / 'gazebo.log').open('w', encoding='utf-8') as log:
        process = subprocess.Popen(['gz', 'sim', '-s', '-r', str(args.output / 'world.sdf')], env=env,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            if not done.wait(50):
                raise RuntimeError('Armor camera image missing')
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
    flags = shlex.split(subprocess.check_output(['pkg-config', '--cflags', '--libs', 'opencv4'], text=True))
    binary = args.output / 'detect_armor'
    subprocess.run(['c++', '-std=c++17', '-O2', '-I', str(ROOT / 'companion/include'), '-I', '/usr/include/eigen3',
                    str(Path(__file__).with_suffix('.cpp')), str(args.build / 'libcompanion_rm_auto_aim.a'),
                    str(args.build / 'libcompanion_core.a'), *flags, '-o', str(binary)], check=True)
    result = subprocess.run([str(binary), str(image_path), str(args.map)], text=True, capture_output=True, check=False)
    (args.output / 'detection.json').write_text(result.stdout, encoding='utf-8')
    print(result.stdout)
    if result.returncode:
        raise RuntimeError(f'Armor recognition failed: {result.stderr}')


if __name__ == '__main__':
    main()
