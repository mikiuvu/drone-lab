"""用 180 g 球形探针自由落体检查 CAD 装甲正面的实际接触力。"""

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent))


def main():
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.transport13 import Node

    from simulation.launch_linux import environment

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, default=ROOT / 'maps/rmuc_2027.json')
    parser.add_argument('--world', type=Path, required=True, help='装甲视觉检查生成的 world.sdf')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    config = json.loads(args.map.read_text(encoding='utf-8'))
    tree = ET.parse(args.world)
    world = tree.getroot().find('world')
    if world.find('plugin[@name="gz::sim::systems::Contact"]') is None:
        ET.SubElement(world, 'plugin', filename='gz-sim-contact-system', name='gz::sim::systems::Contact')
    target = config['team_mission']['target']
    radius, mass = config['team_mission']['contact_radius_m'], .18
    model = ET.SubElement(world, 'model', name='micro_0')
    ET.SubElement(model, 'pose').text = f'{target["x"] - .05} {target["y"]} {target["z"] + .8} 0 0 0'
    link = ET.SubElement(model, 'link', name='base_link')
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'mass').text = str(mass)
    inertia = ET.SubElement(inertial, 'inertia')
    for key in ('ixx', 'iyy', 'izz'):
        ET.SubElement(inertia, key).text = str(.4 * mass * radius * radius)
    for key in ('ixy', 'ixz', 'iyz'):
        ET.SubElement(inertia, key).text = '0'
    for kind in ('collision', 'visual'):
        item = ET.SubElement(link, kind, name='body_collision')
        sphere = ET.SubElement(ET.SubElement(item, 'geometry'), 'sphere')
        ET.SubElement(sphere, 'radius').text = str(radius)
    tree.write(args.output / 'world.sdf', encoding='utf-8', xml_declaration=True)
    env = environment()
    env['GZ_PARTITION'] = f'rmuc2027-pressure-{os.getpid()}'
    os.environ.update(env)
    node, done, forces = Node(), threading.Event(), []

    def contact(message):
        for hit in message.contact:
            names = (hit.collision1.name, hit.collision2.name)
            if not any('armor_plate' in name for name in names) or not any(name.startswith('micro_0::') for name in names):
                continue
            total = [sum(getattr(wrench.body_1_wrench.force, axis) for wrench in hit.wrench) for axis in ('x', 'y', 'z')]
            force = math.sqrt(sum(value * value for value in total))
            if math.isfinite(force) and force > 0:
                forces.append(force)
                done.set()

    topic = f'/world/{world.get("name")}/model/enemy_base_moving_armor/link/armor/sensor/armor_contact/contact'
    node.subscribe(Contacts, topic, contact)
    with (args.output / 'gazebo.log').open('w', encoding='utf-8') as log:
        process = subprocess.Popen(['gz', 'sim', '-s', '-r', str(args.output / 'world.sdf')], env=env,
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            done.wait(40)
            time.sleep(.5)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
    report = {'peak_force_n': max(forces, default=0), 'positive_samples': len(forces),
              'probe_mass_kg': mass, 'probe_radius_m': radius, 'drop_height_above_target_m': .8}
    (args.output / 'pressure.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report))
    if report['peak_force_n'] < config['team_mission']['armor_pressure']['min_force_n']:
        raise RuntimeError('CAD armor pressure threshold not reached')


if __name__ == '__main__':
    main()
