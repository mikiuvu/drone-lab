"""Verify live Gazebo rendered sensors and the plugin UDP contract.

This invokes the real engine; expected plane distances are only used by the
evaluator, never to create an observation or a camera pixel.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET


def add(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


def box(world, name, pose, size, color):
    model = add(world, "model", name=name)
    add(model, "static", "true")
    add(model, "pose", pose)
    link = add(model, "link", name="link")
    for kind in ("collision", "visual"):
        shape = add(link, kind, name=kind)
        add(add(add(shape, "geometry"), "box"), "size", size)
        if kind == "visual":
            material = add(shape, "material")
            add(material, "ambient", color)
            add(material, "diffuse", color)
    return model


def make_world(path, models, port_base):
    root = ET.Element("sdf", {"version": "1.9"})
    world = add(root, "world", name="sensor_smoke")
    physics = add(world, "physics", name="physics", type="ignored")
    add(physics, "max_step_size", "0.004")
    add(physics, "real_time_factor", "1")
    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
        ("gz-sim-sensors-system", "gz::sim::systems::Sensors"),
    ):
        plugin = add(world, "plugin", filename=filename, name=name)
        if "Sensors" in name:
            add(plugin, "render_engine", "ogre2")
    plugin = add(world, "plugin", filename="libpeer_height_system.so",
                 name="drone_sim::PeerHeightSystem")
    add(plugin, "port_base", port_base)
    add(plugin, "range_noise_std_m", "0")
    add(plugin, "bearing_noise_std_rad", "0")
    add(plugin, "drop_probability", "0")
    add(plugin, "publish_hz", "50")
    scene = add(world, "scene")
    add(scene, "ambient", "0.8 0.8 0.8 1")
    add(scene, "background", "0.2 0.2 0.2 1")
    light = add(world, "light", name="sun", type="directional")
    add(light, "pose", "0 0 10 0 0 0")
    add(light, "diffuse", "0.8 0.8 0.8 1")
    add(light, "direction", "0 0 -1")
    for i in range(4):
        origin = i * 8
        include = add(world, "include")
        add(include, "uri", (models / f"micro_{i}" / "model.sdf").as_uri())
        add(include, "name", f"micro_{i}")
        add(include, "pose", f"{origin} 0 1 0 0 0")
        # Interior planes are exactly at local x/y = +/-2 and z=0.
        box(world, f"floor_{i}", f"{origin} 0 -0.1 0 0 0", "4.2 4.2 0.2", "0.4 0.4 0.4 1")
        for label, dx, dy, size in (
            ("front", 2.1, 0, "0.2 4.2 4"),
            ("back", -2.1, 0, "0.2 4.2 4"),
            ("left", 0, 2.1, "4.2 0.2 4"),
            ("right", 0, -2.1, "4.2 0.2 4"),
        ):
            box(world, f"{label}_{i}", f"{origin + dx} {dy} 1.9 0 0 0", size, "0.5 0.5 0.5 1")
        # Above the ToF FOV, visible left and up in the rendered RGB image.
        box(world, f"red_marker_{i}", f"{origin + 1.7} 0.65 1.65 0 0 0",
            "0.06 0.12 0.12", "1 0 0 1")
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def expected_depth(sensor, row, col):
    lateral = (1 - 2 * (col + 0.5) / 8) * math.tan(math.pi / 8)
    vertical = (1 - 2 * (row + 0.5) / 8) * math.tan(math.pi / 8)
    pitch = math.radians(20)
    yaw = (0, 2 * math.pi / 3, -2 * math.pi / 3)[sensor]
    x = math.cos(pitch) + math.sin(pitch) * vertical
    y = lateral
    z = -math.sin(pitch) + math.cos(pitch) * vertical
    dx = math.cos(yaw) * x - math.sin(yaw) * y
    dy = math.sin(yaw) * x + math.cos(yaw) * y
    distances = [2 / abs(dx), 2 / abs(dy)] if abs(dy) > 1e-10 else [2 / abs(dx)]
    if z < -1e-10:
        distances.append(-1 / z)
    return min(distances)


def read_ppm(path):
    with path.open("rb") as stream:
        assert stream.readline().strip() == b"P6"
        comment = stream.readline().decode("ascii").strip()
        assert comment.startswith("# frame_seq=")
        fields = dict(field.split("=") for field in comment[2:].split())
        width, height = map(int, stream.readline().split())
        assert stream.readline().strip() == b"255"
        pixels = stream.read()
    assert len(pixels) == width * height * 3
    return int(fields["frame_seq"]), int(fields["sim_time_us"]), width, height, pixels


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main(args):
    root = Path(__file__).resolve().parents[1]
    work = Path(args.work_dir).resolve()
    work.mkdir(parents=True, exist_ok=True)
    models = work / "fixture_models"
    runtime = work / "runtime"
    subprocess.run(["python3", str(root / "models/generate_sensor_models.py"),
                    "--output", str(models)], check=True, stdout=subprocess.DEVNULL)
    world = work / "sensor_smoke.sdf"
    make_world(world, models, args.port_base)
    sockets = []
    for i in range(4):
        connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        connection.bind(("127.0.0.1", args.port_base + i))
        connection.setblocking(False)
        sockets.append(connection)
    env = dict(os.environ)
    env["GZ_PARTITION"] = f"sensor-smoke-{os.getpid()}"
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(Path(args.build_dir).resolve())
    env["GZ_SIM_RESOURCE_PATH"] = str(models)
    build = Path(args.build_dir)
    logs = [(work / "gateway.log").open("w", encoding="utf-8"),
            (work / "gazebo.log").open("w", encoding="utf-8")]
    gateway = subprocess.Popen([str(build / "sensor_gateway"), "--runtime-dir", str(runtime),
                                "--udp-base-port", str(args.port_base),
                                "--tof-noise-std-m", "0", "--tof-invalid-probability", "0",
                                "--tof-mixed-pixel-probability", "0"],
                               env=env, stdout=logs[0], stderr=logs[0], start_new_session=True)
    gazebo = subprocess.Popen(["gz", "sim", "-s", "-r", str(world)],
                              env=env, stdout=logs[1], stderr=logs[1], start_new_session=True)
    seen = {}
    counts = {}
    error_max = 0
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            if gazebo.poll() is not None or gateway.poll() is not None:
                raise RuntimeError("engine or gateway exited; inspect gazebo.log/gateway.log")
            ready, _, _ = select.select(sockets, [], [], 0.2)
            for connection in ready:
                while True:
                    try:
                        payload, _ = connection.recvfrom(65535)
                    except BlockingIOError:
                        break
                    record = json.loads(payload)
                    vehicle = sockets.index(connection)
                    assert record["schema"] == 1 and record["vehicle"] == vehicle
                    assert type(record["sim_time_us"]) is int
                    kind = record["kind"]
                    key = (vehicle, kind, record.get("sensor", record.get("peer", -1)))
                    counts[key] = counts.get(key, 0) + 1
                    seen[key] = record
                    if kind == "height_assist":
                        assert set(record) == {"schema", "kind", "vehicle", "sim_time_us", "z_world_m", "source"}
                        assert record["source"] == "SIM_HEIGHT_ASSIST"
                        assert abs(record["z_world_m"] - 1) < 1e-9
                    elif kind == "uwb":
                        assert set(record) == {"schema", "kind", "vehicle", "peer", "sim_time_us", "range_m", "bearing_rad", "valid"}
                        peer = record["peer"]
                        assert peer != vehicle and record["valid"]
                        assert abs(record["range_m"] - abs(peer - vehicle) * 8) < 1e-7
                        bearing = 0 if peer > vehicle else math.pi
                        assert abs(abs(record["bearing_rad"]) - bearing) < 1e-7
                    elif kind == "tof":
                        assert len(record["distance_m"]) == len(record["valid"]) == 64
                        for index, (distance, valid) in enumerate(zip(record["distance_m"], record["valid"])):
                            assert valid, (vehicle, record["sensor"], index, record)
                            expected = expected_depth(record["sensor"], index // 8, index % 8)
                            error_max = max(error_max, abs(distance - expected))
                    else:
                        raise AssertionError(f"unexpected observation kind {kind}")
            if (len(seen) == 28 and min(counts.values()) >= 5 and
                    all((runtime / f"camera_{i}.json").exists() for i in range(4))):
                break
        else:
            raise RuntimeError(f"missing live sensors after {args.timeout}s: {list(seen)}; inspect logs")
    finally:
        stop(gazebo)
        stop(gateway)
        for connection in sockets:
            connection.close()
        for log in logs:
            log.close()
    cameras = []
    for i in range(4):
        path = runtime / f"camera_{i}.ppm"
        metadata = json.loads((runtime / f"camera_{i}.json").read_text(encoding="utf-8"))
        sequence, sim_us, width, height, pixels = read_ppm(path)
        assert metadata["frame_seq"] == sequence
        assert metadata["sim_time_us"] == sim_us
        assert metadata["frame_bytes"] == path.stat().st_size
        red = []
        for index in range(width * height):
            r, g, b = pixels[index * 3:index * 3 + 3]
            if r > 80 and r > 2 * g and r > 2 * b:
                red.append((index % width, index // width))
        assert red, f"no rendered red marker in camera {i}"
        centroid = [sum(p[axis] for p in red) / len(red) for axis in range(2)]
        assert centroid[0] < width / 2 and centroid[1] < height / 2, centroid
        cameras.append({"vehicle": i, "sim_time_us": sim_us, "frame_seq": sequence,
                        "red_marker_centroid": centroid, "frame_bytes": metadata["frame_bytes"]})
    report = {"engine": "gz-sim8", "live_observation_channels": len(seen),
              "minimum_frames_per_channel": min(counts.values()),
              "tof_axial_plane_max_error_m": error_max, "cameras": cameras,
              "height_assist": "zero-noise ENU z scalar only",
              "limitation": "static engine sensor fixtures; not a PX4 flight or micro dynamics validation"}
    (work / "sensor_smoke_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    assert error_max < 0.035, f"ToF pixel projection mismatch: {error_max}m"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", default="/home/devcontainers/drone-sim/build-sensors")
    parser.add_argument("--work-dir", default="/home/devcontainers/drone-sim/build-sensors/smoke")
    parser.add_argument("--port-base", type=int, default=26000)
    parser.add_argument("--timeout", type=float, default=40)
    main(parser.parse_args())
