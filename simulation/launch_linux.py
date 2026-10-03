# @file launch_linux.py
# @brief 管理 Linux/WSL 中的 Gazebo、PX4 和伴随进程，生成运行配置并回收进程组。

"""Launch real Gazebo/PX4 processes in WSL; no scripted trajectories or truth odometry.

This manager's pose knowledge is confined to one-time spawning and evaluation.
"""

import argparse
import json
import math
import os
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    from simulation.fleet_config import configure_fleet
except ImportError:
    from fleet_config import configure_fleet

ROOT = Path(__file__).resolve().parent
PX4 = Path("/home/devcontainers/drone-sim/PX4-Autopilot")
SENSORS = Path("/home/devcontainers/drone-sim/build-sensors")
ACTIVE = ROOT / "runtime" / "manager.json"


# @brief 为同一次运行的所有进程提供一致的 Gazebo 通信与资源搜索环境。
# @param dynamics micro 或 x500，决定模型资源目录。
# @param horizontal 是否使用水平 ToF 的 micro 模型。
# @return 当前环境的副本，设置 Gazebo 分区、资源路径、插件路径和 Python 工具路径。
def environment(dynamics="micro", horizontal=False):
    env = dict(os.environ)
    env["GZ_PARTITION"] = "drone-lab-four"
    model_folder = (
        ("generated_micro_horizontal" if horizontal else "generated_micro")
        if dynamics == "micro"
        else "generated"
    )
    env["GZ_SIM_RESOURCE_PATH"] = (
        str(ROOT / "models" / model_folder)
        + ":"
        + str(PX4 / "Tools/simulation/gz/models")
        + ":"
        + str(ROOT / "models" / "terrain")
    )
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = str(SENSORS)
    env["PATH"] = "/home/devcontainers/drone-sim/venv/bin:" + env.get("PATH", "")
    return env


# @brief 从地图世界模板生成含机队和观测插件的本次运行世界。
# @param output 本次运行的世界 SDF 输出路径。
# @param map_path 地图 JSON 路径，用于解析世界与地形的相对路径。
# @param config 已解析机队配置；None 时从地图加载。
# @return 世界名称和实际采用的配置；写入传感器插件、出生模型及可选起点标记。
def prepare_world(output, map_path, config=None):
    config = (
        configure_fleet(json.loads(map_path.read_text(encoding="utf-8")))
        if config is None
        else config
    )
    # 先加入接触检测和 50 Hz 观测插件，再写入每机模型，保证观测编号与出生点一致。
    tree = ET.parse(map_path.parent / config["world_file"])
    world = tree.getroot().find("world")
    ET.SubElement(
        world,
        "plugin",
        filename="gz-sim-contact-system",
        name="gz::sim::systems::Contact",
    )
    plugin = ET.SubElement(
        world,
        "plugin",
        filename="libpeer_height_system.so",
        name="drone_sim::PeerHeightSystem",
    )
    ET.SubElement(plugin, "publish_hz").text = "50"
    ET.SubElement(plugin, "port_base").text = "16000"
    ET.SubElement(plugin, "seed").text = "2027"
    ET.SubElement(plugin, "vehicle_count").text = str(len(config["spawns"]))
    flow = config.get("optical_flow", {})
    ET.SubElement(plugin, "flow_enabled").text = str(flow.get("enabled", True)).lower()
    for key, default in (
        ("noise_std_mps", 0.015),
        ("scale_error", 0.005),
        ("drop_probability", 0.01),
    ):
        ET.SubElement(plugin, "flow_" + key).text = str(flow.get(key, default))
    anchors = config.get("uwb_anchors", [])
    ET.SubElement(plugin, "anchor_count").text = str(len(anchors))
    for i, anchor in enumerate(anchors):
        ET.SubElement(plugin, f"anchor_{i}_id").text = str(anchor["id"])
        ET.SubElement(plugin, f"anchor_{i}_position").text = " ".join(
            str(anchor[k]) for k in ("x", "y", "z")
        )
    if "terrain_triangles_file" in config:
        ET.SubElement(plugin, "terrain_triangles_file").text = str(
            (map_path.parent / config["terrain_triangles_file"]).resolve()
        )
    for spawn in config["spawns"]:
        i = spawn["id"]
        include = ET.SubElement(world, "include")
        ET.SubElement(include, "uri").text = f"model://micro_{i}"
        ET.SubElement(include, "name").text = f"micro_{i}"
        ET.SubElement(include, "pose").text = " ".join(
            str(spawn[k]) for k in ("x", "y", "z", "roll", "pitch", "yaw")
        )
        if "home_marker" in config:
            marker = config["home_marker"]
            model = ET.SubElement(world, "model", name=f"home_{i}")
            ET.SubElement(model, "static").text = "true"
            ET.SubElement(
                model, "pose"
            ).text = f"{spawn['x']} {spawn['y']} {marker['ground_z_m'] + 0.002} 0 0 0"
            link = ET.SubElement(model, "link", name="outline")
            size, width = marker["size_m"], marker["line_width_m"]
            for edge, (x, y, sx, sy) in enumerate(
                (
                    (0, -size / 2, size, width),
                    (0, size / 2, size, width),
                    (-size / 2, 0, width, size),
                    (size / 2, 0, width, size),
                )
            ):
                visual = ET.SubElement(link, "visual", name=f"edge_{edge}")
                ET.SubElement(visual, "pose").text = f"{x} {y} 0 0 0 0"
                box = ET.SubElement(ET.SubElement(visual, "geometry"), "box")
                ET.SubElement(box, "size").text = f"{sx} {sy} .002"
                material = ET.SubElement(visual, "material")
                ET.SubElement(material, "ambient").text = "1 0 0 1"
                ET.SubElement(material, "diffuse").text = "1 0 0 1"
                ET.SubElement(material, "emissive").text = ".7 0 0 1"
    if "home_marker" in config:
        overview = ET.SubElement(world, "model", name="home_overview")
        ET.SubElement(overview, "static").text = "true"
        ET.SubElement(
            overview, "pose"
        ).text = "0 0 1.8 0 1.5707963267948966 1.5707963267948966"
        link = ET.SubElement(overview, "link", name="camera")
        sensor = ET.SubElement(link, "sensor", name="home_overview", type="camera")
        ET.SubElement(sensor, "topic").text = "/operator/home_overview"
        ET.SubElement(sensor, "update_rate").text = "2"
        camera = ET.SubElement(sensor, "camera")
        ET.SubElement(camera, "horizontal_fov").text = "1.8"
        image = ET.SubElement(camera, "image")
        ET.SubElement(image, "width").text = "960"
        ET.SubElement(image, "height").text = "720"
        ET.SubElement(image, "format").text = "R8G8B8"
        clip = ET.SubElement(camera, "clip")
        ET.SubElement(clip, "near").text = ".05"
        ET.SubElement(clip, "far").text = "5"
    ET.indent(tree)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return world.attrib["name"], config


# @brief 创建独立 Linux 会话并记录进程身份，供停止与状态查询使用。
# @param name 日志名称和状态记录中的进程名称。
# @param args 子进程参数序列；路径在启动前转为字符串。
# @param env 子进程环境。
# @param run_dir 日志目录。
# @param cwd 子进程工作目录，None 继承当前目录。
# @return pid、命令、日志路径和 /proc 启动 tick，供之后确认进程身份。
# @note 新会话使 PID 同时成为进程组 ID；返回后子进程独立运行，stdout/stderr 共用日志。
def start_one(name, args, env, run_dir, cwd=None):
    path = run_dir / (name + ".log")
    with path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(
            list(map(str, args)),
            cwd=cwd,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=log,
            start_new_session=True,
        )
    return {
        "name": name,
        "pid": process.pid,
        "command": list(map(str, args)),
        "log": str(path),
        "start_ticks": Path(f"/proc/{process.pid}/stat")
        .read_text()
        .rsplit(")", 1)[1]
        .split()[19],
    }


# 校验 PID 对应进程非僵尸且启动 tick 与记录一致，避免 PID 重用后误认进程；无法读取返回 False。
def alive(item):
    try:
        stat = Path(f"/proc/{item['pid']}/stat").read_text().rsplit(")", 1)[1].split()
        return stat[0] != "Z" and stat[19] == item["start_ticks"]
    except (OSError, KeyError, IndexError):
        return False


# 将已启动进程及地图配置写入临时 JSON，再替换 manager.json，使读取方获得完整快照。
def write_status(items, run_dir, config):
    ACTIVE.parent.mkdir(parents=True, exist_ok=True)
    tmp = ACTIVE.with_suffix(".tmp")
    tmp.write_text(
        json.dumps(
            {
                "run_dir": str(run_dir),
                "processes": items,
                "map": config,
                "height_assist": True,
                "official_field": config.get("official_field", False),
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    tmp.replace(ACTIVE)


# 逆启动顺序向仍匹配身份的进程组发送 SIGTERM，最多等待 8 秒，再以 SIGKILL 终止残留组。
# 状态保留在 manager.json 并标记 stopped，便于后续读取运行记录。
def stop():
    if not ACTIVE.exists():
        return
    state = json.loads(ACTIVE.read_text(encoding="utf-8"))
    for item in reversed(state["processes"]):
        if alive(item):
            try:
                os.killpg(item["pid"], signal.SIGTERM)
            except ProcessLookupError:
                pass
    deadline = time.monotonic() + 8
    while any(alive(p) for p in state["processes"]) and time.monotonic() < deadline:
        time.sleep(0.1)
    for item in reversed(state["processes"]):
        if alive(item):
            try:
                os.killpg(item["pid"], signal.SIGKILL)
            except ProcessLookupError:
                pass
    state["stopped"] = True
    ACTIVE.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")


# 解析 start/stop/status/gui 命令；启动时准备配置、等待世界时钟、启动传感器和各 PX4 实例。
# 每次启动后保存进程清单；异常路径先保存已启动项，再调用 stop 回收。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=["start", "stop", "status", "gui"])
    parser.add_argument(
        "--map", type=Path, default=ROOT / "maps" / "rmuc_2026_sentry26.json"
    )
    parser.add_argument(
        "--mission", choices=["hover", "survey", "team_contact"], default="hover"
    )
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--dynamics", choices=["micro", "x500"], default="micro")
    parser.add_argument("--horizontal-tof", action="store_true")
    parser.add_argument("--vehicle-count", type=int)
    parser.add_argument(
        "--uwb-anchors", action=argparse.BooleanOptionalAction, default=None
    )
    parser.add_argument("--flow-disable", action="store_true")
    parser.add_argument(
        "--companions",
        action="store_true",
        help="Start actual observation consumers and external odometry; no arming by default",
    )
    parser.add_argument(
        "--fly",
        action="store_true",
        help="Enable virtual SITL offboard and arming only, requires --companions",
    )
    parser.add_argument(
        "--vertical-only",
        action="store_true",
        help="Fuse only external height during plumbing verification",
    )
    args = parser.parse_args()
    if args.fly and (not args.companions or args.vertical_only):
        parser.error("--fly requires --companions and full estimated horizontal fusion")
    if args.mission == "team_contact" and args.dynamics != "micro":
        parser.error("team_contact requires the micro vehicle model")
    if args.action == "stop":
        stop()
        return
    if args.action == "status":
        state = (
            json.loads(ACTIVE.read_text(encoding="utf-8"))
            if ACTIVE.exists()
            else {"processes": []}
        )
        for item in state["processes"]:
            item["alive"] = alive(item)
        print(json.dumps(state, ensure_ascii=False))
        return
    env = environment(args.dynamics, args.horizontal_tof)
    if args.action == "gui":
        subprocess.Popen(["gz", "sim", "-g"], env=env, start_new_session=True)
        return
    if ACTIVE.exists():
        previous = json.loads(ACTIVE.read_text(encoding="utf-8"))
        if any(alive(p) for p in previous["processes"]):
            raise SystemExit("An existing simulation is still running; stop it first.")
    for path in (
        PX4 / "build/px4_sitl_default/bin/px4",
        SENSORS / "sensor_gateway",
        SENSORS / "libpeer_height_system.so",
    ):
        if not path.is_file():
            raise SystemExit("Missing build artifact: " + str(path))
    run_dir = ROOT / "runtime" / time.strftime("run_%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=False)
    config = configure_fleet(
        json.loads(args.map.read_text(encoding="utf-8")),
        args.vehicle_count,
        args.uwb_anchors,
    )
    count = len(config["spawns"])
    if count > 245:
        raise SystemExit("PX4 MAVLink system IDs permit at most 245 local instances")
    if args.flow_disable:
        config["optical_flow"]["enabled"] = False
    world_name, config = prepare_world(
        run_dir / "world.sdf", args.map.resolve(), config
    )
    if args.mission == "team_contact" and "team_mission" not in config:
        raise SystemExit("Selected map has no team_mission configuration")
    model_folder = (
        ("generated_micro_horizontal" if args.horizontal_tof else "generated_micro")
        if args.dynamics == "micro"
        else "generated"
    )
    # 从模型契约同步传感器安装外参，保证模拟深度与伴随端投影使用同一几何。
    contract = json.loads(
        (ROOT / "models" / model_folder / "sensor_contract.json").read_text(
            encoding="utf-8"
        )
    )
    source_model = ROOT / "models" / model_folder / "micro_0/model.sdf"
    for i in range(count):
        destination = ROOT / "models" / model_folder / f"micro_{i}"
        if not (destination / "model.sdf").exists():
            destination.mkdir(parents=True, exist_ok=True)
            tree = ET.parse(source_model)
            for element in tree.iter():
                for key, value in list(element.attrib.items()):
                    element.set(key, value.replace("micro_0", f"micro_{i}"))
                if element.text:
                    element.text = element.text.replace(
                        "vehicle_0", f"vehicle_{i}"
                    ).replace("micro_0", f"micro_{i}")
            tree.write(
                destination / "model.sdf", encoding="utf-8", xml_declaration=True
            )
            (destination / "model.config").write_text(
                f'<model><name>micro_{i}</name><version>1</version><sdf version="1.9">model.sdf</sdf></model>',
                encoding="utf-8",
            )
    settings = json.loads((ROOT / "config.json").read_text(encoding="utf-8"))
    settings["mission"]["mode"] = (
        "team_contact" if args.mission == "team_contact" else "localization"
    )
    if args.mission == "hover":
        config["vehicle_waypoints"] = {str(i): [] for i in range(count)}
    # 运行地图移到新目录后，用绝对路径维持地形先验引用。
    if "terrain_triangles_file" in config:
        config["terrain_triangles_file"] = str(
            (args.map.resolve().parent / config["terrain_triangles_file"]).resolve()
        )
    config["mission_mode"] = args.mission
    config["source_map"] = str(args.map.resolve())
    run_map = run_dir / "map.json"
    run_map.write_text(
        json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    settings["companion"]["tof_mounts"] = [
        {
            "translation_body_m": contract["tof_origins_body_m"][i],
            "yaw_rad": contract["tof_yaw_rad"][i],
            "pitch_down_rad": contract["tof_pitch_down_rad"],
            "roll_rad": 0,
            "horizontal_fov_rad": contract["tof_hfov_rad"],
            "vertical_fov_rad": contract["tof_vfov_rad"],
            "min_range_m": 0.02,
            "max_range_m": 4.0,
        }
        for i in range(3)
    ]
    (run_dir / "companion_config.json").write_text(
        json.dumps(settings, indent=2), encoding="utf-8"
    )
    items = []
    try:
        items.append(
            start_one(
                "gazebo", ["gz", "sim", "-s", "-r", run_dir / "world.sdf"], env, run_dir
            )
        )
        write_status(items, run_dir, config)
        # 等待世界时钟话题出现再启动观测网关，超时使用本机单调时钟。
        deadline = time.monotonic() + 35
        while time.monotonic() < deadline:
            if not alive(items[0]):
                raise RuntimeError("Gazebo exited; inspect " + items[0]["log"])
            response = subprocess.run(
                ["gz", "topic", "-l"],
                env=env,
                capture_output=True,
                timeout=8,
                check=False,
            )
            if f"/world/{world_name}/clock".encode() in response.stdout:
                break
            time.sleep(0.5)
        else:
            raise RuntimeError("Gazebo world clock missing")
        items.append(
            start_one(
                "sensors",
                [
                    SENSORS / "sensor_gateway",
                    "--runtime-dir",
                    run_dir / "camera",
                    "--vehicle-count",
                    str(count),
                ],
                env,
                run_dir,
            )
        )
        write_status(items, run_dir, config)
        for i in range(count):
            directory = (
                Path("/home/devcontainers/drone-sim/runtime")
                / run_dir.name
                / f"px4_{i}"
            )
            directory.mkdir(parents=True)
            hooks = directory / "hooks"
            hooks.mkdir()
            # rcS 从 PATH 查找每机参数钩子；这里关闭 GPS 并选择外部里程计融合项。
            params = "\n".join(
                [
                    "param set EKF2_GPS_CTRL 0",
                    "param set SENS_EN_GPSSIM 0",
                    "param set EKF2_EV_CTRL "
                    + (
                        "2"
                        if args.vertical_only
                        else "7"
                        if config.get("optical_flow", {}).get("enabled", True)
                        else "3"
                    ),
                    "param set EKF2_HGT_REF 3",
                    "param set EKF2_BARO_CTRL 0",
                    "param set EKF2_EV_DELAY 0",
                    "param set EKF2_EV_POS_X 0",
                    "param set EKF2_EV_POS_Y 0",
                    "param set EKF2_EV_POS_Z 0",
                    "param set COM_RC_IN_MODE 4",
                    "param set COM_OF_LOSS_T 0.5",
                    "",
                ]
            )
            if args.dynamics == "micro":
                params += (
                    ROOT / "models" / "generated_micro" / "px4_micro_reference.params"
                ).read_text(encoding="utf-8")
            for name, value in config.get("px4_parameters", {}).items():
                if (
                    not name.replace("_", "").isalnum()
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value)
                ):
                    raise ValueError("Invalid PX4 parameter override")
                params += f"\nparam set {name} {value}\n"
            (hooks / "px4-rc.params").write_text(params, encoding="utf-8")
            # 大机队为每机分配独立端口，避免默认脚本复用远端端口。
            if count > 10:
                mavlink_script = PX4 / "ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink"
                lines = mavlink_script.read_text(encoding="utf-8").splitlines()
                lines = [
                    line
                    for line in lines
                    if "udp_offboard_port_remote=14549" not in line
                ]
                content = (
                    "\n".join(lines)
                    .replace("14580+px4_instance", "22000+px4_instance")
                    .replace("14540+px4_instance", "23000+px4_instance")
                )
                (hooks / "px4-rc.mavlink").write_text(content + "\n", encoding="utf-8")
            pe = dict(
                env,
                PX4_SYS_AUTOSTART="4001",
                PX4_GZ_STANDALONE="1",
                PX4_GZ_WORLD=world_name,
                PX4_GZ_MODEL_NAME=f"micro_{i}",
                PX4_SIM_MODEL="gz_x500",
            )
            pe["PATH"] = (
                str(hooks)
                + ":"
                + str(PX4 / "build/px4_sitl_default/bin")
                + ":"
                + env["PATH"]
            )
            rootfs = PX4 / "build/px4_sitl_default/etc"
            items.append(
                start_one(
                    f"px4_{i}",
                    [
                        PX4 / "build/px4_sitl_default/bin/px4",
                        "-i",
                        str(i),
                        "-d",
                        "-s",
                        rootfs / "init.d-posix/rcS",
                        str(rootfs),
                    ],
                    pe,
                    run_dir,
                    cwd=directory,
                )
            )
            write_status(items, run_dir, config)
        if args.gui:
            items.append(start_one("gazebo_gui", ["gz", "sim", "-g"], env, run_dir))
            write_status(items, run_dir, config)
        if args.companions:
            binary = Path(
                "/home/devcontainers/drone-sim/build-companion/drone_companion_runner"
            )
            if not binary.exists():
                raise RuntimeError("Companion executable missing")
            items.append(
                start_one(
                    "evaluator",
                    [
                        "/usr/bin/python3",
                        ROOT / "evaluate_linux.py",
                        "--world",
                        world_name,
                        "--output",
                        run_dir / "truth_evaluation_only.jsonl",
                    ],
                    env,
                    run_dir,
                )
            )
            if args.mission == "team_contact":
                items.append(
                    start_one(
                        "team_coordinator",
                        [
                            binary.parent / "team_coordinator",
                            "--map",
                            run_map,
                            "--runtime",
                            run_dir,
                        ],
                        env,
                        run_dir,
                    )
                )
                write_status(items, run_dir, config)
            for i in range(count):
                command = [
                    binary,
                    "--vehicle",
                    str(i),
                    "--config",
                    run_dir / "companion_config.json",
                    "--map",
                    run_map,
                    "--runtime",
                    run_dir,
                    "--duration",
                    "0",
                ]
                if args.fly:
                    command += ["--offboard", "--arm"]
                if count > 10:
                    command += [
                        "--mavlink-local-port",
                        str(23000 + i),
                        "--mavlink-remote-port",
                        str(22000 + i),
                    ]
                items.append(start_one(f"companion_{i}", command, env, run_dir))
                write_status(items, run_dir, config)
        print(
            json.dumps(
                {
                    "run_dir": str(run_dir),
                    "world": world_name,
                    "processes": items,
                    "status": "started_not_yet_flight_verified",
                },
                ensure_ascii=False,
            )
        )
    except Exception:
        write_status(items, run_dir, config)
        stop()
        raise


if __name__ == "__main__":
    main()
