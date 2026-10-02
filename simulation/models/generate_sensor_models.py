"""Add Gazebo-rendered sensing to a supplied PX4 model without editing it.

Default output is a sensor-only static fixture for engine smoke verification.
Use --base-model /.../x500/model.sdf for the PX4 flight model; the
generator preserves its motor plugins and adds a fixed sensor link.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def child(parent: ET.Element, tag: str, text: object | None = None, **attrs: str):
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


def box_inertia(mass: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    return (mass * (y * y + z * z) / 12,
            mass * (x * x + z * z) / 12,
            mass * (x * x + y * y) / 12)


def cylinder_inertia(mass: float, radius: float, height: float) -> tuple[float, float, float]:
    radial = mass * (3 * radius * radius + height * height) / 12
    return radial, radial, mass * radius * radius / 2


def add_inertial(parent: ET.Element, mass: float,
                 inertia: tuple[float, float, float]) -> None:
    inertial = child(parent, "inertial")
    child(inertial, "mass", f"{mass:.17g}")
    inertia_node = child(inertial, "inertia")
    for axis, value in zip(("ixx", "iyy", "izz"), inertia):
        child(inertia_node, axis, f"{value:.17g}")
    for axis in ("ixy", "ixz", "iyz"):
        child(inertia_node, axis, "0")


def expand_x500_base(source: Path, model: ET.Element) -> None:
    """Inline PX4's merge include so the generated sensor model is standalone."""
    for include in list(model.findall("include")):
        if ((include.get("merge") or "").lower() != "true" or
                (include.findtext("uri") or "").strip() != "model://x500_base"):
            continue
        base_path = source.parent.parent / "x500_base" / "model.sdf"
        base_root = ET.parse(base_path).getroot()
        base_model = base_root.find("model")
        if base_model is None:
            raise ValueError(f"PX4 base model has no model element: {base_path}")
        position = list(model).index(include)
        model.remove(include)
        for element in reversed(list(base_model)):
            model.insert(position, copy.deepcopy(element))


def sensor_link(vehicle: int, pitch_down_deg: float, camera_width: int,
                camera_height: int, camera_hfov: float,
                mount_radius: float, sensor_z: float,
                camera_x: float, mass: float = 0.001) -> ET.Element:
    link = ET.Element("link", {"name": "companion_sensor_link"})
    child(link, "gravity", "true")
    add_inertial(link, mass, box_inertia(mass, 0.04, 0.03, 0.012))
    for sensor_id, yaw_deg in enumerate((0.0, 120.0, -120.0)):
        sensor = child(link, "sensor", name=f"tof_{sensor_id}", type="depth_camera")
        yaw, pitch = math.radians(yaw_deg), math.radians(pitch_down_deg)
        x = mount_radius * math.cos(yaw)
        y = mount_radius * math.sin(yaw)
        child(sensor, "pose", f"{x:.17g} {y:.17g} {sensor_z:.17g} 0 {pitch:.17g} {yaw:.17g}")
        child(sensor, "always_on", "true")
        child(sensor, "update_rate", "15")
        child(sensor, "topic", f"/simulation/vehicle_{vehicle}/tof_{sensor_id}/depth_image")
        camera = child(sensor, "camera")
        child(camera, "horizontal_fov", f"{math.pi / 4:.17g}")
        image = child(camera, "image")
        child(image, "width", "8")
        child(image, "height", "8")
        child(image, "format", "R_FLOAT32")
        clip = child(camera, "clip")
        child(clip, "near", "0.02")
        child(clip, "far", "4.0")
    sensor = child(link, "sensor", name="gc4653_camera", type="camera")
    child(sensor, "pose", f"{camera_x:.17g} 0 {sensor_z:.17g} 0 0 0")
    child(sensor, "always_on", "true")
    child(sensor, "update_rate", "15")
    child(sensor, "topic", f"/simulation/vehicle_{vehicle}/camera/image")
    camera = child(sensor, "camera")
    child(camera, "horizontal_fov", f"{camera_hfov:.17g}")
    image = child(camera, "image")
    child(image, "width", camera_width)
    child(image, "height", camera_height)
    child(image, "format", "R8G8B8")
    clip = child(camera, "clip")
    child(clip, "near", "0.02")
    child(clip, "far", "100")
    return link


def generate(args):
    mode = "x500" if args.base_model else args.dynamics
    if args.dynamics == "micro" and args.base_model:
        raise ValueError("--dynamics micro cannot use --base-model")
    if mode == "x500" and not args.base_model:
        raise ValueError("--dynamics x500 requires --base-model")
    output = Path(args.output or Path(__file__).parent /
                  ("generated_micro" if mode == "micro" else "generated")).resolve()
    if mode == "micro" and output == (Path(__file__).parent / "generated").resolve():
        raise ValueError("micro output must be separate from the existing generated/ x500 models")
    source = None
    if args.base_model:
        source = Path(args.base_model).resolve(strict=True)
        if args.expected_gz_commit:
            import subprocess
            commit = subprocess.run(
                ["git", "-C", str(source.parent), "rev-parse", "HEAD"],
                check=True, capture_output=True, text=True, encoding="utf-8"
            ).stdout.strip()
            if not commit.startswith(args.expected_gz_commit):
                raise ValueError(f"PX4 gz checkout {commit} does not match pinned {args.expected_gz_commit}")
        template = ET.parse(source).getroot().find("model")
        if template is None:
            raise ValueError("base SDF must contain a model")
        template = copy.deepcopy(template)
        expand_x500_base(source, template)
    else:
        template = None
    micro_mass = args.mass_kg
    micro_rotor_mass = micro_mass / 30
    micro_sensor_mass = micro_mass / 30
    micro_body_mass = micro_mass - 4 * micro_rotor_mass - micro_sensor_mass
    micro_body_size = (0.08, 0.06, 0.025)
    micro_rotor_radius = args.rotor_radius_m
    micro_rotor_height = 0.002
    micro_body_inertia = box_inertia(micro_body_mass, *micro_body_size)
    micro_rotor_inertia = cylinder_inertia(
        micro_rotor_mass, micro_rotor_radius, micro_rotor_height)
    micro_sensor_inertia = box_inertia(micro_sensor_mass, 0.04, 0.03, 0.012)
    hover_speed = args.hover_speed_rad_s
    gravity = 9.80665
    arm = args.rotor_arm_m
    moment_constant = 0.006
    if (not all(math.isfinite(value) and value > 0 for value in
                (micro_mass, micro_rotor_radius, hover_speed, arm)) or
            hover_speed >= args.max_motor_speed_rad_s or
            micro_rotor_radius >= arm or arm + micro_rotor_radius > 0.058):
        raise ValueError("mass/speed must be positive, hover below max speed, and rotors must fit within 0.116m without overlap")
    motor_constant = micro_mass * gravity / (4 * hover_speed * hover_speed)
    micro_rotors = (
        (arm, -arm, "ccw", moment_constant),
        (-arm, arm, "ccw", moment_constant),
        (arm, arm, "cw", -moment_constant),
        (-arm, -arm, "cw", -moment_constant),
    )
    if mode == "micro":
        # Keep optical centers above the contact plane when the body rests on
        # the ground. Lateral offsets clear the body without going below it.
        mount_radius, sensor_z, camera_x, sensor_mass = 0.055, 0.0, 0.052, micro_sensor_mass
    elif mode == "x500":
        mount_radius, sensor_z, camera_x, sensor_mass = 0.4, 0.08, 0.4, 0.001
    else:
        mount_radius, sensor_z, camera_x, sensor_mass = 0.0, 0.0, 0.0, 0.001
    tof_origins = [[mount_radius * math.cos(yaw), mount_radius * math.sin(yaw), sensor_z]
                   for yaw in (0, math.radians(120), math.radians(-120))]
    contract = {
        "schema": 1, "distance_mode": "axial", "tof_resolution": [8, 8],
        "tof_hfov_rad": math.pi / 4, "tof_vfov_rad": math.pi / 4,
        "tof_yaw_rad": [0, math.radians(120), math.radians(-120)],
        "tof_pitch_down_deg": args.tof_pitch_down_deg,
        "tof_pitch_down_rad": math.radians(args.tof_pitch_down_deg),
        "tof_origins_body_m": tof_origins, "body_frame": "FLU", "world_frame": "ENU",
        "camera_origin_body_m": [camera_x, 0, sensor_z],
        "mount_calibration_status": "assumed external sensor origins; not measured hardware mounts",
        "camera_width": args.camera_width, "camera_height": args.camera_height,
        "camera_hfov_rad": math.radians(args.camera_hfov_deg),
        "camera_optical_from_body": "(-y_body,-z_body,x_body)",
        "dynamics": ("reference physical micro model; DM-FC01 dynamics uncalibrated"
                     if mode == "micro" else
                     "supplied PX4 flight model" if source else
                     "static sensor fixture; no flight dynamics"),
        "base_model": str(source) if source else None,
    }
    if mode == "micro":
        idle_speed = hover_speed / 6.0
        parameters = {"CA_AIRFRAME": 0, "CA_ROTOR_COUNT": 4,
                      "THR_MDL_FAC": 0.0,
                      "MPC_THR_HOVER": (hover_speed-idle_speed)/(args.max_motor_speed_rad_s-idle_speed)}
        parameters.update({'MC_ROLLRATE_P': 0.025, 'MC_PITCHRATE_P': 0.035, 'MC_YAWRATE_P': 0.1, 'MC_ROLLRATE_I': 0.03, 'MC_PITCHRATE_I': 0.04, 'MC_YAWRATE_I': 0.03, 'MC_ROLLRATE_D': 0.0003, 'MC_PITCHRATE_D': 0.0004, 'MC_YAWRATE_D': 0.0, 'MC_ROLL_P': 4.0, 'MC_PITCH_P': 4.0, 'MC_YAW_P': 2.0})
        parameters["LNDMC_Z_VEL_MAX"] = 0.08
        for rotor_id, (x, y, _, km) in enumerate(micro_rotors):
            # PX4 allocation uses FRD: x_frd=x_flu, y_frd=-y_flu, z_frd=-z_flu.
            parameters.update({f"CA_ROTOR{rotor_id}_PX": x, f"CA_ROTOR{rotor_id}_PY": -y,
                               f"CA_ROTOR{rotor_id}_PZ": -0.015,
                               f"CA_ROTOR{rotor_id}_AX": 0, f"CA_ROTOR{rotor_id}_AY": 0,
                               f"CA_ROTOR{rotor_id}_AZ": -1, f"CA_ROTOR{rotor_id}_CT": 20.0,
                               f"CA_ROTOR{rotor_id}_KM": km,
                               f"SIM_GZ_EC_FUNC{rotor_id + 1}": 101 + rotor_id,
                               f"SIM_GZ_EC_MIN{rotor_id + 1}": idle_speed,
                               f"SIM_GZ_EC_MAX{rotor_id + 1}": args.max_motor_speed_rad_s})
        contract.update({
            "mass_total_kg": micro_mass,
            "collision_envelope_m": [0.104, 0.104, 0.032],
            "protective_guard_radius_m": 0.052,
            "body_mass_kg": micro_body_mass,
            "body_size_m": micro_body_size,
            "body_inertia_kg_m2": micro_body_inertia,
            "rotor_mass_kg": micro_rotor_mass,
            "rotor_radius_m": micro_rotor_radius,
            "rotor_centers_flu_m": [[x, y, 0.015] for x, y, _, _ in micro_rotors],
            "rotor_centers_frd_m": [[x, -y, -0.015] for x, y, _, _ in micro_rotors],
            "reference_extent_limit_m": [0.116, 0.116, 0.116],
            "rotor_inertia_kg_m2": micro_rotor_inertia,
            "sensor_link_mass_kg": micro_sensor_mass,
            "sensor_link_inertia_kg_m2": micro_sensor_inertia,
            "hover_speed_rad_s": hover_speed,
            "idle_speed_rad_s": idle_speed,
            "max_motor_speed_rad_s": args.max_motor_speed_rad_s,
            "gravity_m_s2": gravity,
            "motor_thrust_coefficient_N_s2": motor_constant,
            "hover_thrust_total_N": micro_mass * gravity,
            "hover_thrust_per_rotor_N": micro_mass * gravity / 4,
            "rotor_velocity_slowdown_sim": 10.0,
            "moment_constant_m": moment_constant,
            "px4_parameters": parameters,
            "allocation_scale_note": "CT=20 is a uniform effectiveness normalization to avoid PX4 v1.15 weak-row threshold0.05; physical motorConstant is unchanged.",
            "motor_model": "T=k*omega^2; omega is commanded rad/s, not slowed visual joint speed",
            "normalized_speed_hover": hover_speed / args.max_motor_speed_rad_s,
            "normalized_thrust_hover": (hover_speed / args.max_motor_speed_rad_s) ** 2,
            "calibration_status": "reference only; not tuned to DM-FC01",
        })
    output.mkdir(parents=True, exist_ok=True)
    for vehicle in range(4):
        model_dir = output / f"micro_{vehicle}"
        model_dir.mkdir(exist_ok=True)
        root = ET.Element("sdf", {"version": "1.9"})
        if template is not None:
            model = copy.deepcopy(template)
            model.set("name", f"micro_{vehicle}")
            root.append(model)
            # Relative asset URIs must keep referring to the untouched source.
            for uri in model.iter("uri"):
                value = (uri.text or "").strip()
                if value and "://" not in value and not Path(value).is_absolute():
                    uri.text = (source.parent / value).resolve().as_uri()
        elif mode == "micro":
            model = child(root, "model", name=f"micro_{vehicle}")
            child(model, "static", "false")
            child(model, "self_collide", "false")
            child(model, "pose", "0 0 0 0 0 0")
            base = child(model, "link", name="base_link")
            child(base, "gravity", "true")
            add_inertial(base, micro_body_mass, micro_body_inertia)
            collision = child(base, "collision", name="body_collision")
            guard_shape = child(child(collision, "geometry"), "cylinder")
            child(guard_shape, "radius", "0.052")
            child(guard_shape, "length", "0.032")
            surface = child(collision, "surface")
            friction = child(child(surface, "friction"), "ode")
            child(friction, "mu", "0.2")
            child(friction, "mu2", "0.2")
            bounce = child(surface, "bounce")
            child(bounce, "restitution_coefficient", "0.1")
            child(bounce, "threshold", "0.05")
            guard = child(base, "visual", name="protective_envelope")
            guard_shape = child(child(guard, "geometry"), "cylinder")
            child(guard_shape, "radius", "0.052")
            child(guard_shape, "length", "0.032")
            child(guard, "transparency", "0.88")
            guard_material = child(guard, "material")
            child(guard_material, "ambient", "0.3 0.5 0.6 1")
            child(guard_material, "diffuse", "0.3 0.5 0.6 1")
            contact = child(base, "sensor", name="body_contact", type="contact")
            child(contact, "always_on", "true")
            child(contact, "update_rate", "100")
            child(contact, "topic", f"/evaluation/vehicle_{vehicle}/contacts")
            child(child(contact, "contact"), "collision", "body_collision")
            visual = child(base, "visual", name="body_visual")
            child(child(child(visual, "geometry"), "box"), "size",
                " ".join(f"{value:.17g}" for value in micro_body_size))
            material = child(visual, "material")
            child(material, "ambient", "0.12 0.14 0.16 1")
            child(material, "diffuse", "0.12 0.14 0.16 1")
            for sensor_name, sensor_type, hz in (("imu_sensor", "imu", 250),
                                                 ("air_pressure_sensor", "air_pressure", 50),
                                                 ("navsat_sensor", "navsat", 30)):
                sensor = child(base, "sensor", name=sensor_name, type=sensor_type)
                child(sensor, "always_on", "true")
                child(sensor, "update_rate", hz)
            for rotor_id, (x, y, direction, km) in enumerate(micro_rotors):
                rotor = child(model, "link", name=f"rotor_{rotor_id}")
                child(rotor, "gravity", "true")
                add_inertial(rotor, micro_rotor_mass, micro_rotor_inertia)
                rotor_collision = child(rotor, "collision", name="rotor_collision")
                cylinder = child(child(rotor_collision, "geometry"), "cylinder")
                child(cylinder, "radius", f"{micro_rotor_radius:.17g}")
                child(cylinder, "length", f"{micro_rotor_height:.17g}")
                rotor_visual = child(rotor, "visual", name="rotor_visual")
                cylinder = child(child(rotor_visual, "geometry"), "cylinder")
                child(cylinder, "radius", f"{micro_rotor_radius:.17g}")
                child(cylinder, "length", f"{micro_rotor_height:.17g}")
                child(rotor, "pose", f"{x:.17g} {y:.17g} 0.015 0 0 0")
                joint = child(model, "joint", name=f"rotor_{rotor_id}_joint", type="revolute")
                child(joint, "parent", "base_link")
                child(joint, "child", f"rotor_{rotor_id}")
                axis = child(joint, "axis")
                child(axis, "xyz", "0 0 1")
                limit = child(axis, "limit")
                child(limit, "lower", "-1e16")
                child(limit, "upper", "1e16")
                plugin = child(model, "plugin",
                               filename="gz-sim-multicopter-motor-model-system",
                               name="gz::sim::systems::MulticopterMotorModel")
                child(plugin, "jointName", f"rotor_{rotor_id}_joint")
                child(plugin, "linkName", f"rotor_{rotor_id}")
                child(plugin, "turningDirection", direction)
                child(plugin, "timeConstantUp", "0.0125")
                child(plugin, "timeConstantDown", "0.025")
                child(plugin, "maxRotVelocity", f"{args.max_motor_speed_rad_s:.17g}")
                child(plugin, "motorConstant", f"{motor_constant:.17g}")
                child(plugin, "momentConstant", f"{abs(km):.17g}")
                child(plugin, "commandSubTopic", "command/motor_speed")
                child(plugin, "motorNumber", rotor_id)
                child(plugin, "rotorDragCoefficient", "1e-6")
                child(plugin, "rollingMomentCoefficient", "1e-7")
                child(plugin, "rotorVelocitySlowdownSim", "10")
                child(plugin, "motorType", "velocity")
        else:
            model = child(root, "model", name=f"micro_{vehicle}")
            child(model, "static", "true")
        model.append(sensor_link(vehicle, args.tof_pitch_down_deg, args.camera_width,
                                 args.camera_height, math.radians(args.camera_hfov_deg),
                                 mount_radius, sensor_z, camera_x, sensor_mass))
        if mode != "fixture":
            joint = child(model, "joint", name="companion_sensor_joint", type="fixed")
            child(joint, "parent", args.base_link)
            child(joint, "child", "companion_sensor_link")
        ET.indent(root)
        ET.ElementTree(root).write(model_dir / "model.sdf", encoding="utf-8", xml_declaration=True)
        config = ET.Element("model")
        child(config, "name", f"micro_{vehicle}")
        child(config, "version", "1.0")
        child(config, "sdf", "model.sdf", version="1.9")
        child(config, "description", contract["dynamics"])
        ET.indent(config)
        ET.ElementTree(config).write(model_dir / "model.config", encoding="utf-8", xml_declaration=True)
    (output / "sensor_contract.json").write_text(json.dumps(contract, indent=2) + "\n", encoding="utf-8")
    if mode == "micro":
        (output / "micro_physics_manifest.json").write_text(
            json.dumps(contract, indent=2) + "\n", encoding="utf-8")
        commands = ["# Uncalibrated reference, simulator only. Apply with the matching micro model."]
        commands += [f"param set {key} {value}" for key, value in parameters.items()]
        (output / "px4_micro_reference.params").write_text("\n".join(commands) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(output), **contract}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output")
    parser.add_argument("--base-model")
    parser.add_argument("--base-link", default="base_link")
    parser.add_argument("--expected-gz-commit")
    parser.add_argument("--dynamics", choices=("fixture", "x500", "micro"), default="fixture")
    parser.add_argument("--mass-kg", type=float, default=0.18)
    parser.add_argument("--rotor-arm-m", type=float, default=0.025)
    parser.add_argument("--rotor-radius-m", type=float, default=0.016)
    parser.add_argument("--hover-speed-rad-s", type=float, default=600.0)
    parser.add_argument("--max-motor-speed-rad-s", type=float, default=1200.0)
    parser.add_argument("--tof-pitch-down-deg", "--pitch-down-deg", dest="tof_pitch_down_deg",
                        type=float, default=20.0,
                        help="ToF pitch about +Y in FLU; positive points downward")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=360)
    parser.add_argument("--camera-hfov-deg", type=float, default=90.0)
    generate(parser.parse_args())
