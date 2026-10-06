# @file generate_sensor_models.py
# @brief 生成带三向 ToF 和前视相机的 SDF 模型，并输出安装参数及微型机动力学配置。

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
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET

from mesh_bounds import read_bounds


# 创建并返回 XML 子节点；text 非 None 时转为字符串，属性通过 attrs 传入。
def child(parent: ET.Element, tag: str, text: object | None = None, **attrs: str):
    """@brief 创建 XML 子元素并可选写入文本和属性。

    @param parent 父元素。
    @param tag 元素名称。
    @param text 元素文本；None 表示保留空文本。
    @param attrs 元素属性。
    @return 新建的 XML 元素。
    """
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


# @param mass 均匀长方体质量（kg）。
# @param x,y,z 沿三轴的完整边长（m）。
# @return 质心处 (Ixx,Iyy,Izz)，单位 kg·m²，使用各垂直边长平方和乘 mass/12。
def box_inertia(mass: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    return (mass * (y * y + z * z) / 12,
            mass * (x * x + z * z) / 12,
            mass * (x * x + y * y) / 12)


# 计算均匀实心圆柱的主惯量，轴线沿 Z；mass 为 kg，radius/height 为 m，返回 kg·m²。
def cylinder_inertia(mass: float, radius: float, height: float) -> tuple[float, float, float]:
    radial = mass * (3 * radius * radius + height * height) / 12
    return radial, radial, mass * radius * radius / 2


# 给 link 添加质量及对角惯量；inertia 顺序为 Ixx/Iyy/Izz（kg·m²），惯性积写零。
def add_inertial(parent: ET.Element, mass: float,
                 inertia: tuple[float, float, float]) -> None:
    inertial = child(parent, "inertial")
    child(inertial, "mass", f"{mass:.17g}")
    inertia_node = child(inertial, "inertia")
    for axis, value in zip(("ixx", "iyy", "izz"), inertia):
        child(inertia_node, axis, f"{value:.17g}")
    for axis in ("ixy", "ixz", "iyz"):
        child(inertia_node, axis, "0")


# 原位展开 x500_base 的 merge include；从源模型同级目录取基础模型，以深拷贝保留原节点顺序。
def expand_x500_base(source: Path, model: ET.Element) -> None:
    """Inline PX4's merge include so the generated sensor model is standalone."""
    # 只展开标记为 merge 的 x500_base，其他 include 保持原样。
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


# @param vehicle 车辆编号，用于隔离传感器话题。
# @param pitch_down_deg ToF 在 FLU 机体系绕 +Y 的俯角（度），正值朝下。
# @param camera_width,camera_height 相机输出像素尺寸；camera_hfov 为水平视场弧度。
# @param mount_radius,sensor_z,camera_x 安装位置参数（米），mass 为传感器连杆质量（kg）。
# @return 含三个 8×8 深度相机及一个 RGB 相机的 link 元素，均以 15 Hz 更新。
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


# @param args 命令行解析结果，包括模型类型、安装几何、质量、转速及输出目录。
# @note x500 从外部 SDF 深拷贝，micro 构造刚体与旋翼，fixture 构造静态传感器支架。
# 输出每机 SDF、模型描述及共享传感器契约；micro 另写物理清单和对应 PX4 参数。
def install_visual_mesh(output, args):
    """@brief 把真实 CAD 导出的机体网格装进输出目录，并算出对齐参数。

    @param output 模型输出根目录。
    @param args 命令行解析结果，使用 visual_mesh 及其配套参数。
    @return (相对 URI, 缩放字符串, 位姿字符串, 说明) 四元组；未指定网格时返回 None。
    @note 只替换视觉，不碰碰撞体、质量、惯量与旋翼几何——那些决定 PX4 分配矩阵，
          换掉会连带改变飞行特性。单位按包围盒自动判定：任一轴超过 1 视为毫米导出。
    """
    if not args.visual_mesh:
        # 上一次接入过网格时，去掉参数重跑不应留下无引用的残留文件。
        for stale in output.glob("drone_visual.*"):
            stale.unlink()
        return None
    source = Path(args.visual_mesh)
    if not source.exists():
        raise FileNotFoundError(f"visual mesh not found: {source}")
    bounds = read_bounds(source)
    scale = args.visual_mesh_scale
    note = "scale 由命令行指定"
    if scale is None:
        # SolidWorks 默认按毫米导出；真实机体尺寸在 0.1–1 m 量级，据此自动判定。
        scale = 0.001 if max(bounds.extent) > 1.0 else 1.0
        note = "scale 自动判定为 {}".format("毫米→米" if scale == 0.001 else "已是米")
    target = output / f"drone_visual{source.suffix.lower()}"
    shutil.copyfile(source, target)
    # 网格中心平移到机体原点；--visual-mesh-pose 的平移在缩放后的机体坐标系里追加。
    pose = [
        -bounds.center[0] * scale + args.visual_mesh_pose[0],
        -bounds.center[1] * scale + args.visual_mesh_pose[1],
        -bounds.center[2] * scale + args.visual_mesh_pose[2],
        args.visual_mesh_pose[3],
        args.visual_mesh_pose[4],
        args.visual_mesh_pose[5],
    ]
    print(f"机体网格 {source.name}: {bounds}")
    print(f"  {note}，安装到 {target.name}")
    print(
        "  缩放后尺寸 {:.4f} × {:.4f} × {:.4f} m".format(
            *(value * scale for value in bounds.extent)
        )
    )
    return (
        f"../{target.name}",
        f"{scale:.17g} {scale:.17g} {scale:.17g}",
        " ".join(f"{value:.17g}" for value in pose),
        {
            "source_name": source.name,
            "source_extent": bounds.extent,
            "scale": scale,
            "scale_mode": note,
            "installed_pose": pose,
            "vertices": bounds.vertices,
            "triangles": bounds.triangles,
        },
    )


def generate(args):
    """@brief 按动力学模式生成每机 SDF、模型描述和共享传感器契约。

    @param args 命令行解析结果，包括模型类型、安装几何、质量、转速及输出目录。
    @note x500 从外部 SDF 深拷贝并附加传感器 link；micro 计算刚体/旋翼惯量和 PX4 参数；
    fixture 只生成静态传感器支架。输出契约统一使用 FLU 机体、ENU 世界和米制单位。
    """
    # 1. 确定模型来源并校验输出目录；x500 的 merge include 在内存副本中展开。
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
    # 2. 按总质量分配机身、四个旋翼和传感器质量，再以几何尺寸计算主惯量（kg·m²）。
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
    # 悬停时四桨总推力等于 mg；由单桨 T=kω² 反推 k，ω 使用命令转速 rad/s。
    motor_constant = micro_mass * gravity / (4 * hover_speed * hover_speed)
    micro_rotors = (
        (arm, -arm, "ccw", moment_constant),
        (-arm, arm, "ccw", moment_constant),
        (arm, arm, "cw", -moment_constant),
        (-arm, -arm, "cw", -moment_constant),
    )
    if mode == "micro":
        # 光心保持在机体接地面上方，侧向安装偏移避开机身遮挡。
        mount_radius, sensor_z, camera_x, sensor_mass = 0.055, 0.0, 0.052, micro_sensor_mass
    elif mode == "x500":
        mount_radius, sensor_z, camera_x, sensor_mass = 0.4, 0.08, 0.4, 0.001
    else:
        mount_radius, sensor_z, camera_x, sensor_mass = 0.0, 0.0, 0.0, 0.001
    tof_origins = [[mount_radius * math.cos(yaw), mount_radius * math.sin(yaw), sensor_z]
                   for yaw in (0, math.radians(120), math.radians(-120))]
    # 3. 传感器模型与伴随端共享安装外参；轴向深度不等同于归一化射线上的斜距。
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
            # PX4 分配器使用 FRD：x 保持、y/z 取反；推力沿 FRD 的 -Z。
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
    vehicle_count = getattr(args, "vehicle_count", 4)
    if vehicle_count < 1:
        raise ValueError("vehicle_count must be positive")
    contract["vehicle_count"] = vehicle_count
    # 真实机体网格只装一份，各机用相对 URI 引用，避免按机重复拷贝大文件。
    visual_mesh = install_visual_mesh(output, args) if mode == "micro" else None
    if visual_mesh is not None:
        contract["visual_mesh"] = visual_mesh[3]
    # 4. 每机使用独立模型名和观测话题；动力学 link 与传感器 link 通过固定关节连接。
    for vehicle in range(vehicle_count):
        model_dir = output / f"micro_{vehicle}"
        model_dir.mkdir(exist_ok=True)
        root = ET.Element("sdf", {"version": "1.9"})
        if template is not None:
            model = copy.deepcopy(template)
            model.set("name", f"micro_{vehicle}")
            root.append(model)
            # 模型复制到输出目录后，相对资源 URI 转为原来源的绝对 URI。
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
            if visual_mesh is None:
                child(child(child(visual, "geometry"), "box"), "size",
                    " ".join(f"{value:.17g}" for value in micro_body_size))
            else:
                # 真实 CAD 网格只作视觉；碰撞体仍是上面的保护圈圆柱，
                # 以保证接触判定与 PX4 分配矩阵不随外观变化。
                mesh = child(child(visual, "geometry"), "mesh")
                child(mesh, "uri", visual_mesh[0])
                child(mesh, "scale", visual_mesh[1])
                child(visual, "pose", visual_mesh[2])
                if args.visual_mesh_transparency is not None:
                    child(visual, "transparency", f"{args.visual_mesh_transparency:.17g}")
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
    # 5. 保存共享契约；micro 模式同时导出物理清单与匹配该模型的 PX4 参数。
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
    parser.add_argument("--vehicle-count", type=int, default=4)
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
    # 真实机体外观：SolidWorks 导出 STL/OBJ 后直接接入，只替换视觉不动物理。
    parser.add_argument(
        "--visual-mesh",
        help="真实机体网格（.stl/.obj），作为 base_link 的视觉替换占位方块",
    )
    parser.add_argument(
        "--visual-mesh-scale",
        type=float,
        help="网格到米的缩放；省略时按包围盒自动判定（任一轴>1 视为毫米）",
    )
    parser.add_argument(
        "--visual-mesh-pose",
        type=float,
        nargs=6,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        default=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        help="在自动居中基础上追加的机体系平移与旋转（m、rad）",
    )
    parser.add_argument(
        "--visual-mesh-transparency",
        type=float,
        help="网格透明度 0..1，用于同时观察内部结构；省略表示不透明",
    )
    generate(parser.parse_args())
