"""@file test_visual_mesh.py
@brief 验证真实机体网格接入：STL/OBJ 解析、单位自动判定、居中位姿与 SDF 接线。
@note 用例通过子进程调用 simulation/models/generate_sensor_models.py，与
      sensor_smoke.py 的做法一致；不依赖 Gazebo，只检查生成的 SDF 结构。
"""

import json
import os
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "simulation" / "models" / "generate_sensor_models.py"
sys.path.insert(0, str(ROOT / "simulation" / "models"))

from mesh_bounds import read_bounds  # noqa: E402


def write_binary_stl(path, size_mm):
    """@brief 写一个轴对齐长方体的二进制 STL。
    @param path 输出路径。
    @param size_mm 三个轴的尺寸，单位与文件一致。
    """
    x, y, z = size_mm
    corners = [
        (0, 0, 0), (x, 0, 0), (x, y, 0), (0, y, 0),
        (0, 0, z), (x, 0, z), (x, y, z), (0, y, z),
    ]
    faces = [
        (0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6),
        (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2),
        (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0),
    ]
    with path.open("wb") as handle:
        handle.write(b"\0" * 80)
        handle.write(struct.pack("<I", len(faces)))
        for a, b, c in faces:
            handle.write(struct.pack("<3f", 0.0, 0.0, 0.0))
            for index in (a, b, c):
                handle.write(struct.pack("<3f", *corners[index]))
            handle.write(struct.pack("<H", 0))


def write_ascii_stl(path, size_mm):
    """@brief 写一个 ASCII STL，用于覆盖非二进制分支。
    @param path 输出路径。
    @param size_mm 三个轴的尺寸。
    """
    x, y, z = size_mm
    corners = [
        (0, 0, 0), (x, 0, 0), (x, y, 0), (0, y, 0),
        (0, 0, z), (x, 0, z), (x, y, z), (0, y, z),
    ]
    lines = ["solid test"]
    for a, b, c in ((0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6)):
        lines += ["  facet normal 0 0 0", "    outer loop"]
        lines += ["      vertex {} {} {}".format(*corners[i]) for i in (a, b, c)]
        lines += ["    endloop", "  endfacet"]
    lines.append("endsolid test")
    path.write_text("\n".join(lines), encoding="utf-8")


def write_obj(path, size_mm):
    """@brief 写一个只含顶点与面片索引的 OBJ。
    @param path 输出路径。
    @param size_mm 三个轴的尺寸。
    """
    x, y, z = size_mm
    lines = [
        "v 0 0 0", f"v {x} 0 0", f"v {x} {y} 0", f"v 0 {y} 0",
        f"v 0 0 {z}", f"v {x} 0 {z}", f"v {x} {y} {z}", f"v 0 {y} {z}",
        "f 1 2 3", "f 1 3 4",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def run_generator(mesh, output, extra=(), vehicles=2):
    """@brief 调用模型生成器。
    @param mesh 网格路径或 None。
    @param output 输出目录。
    @param extra 追加的命令行参数。
    @param vehicles 生成的机数。
    @return 子进程结果对象。
    """
    command = [
        sys.executable, str(GENERATOR),
        "--dynamics", "micro",
        "--output", str(output),
        "--vehicle-count", str(vehicles),
    ]
    if mesh is not None:
        command += ["--visual-mesh", str(mesh), *extra]
    # 生成器输出中文；显式指定 UTF-8，避免在 GBK 控制台上解码失败。
    return subprocess.run(
        command, capture_output=True, text=True, encoding="utf-8",
        errors="replace", cwd=str(ROOT),
        env=dict(os.environ, PYTHONIOENCODING="utf-8"),
    )


def body_visual(output, vehicle=0):
    """@return 指定机 model.sdf 里的 body_visual 元素。"""
    tree = ET.parse(output / f"micro_{vehicle}" / "model.sdf")
    return tree.find(".//visual[@name='body_visual']")


def test_read_bounds_handles_both_stl_encodings(tmp_path):
    """@brief 二进制与 ASCII STL 都应量出同一包围盒。"""
    binary = tmp_path / "binary.stl"
    ascii_ = tmp_path / "ascii.stl"
    write_binary_stl(binary, (104.0, 80.0, 40.0))
    write_ascii_stl(ascii_, (104.0, 80.0, 40.0))
    for path in (binary, ascii_):
        bounds = read_bounds(path)
        assert bounds.extent == pytest.approx([104.0, 80.0, 40.0])
        assert bounds.center == pytest.approx([52.0, 40.0, 20.0])
    assert read_bounds(binary).triangles == 12


def test_read_bounds_handles_obj(tmp_path):
    """@brief OBJ 顶点应被正确读出。"""
    path = tmp_path / "part.obj"
    write_obj(path, (0.2, 0.1, 0.05))
    bounds = read_bounds(path)
    assert bounds.extent == pytest.approx([0.2, 0.1, 0.05])
    assert bounds.vertices == 8


def test_read_bounds_rejects_unsupported_suffix(tmp_path):
    """@brief STEP 等格式必须明确报错，而不是静默按错误单位处理。"""
    path = tmp_path / "part.step"
    path.write_text("ISO-10303-21;", encoding="utf-8")
    with pytest.raises(ValueError, match="只支持 .stl 与 .obj"):
        read_bounds(path)


def test_millimetre_export_is_auto_scaled_and_centred(tmp_path):
    """@brief 毫米导出应自动缩放为米，并把网格中心平移到机体原点。"""
    mesh = tmp_path / "drone.stl"
    write_binary_stl(mesh, (104.0, 104.0, 40.0))
    output = tmp_path / "generated"
    result = run_generator(mesh, output)
    assert result.returncode == 0, result.stderr

    visual = body_visual(output)
    assert visual is not None
    assert visual.find("geometry/box") is None
    assert visual.find("geometry/mesh/uri").text == "../drone_visual.stl"
    assert visual.find("geometry/mesh/scale").text == pytest.approx("0.001 0.001 0.001")
    pose = [float(v) for v in visual.find("pose").text.split()]
    assert pose[:3] == pytest.approx([-0.052, -0.052, -0.020])
    assert pose[3:] == pytest.approx([0.0, 0.0, 0.0])
    # 网格只装一份，各机用相对 URI 引用，避免按机重复拷贝大文件。
    assert (output / "drone_visual.stl").exists()
    assert not (output / "micro_0" / "drone_visual.stl").exists()
    contract = json.loads((output / "sensor_contract.json").read_text(encoding="utf-8"))
    assert contract["visual_mesh"]["scale"] == pytest.approx(0.001)
    assert contract["visual_mesh"]["triangles"] == 12


def test_metre_export_is_not_scaled(tmp_path):
    """@brief 已经是米的导出不应再被缩放。"""
    mesh = tmp_path / "drone.stl"
    write_ascii_stl(mesh, (0.2, 0.2, 0.05))
    output = tmp_path / "generated"
    assert run_generator(mesh, output).returncode == 0
    scale = body_visual(output).find("geometry/mesh/scale").text
    assert [float(v) for v in scale.split()] == pytest.approx([1.0, 1.0, 1.0])


def test_explicit_scale_and_pose_override(tmp_path):
    """@brief 显式缩放与追加位姿应覆盖自动判定结果。"""
    mesh = tmp_path / "drone.stl"
    write_binary_stl(mesh, (104.0, 104.0, 40.0))
    output = tmp_path / "generated"
    result = run_generator(
        mesh, output,
        ("--visual-mesh-scale", "0.002",
         "--visual-mesh-pose", "0", "0", "0.03", "0", "0", "0"),
    )
    assert result.returncode == 0, result.stderr
    visual = body_visual(output)
    assert [float(v) for v in visual.find("geometry/mesh/scale").text.split()] == pytest.approx(
        [0.002, 0.002, 0.002]
    )
    pose = [float(v) for v in visual.find("pose").text.split()]
    # 中心 z = 40/2×0.002 = 0.04，加 0.03 抬升后为 -0.01。
    assert pose[2] == pytest.approx(-0.01)


def test_missing_mesh_keeps_placeholder_box(tmp_path):
    """@brief 不指定网格时保持原来的占位方块，行为不变。"""
    output = tmp_path / "generated"
    assert run_generator(None, output).returncode == 0
    visual = body_visual(output)
    assert visual.find("geometry/box") is not None
    assert visual.find("geometry/mesh") is None
    assert not (output / "drone_visual.stl").exists()


def test_regenerating_without_mesh_removes_stale_mesh(tmp_path):
    """@brief 接入过网格后再不带参数重跑，不应留下无引用的残留网格文件。"""
    mesh = tmp_path / "drone.stl"
    write_binary_stl(mesh, (104.0, 104.0, 40.0))
    output = tmp_path / "generated"
    assert run_generator(mesh, output).returncode == 0
    assert (output / "drone_visual.stl").exists()
    assert run_generator(None, output).returncode == 0
    assert not (output / "drone_visual.stl").exists()
    assert body_visual(output).find("geometry/box") is not None


def test_unknown_mesh_path_fails_loudly(tmp_path):
    """@brief 网格路径不存在时应报错退出，而不是生成没有视觉的模型。"""
    output = tmp_path / "generated"
    result = run_generator(tmp_path / "missing.stl", output)
    assert result.returncode != 0
    assert "visual mesh not found" in (result.stderr + result.stdout)


def test_visual_mesh_is_ignored_for_fixture_dynamics(tmp_path):
    """@brief fixture 模式没有机体视觉，不应因给了网格而报错或改动物理。"""
    mesh = tmp_path / "drone.stl"
    write_binary_stl(mesh, (104.0, 104.0, 40.0))
    output = tmp_path / "fixture"
    command = [
        sys.executable, str(GENERATOR),
        "--dynamics", "fixture",
        "--output", str(output),
        "--vehicle-count", "1",
        "--visual-mesh", str(mesh),
    ]
    result = subprocess.run(
        command, capture_output=True, text=True, encoding="utf-8",
        errors="replace", cwd=str(ROOT),
        env=dict(os.environ, PYTHONIOENCODING="utf-8"),
    )
    assert result.returncode == 0, result.stderr
    assert not (output / "drone_visual.stl").exists()
