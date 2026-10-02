"""Import the pinned Sentry26 terrain and export the static geometry prior."""

import hashlib
import json
import shutil
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent / "downloads/Sentry26/src/simulator/rmu_gazebo_simulator"
COMMIT = "9f3ae17627cef5ca1f102a27668f8b5e78a2cdb1"
DTYPE = np.dtype(
    [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")]
)


def read_stl(path):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + count * 50:
        raise ValueError("Expected complete binary STL")
    records = np.frombuffer(data, dtype=DTYPE, offset=84, count=count).copy()
    if not np.isfinite(records["vertices"]).all():
        raise ValueError("Non-finite mesh vertex")
    return records


def height_grid(triangles, step=0.1):
    """Offline top-down raster of known static prior; keeps highest surface."""
    x = np.arange(-14.6, 14.61, step)
    y = np.arange(-8.1, 8.11, step)
    height = np.full((len(y), len(x)), np.nan)
    for tri in triangles:
        a, b, c = tri
        denominator = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(denominator) < 1e-9:
            continue
        xmin, ymin = (
            np.searchsorted(x, tri[:, 0].min()),
            np.searchsorted(y, tri[:, 1].min()),
        )
        xmax, ymax = (
            np.searchsorted(x, tri[:, 0].max(), side="right"),
            np.searchsorted(y, tri[:, 1].max(), side="right"),
        )
        if xmax <= xmin or ymax <= ymin:
            continue
        xx, yy = np.meshgrid(x[xmin:xmax], y[ymin:ymax])
        u = ((b[1] - c[1]) * (xx - c[0]) + (c[0] - b[0]) * (yy - c[1])) / denominator
        v = ((c[1] - a[1]) * (xx - c[0]) + (a[0] - c[0]) * (yy - c[1])) / denominator
        z = u * a[2] + v * b[2] + (1 - u - v) * c[2]
        valid = (u >= -1e-6) & (v >= -1e-6) & (u + v <= 1 + 1e-6)
        old = height[ymin:ymax, xmin:xmax]
        old[valid] = np.fmax(old[valid], z[valid])
    return x, y, height


def main():
    source = SOURCE / "rmu_gazebo_simulator/resource/models/rmuc_2026"
    output = ROOT / "models/terrain/rmuc_2026_sentry26"
    output.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, output / "upstream", dirs_exist_ok=True)
    shutil.copy2(SOURCE / "LICENSE", output / "LICENSE")
    original = source / "meshes/rmuc_2026.stl"
    records = read_stl(original)
    # Upstream contains enclosing vertical triangles reaching 18.27 m. These
    # would provide undocumented perfect ToF walls at every flight altitude.
    # Preserve them in upstream, exclude them in the explicit open-boundary
    # derivative; no replacement fence height is invented.
    excluded = records["vertices"][:, :, 2].max(axis=1) > 5.0
    kept = records[~excluded].copy()
    shift = np.array([0.0, 0.0, -0.2275390625], dtype=np.float32)
    kept["vertices"] += shift
    mesh = output / "terrain.stl"
    mesh.write_bytes(
        b"Sentry26 derived terrain; see provenance.json".ljust(80, b" ")
        + struct.pack("<I", len(kept))
        + kept.tobytes()
    )
    triangles = kept["vertices"].astype("<f4")
    (output / "terrain_triangles.f32").write_bytes(triangles.tobytes())
    bounds = triangles.min(axis=(0, 1)).tolist(), triangles.max(axis=(0, 1)).tolist()
    metadata = {
        "source_url": "https://github.com/Boombroke/Sentry26",
        "commit": COMMIT,
        "source_asset": str(original.relative_to(ROOT.parent)).replace("\\", "/"),
        "source_sha256": hashlib.sha256(original.read_bytes()).hexdigest(),
        "derived_sha256": hashlib.sha256(mesh.read_bytes()).hexdigest(),
        "season": 2026,
        "official_2027_geometry": False,
        "upstream_license": "Apache-2.0 at rmu_gazebo_simulator/LICENSE; no asset-specific origin certificate supplied",
        "original_triangle_count": len(records),
        "retained_triangle_count": len(kept),
        "excluded_triangle_count": int(excluded.sum()),
        "excluded_indices": np.flatnonzero(excluded).tolist(),
        "exclusion": "All triangles with any vertex z > 5 m: upstream enclosing walls reach 18.268883 m. No replacement walls added; boundary is open at height.",
        "world_translation_m": shift.tolist(),
        "world_bounds_m": bounds,
        "triangle_format": "little-endian float32, consecutive triangle vertices xyz, 36 bytes/triangle, no header",
        "limitations": [
            "Not a verified 2027 preview field",
            "No rule referee, dynamic ground robots, calibrated fence/net geometry or radio propagation model",
            "Meshes are static; floor texture not mapped by upstream STL model",
        ],
    }
    (output / "provenance.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    (output / "model.config").write_text(
        '<model><name>rmuc_2026_sentry26</name><version>1</version><sdf version="1.9">model.sdf</sdf></model>',
        encoding="utf-8",
    )
    (output / "model.sdf").write_text(
        """<sdf version="1.9"><model name="rmuc_2026_sentry26"><static>true</static><link name="terrain">
      <visual name="terrain_visual"><geometry><mesh><uri>model://rmuc_2026_sentry26/terrain.stl</uri></mesh></geometry><material><ambient>0.42 0.48 0.50 1</ambient><diffuse>0.55 0.61 0.63 1</diffuse></material></visual>
      <collision name="terrain_collision"><geometry><mesh><uri>model://rmuc_2026_sentry26/terrain.stl</uri></mesh></geometry><surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface></collision>
    </link></model></sdf>""",
        encoding="utf-8",
    )
    tree = ET.parse(ROOT / "worlds/calibration.sdf")
    world = tree.getroot().find("world")
    world.set("name", "rmuc_2026_sentry26")
    for model in list(world.findall("model")):
        world.remove(model)
    include = ET.SubElement(world, "include")
    ET.SubElement(include, "uri").text = "model://rmuc_2026_sentry26"
    # Overhead RGB sensor is output for operator/evaluation only.
    overview = ET.SubElement(world, "model", name="operator_overview")
    ET.SubElement(overview, "static").text = "true"
    ET.SubElement(
        overview, "pose"
    ).text = "0 0 28 0 1.5707963267948966 1.5707963267948966"
    link = ET.SubElement(overview, "link", name="camera")
    sensor = ET.SubElement(link, "sensor", name="overview", type="camera")
    ET.SubElement(sensor, "topic").text = "/operator/terrain_overview"
    ET.SubElement(sensor, "update_rate").text = "1"
    camera = ET.SubElement(sensor, "camera")
    ET.SubElement(camera, "horizontal_fov").text = "1.15"
    img = ET.SubElement(camera, "image")
    ET.SubElement(img, "width").text = "1280"
    ET.SubElement(img, "height").text = "800"
    ET.SubElement(img, "format").text = "R8G8B8"
    clip = ET.SubElement(camera, "clip")
    ET.SubElement(clip, "near").text = "0.1"
    ET.SubElement(clip, "far").text = "60"
    ET.indent(tree)
    tree.write(
        ROOT / "worlds/rmuc_2026_sentry26.sdf", encoding="utf-8", xml_declaration=True
    )
    x, y, heights = height_grid(triangles)
    np.savez_compressed(output / "terrain_height_grid.npz", x=x, y=y, z=heights)
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, ax = plt.subplots(figsize=(13, 7))
    im = ax.imshow(
        heights,
        origin="lower",
        extent=[x[0], x[-1], y[0], y[-1]],
        cmap="terrain",
        vmin=0,
        vmax=3,
    )
    ax.set_aspect("equal")
    ax.set_xlabel("world X (m)")
    ax.set_ylabel("world Y (m)")
    ax.set_title(
        "Sentry26 2026 terrain prior / open boundary derivative"
    )
    ax.grid(alpha=0.3)
    figure.colorbar(im, ax=ax, label="Top surface Z (m)")
    figure.savefig(ROOT / "docs/terrain-height.png", dpi=150, bbox_inches="tight")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
