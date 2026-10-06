"""@file mesh_bounds.py
@brief 读取 STL/OBJ 网格的坐标包围盒，供模型生成器把真实 CAD 网格对齐到机体系。

SolidWorks 可直接导出 STL 与 OBJ，两者都能用标准库解析，因此这里不引入任何
第三方依赖。包围盒用于三件事：判断导出单位（mm 还是 m）、把网格中心平移到机体
原点、以及核对尺寸是否与物理契约相符。

@note 只读取顶点，不构建拓扑；二进制与 ASCII STL 都支持。
"""

import pathlib
import struct

# 二进制 STL：80 字节头 + 4 字节三角面数，其后每个三角面 50 字节。
BINARY_HEADER_BYTES = 84
BINARY_TRIANGLE_BYTES = 50


class MeshBounds:
    """@brief 网格顶点包围盒与统计量。"""

    def __init__(self, low, high, vertices, triangles):
        """@brief 保存包围盒与计数。
        @param low 三个轴的最小值。
        @param high 三个轴的最大值。
        @param vertices 读到的顶点数。
        @param triangles 读到的三角面数。
        """
        self.low = list(low)
        self.high = list(high)
        self.vertices = vertices
        self.triangles = triangles

    @property
    def extent(self):
        """@return 三个轴的尺寸，单位与文件一致。"""
        return [self.high[i] - self.low[i] for i in range(3)]

    @property
    def center(self):
        """@return 包围盒中心，用于把网格平移到机体原点。"""
        return [(self.high[i] + self.low[i]) / 2.0 for i in range(3)]

    def __str__(self):
        """@return 单行摘要，便于在生成日志里打印。"""
        return (
            "尺寸 {:.2f} × {:.2f} × {:.2f}，中心 ({:.2f}, {:.2f}, {:.2f})，"
            "{} 顶点 / {} 面".format(
                *self.extent, *self.center, self.vertices, self.triangles
            )
        )


def _is_binary_stl(data):
    """@brief 依据长度公式判断是否为二进制 STL。
    @param data 文件原始字节。
    @return 为真时按二进制解析。
    """
    if len(data) < BINARY_HEADER_BYTES:
        return False
    count = struct.unpack_from("<I", data, 80)[0]
    return len(data) == BINARY_HEADER_BYTES + count * BINARY_TRIANGLE_BYTES


def read_stl(path, data):
    """@brief 解析 STL 顶点。
    @param path 文件路径，用于报错信息。
    @param data 文件原始字节。
    @return (顶点列表, 三角面数) 二元组。
    """
    if _is_binary_stl(data):
        count = struct.unpack_from("<I", data, 80)[0]
        vertices = []
        for index in range(count):
            offset = BINARY_HEADER_BYTES + index * BINARY_TRIANGLE_BYTES + 12
            values = struct.unpack_from("<9f", data, offset)
            vertices.extend(zip(values[0::3], values[1::3], values[2::3]))
        return vertices, count

    vertices = []
    triangles = 0
    for line in data.decode("utf-8", errors="replace").splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == "vertex":
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
        elif len(parts) >= 2 and parts[0] == "facet":
            triangles += 1
    if not vertices:
        raise ValueError(f"{path.name}: 没有解析到 STL 顶点")
    return vertices, triangles


def read_obj(path, data):
    """@brief 解析 OBJ 顶点行，忽略面片与材质。
    @param path 文件路径，用于报错信息。
    @param data 文件原始字节。
    @return (顶点列表, 面数) 二元组。
    """
    vertices = []
    faces = 0
    for line in data.decode("utf-8", errors="replace").splitlines():
        if line.startswith("v "):
            parts = line.split()
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
        elif line.startswith("f "):
            faces += 1
    if not vertices:
        raise ValueError(f"{path.name}: 没有解析到 OBJ 顶点")
    return vertices, faces


def read_bounds(path):
    """@brief 读取网格文件的顶点包围盒。
    @param path 网格文件路径，扩展名决定解析方式。
    @return MeshBounds 实例。
    @note 只支持 STL 与 OBJ；其他格式请显式给出缩放与位姿参数。
    """
    path = pathlib.Path(path)
    data = path.read_bytes()
    suffix = path.suffix.lower()
    if suffix == ".stl":
        vertices, triangles = read_stl(path, data)
    elif suffix == ".obj":
        vertices, triangles = read_obj(path, data)
    else:
        raise ValueError(
            f"{path.name}: 只支持 .stl 与 .obj 的自动测量，"
            "其他格式请用 --visual-mesh-scale 与 --visual-mesh-pose 手工指定"
        )
    axes = list(zip(*vertices))
    return MeshBounds(
        [min(axis) for axis in axes],
        [max(axis) for axis in axes],
        len(vertices),
        triangles,
    )
