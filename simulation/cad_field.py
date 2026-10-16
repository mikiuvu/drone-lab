"""展开 STEP 装配体，保留部件路径、颜色和世界坐标三角网格。"""

import hashlib
import json
from pathlib import Path

import numpy as np


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def extract_step(source, output, deflection_m=0.003, healing=True):
    """输出毫米制 CAD 网格及部件目录；实例位置由装配树逐层相乘。"""
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDF import TDF_Label
    from OCP.TDocStd import TDocStd_Document
    from OCP.XCAFDoc import XCAFDoc_ColorGen, XCAFDoc_ColorSurf, XCAFDoc_DocumentTool
    try:
        from OCP.TDF import TDF_LabelSequence
    except ImportError:
        from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
    from OCP.Bnd import Bnd_Box
    from OCP.BRep import BRep_Tool
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.Quantity import Quantity_Color
    from OCP.TDataStd import TDataStd_Name
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS

    if not np.isfinite(deflection_m) or deflection_m <= 0:
        raise ValueError('CAD deflection must be finite and positive')
    source, output = Path(source).resolve(strict=True), Path(output)
    document = TDocStd_Document(TCollection_ExtendedString('BinXCAF'))
    reader = STEPCAFControl_Reader()
    print(f'Reading {source}', flush=True)
    if reader.ReadFile(str(source)) != IFSelect_RetDone:
        raise ValueError(f'Cannot read STEP: {source}')
    # OCCT 的转移单位显式设为 mm，与 STEP 文件自己的单位声明分别处理。
    reader.Reader().SetSystemLengthUnit(1.0)
    if not healing:
        from OCP.collections import (
            DataMap_TCollection_AsciiString_TCollection_AsciiString,
        )
        from OCP.TCollection import TCollection_AsciiString
        parameters = DataMap_TCollection_AsciiString_TCollection_AsciiString()
        for mode in ('FixFreeShellMode', 'FixFreeFaceMode', 'FixFreeWireMode', 'FixSolidMode', 'FixSameParameterMode'):
            parameters.Bind(TCollection_AsciiString('FixShape.' + mode), TCollection_AsciiString('0'))
        reader.SetShapeFixParameters(parameters)
    print('Transferring assembly', flush=True)
    if not reader.Transfer(document):
        raise ValueError(f'Cannot transfer STEP assembly: {source}')
    tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    colors = XCAFDoc_DocumentTool.ColorTool_s(document.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    parts, shapes = [], []

    def label_name(label):
        value = TDataStd_Name()
        return value.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), value) else ''

    def bounds(shape):
        box = Bnd_Box()
        BRepBndLib.Add_s(shape, box)
        if box.IsVoid():
            return None
        lo, hi = box.CornerMin(), box.CornerMax()
        return [[lo.X(), lo.Y(), lo.Z()], [hi.X(), hi.Y(), hi.Z()]]

    def walk(label, location, path):
        location = location.Multiplied(tool.GetLocation_s(label))
        path = path + [label_name(label)]
        if tool.IsReference_s(label):
            referred = TDF_Label()
            tool.GetReferredShape_s(label, referred)
            label = referred
            path.append(label_name(label))
        children = TDF_LabelSequence()
        if tool.GetComponents_s(label, children):
            for i in range(1, children.Length() + 1):
                walk(children.Value(i), location, path)
        else:
            shape = tool.GetShape_s(label).Located(location)
            if shape.IsNull() or bounds(shape) is None:
                return
            color = Quantity_Color()
            found = any(colors.GetColor_s(label, kind, color) for kind in (XCAFDoc_ColorSurf, XCAFDoc_ColorGen))
            rgb = [color.Red(), color.Green(), color.Blue()] if found else [.55, .60, .62]
            parts.append({'id': len(parts), 'path': path, 'bounds_mm': bounds(shape), 'rgb': rgb})
            shapes.append(shape)

    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])
    output.mkdir(parents=True, exist_ok=True)
    (output / 'parts.json').write_text(json.dumps(parts, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'Assembly: {len(parts)} parts', flush=True)
    chunks, ids = [], []
    face_of = getattr(TopoDS, 'Face_s', None) or TopoDS.Face
    for i, (shape, part) in enumerate(zip(shapes, parts)):
        BRepMesh_IncrementalMesh(shape, deflection_m * 1000, False, 0.35, True).Perform()
        faces = TopExp_Explorer(shape, TopAbs_FACE)
        triangles = []
        while faces.More():
            face = face_of(faces.Current())
            location = TopLoc_Location()
            mesh = BRep_Tool.Triangulation_s(face, location)
            if mesh is not None:
                transform = location.Transformation()
                vertices = []
                for j in range(1, mesh.NbNodes() + 1):
                    point = mesh.Node(j).Transformed(transform)
                    vertices.append([point.X(), point.Y(), point.Z()])
                indices = np.asarray([mesh.Triangle(j).Get() for j in range(1, mesh.NbTriangles() + 1)], dtype=int)
                if indices.size:
                    if face.Orientation() == TopAbs_REVERSED:
                        indices = indices[:, [0, 2, 1]]
                    triangles.append(np.asarray(vertices)[indices - 1])
            faces.Next()
        tri = np.concatenate(triangles).astype('<f4') if triangles else np.empty((0, 3, 3), dtype='<f4')
        chunks.append(tri)
        ids.append(np.full(len(tri), i, dtype='<i4'))
        part['triangles'] = len(tri)
        if i % 50 == 0:
            print(f'Meshed {i}/{len(parts)}', flush=True)
    if not chunks or not any(len(chunk) for chunk in chunks):
        raise ValueError('STEP assembly contains no meshable faces')
    output.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output / 'cad_mesh.npz', triangles_mm=np.concatenate(chunks), part_ids=np.concatenate(ids))
    (output / 'parts.json').write_text(json.dumps(parts, ensure_ascii=False, indent=2), encoding='utf-8')
    manifest = {'source_sha256': sha256(source), 'deflection_m': deflection_m, 'healing': healing,
                    'occt_transfer_unit': 'mm', 'part_count': len(parts), 'triangle_count': sum(map(len, chunks))}
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    return manifest


def transform_triangles(triangles_mm, matrix):
    """4×4 右手刚体变换：先把 CAD 毫米坐标换为米，再转到地图世界系。"""
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('CAD transform must be a finite 4x4 matrix')
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9):
        raise ValueError('Invalid homogeneous transform')
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9) or not np.isclose(np.linalg.det(rotation), 1):
        raise ValueError('CAD transform must preserve lengths and handedness')
    triangles = np.asarray(triangles_mm, dtype=float)
    if triangles.ndim != 3 or triangles.shape[1:] != (3, 3) or not np.isfinite(triangles).all():
        raise ValueError('Invalid CAD triangle array')
    return (triangles * .001 @ rotation.T + matrix[:3, 3]).astype('<f4')
