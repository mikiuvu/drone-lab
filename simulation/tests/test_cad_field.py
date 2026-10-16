"""CAD 单位、实例装配变换和网格坐标验证。"""

import json
import math

import numpy as np
import pytest

from simulation.cad_field import extract_step, transform_triangles


def test_world_transform_converts_mm_then_rotates_and_translates():
    triangles = np.array([[[1000, 0, 0], [0, 1000, 0], [0, 0, 1000]]])
    matrix = [[0, -1, 0, 2], [1, 0, 0, 3], [0, 0, 1, 4], [0, 0, 0, 1]]
    np.testing.assert_allclose(transform_triangles(triangles, matrix),
                               [[[2, 4, 4], [1, 3, 4], [2, 3, 5]]])


@pytest.mark.parametrize('matrix', [np.diag([2, 1, 1, 1]), np.diag([-1, 1, 1, 1]),
    np.full((4, 4), np.nan), np.eye(3), np.diag([1, 1, 1, 2])])
def test_invalid_frame_rejected(matrix):
    with pytest.raises(ValueError):
        transform_triangles(np.zeros((1, 3, 3)), matrix)


@pytest.mark.parametrize('healing', [True, False])
def test_step_nested_instances_match_kernel_bounds(tmp_path, healing):
    pytest.importorskip('OCP')
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.STEPControl import STEPControl_AsIs
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFDoc import XCAFDoc_ColorSurf, XCAFDoc_DocumentTool
    doc = TDocStd_Document(TCollection_ExtendedString('BinXCAF'))
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    part = tool.AddShape(BRepPrimAPI_MakeBox(20, 30, 40).Shape(), False)
    XCAFDoc_DocumentTool.ColorTool_s(doc.Main()).SetColor(part, Quantity_Color(1, 0, 0, Quantity_TOC_RGB), XCAFDoc_ColorSurf)
    subassembly, root = tool.NewShape(), tool.NewShape()
    shift = gp_Trsf()
    shift.SetTranslation(gp_Vec(100, 200, 300))
    tool.AddComponent(subassembly, part, TopLoc_Location(shift))
    rotation = gp_Trsf()
    rotation.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), math.pi / 2)
    tool.AddComponent(root, subassembly, TopLoc_Location(rotation))
    tool.UpdateAssemblies()
    writer = STEPCAFControl_Writer()
    assert writer.Transfer(doc, STEPControl_AsIs)
    source = tmp_path / 'assembly.step'
    assert writer.Write(str(source)) == IFSelect_RetDone
    cache = tmp_path / 'cache'
    extract_step(source, cache, healing=healing)
    with np.load(cache / 'cad_mesh.npz', allow_pickle=False) as data:
        vertices = data['triangles_mm']
    np.testing.assert_allclose(vertices.min(axis=(0, 1)), [-230, 100, 300], atol=1e-5)
    np.testing.assert_allclose(vertices.max(axis=(0, 1)), [-200, 120, 340], atol=1e-5)
    parts = json.loads((cache / 'parts.json').read_text(encoding='utf-8'))
    assert len(parts) == 1
    np.testing.assert_allclose(parts[0]['rgb'], [1, 0, 0])
