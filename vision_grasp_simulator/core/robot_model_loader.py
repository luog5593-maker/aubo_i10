"""AUBO i10 与 PGC_300_60 模型资源解析、路径兼容和程序化回退模型。"""
from __future__ import annotations
from dataclasses import dataclass, field
from pathlib import Path
import math
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Tuple
import numpy as np

AUBO_LINKS = ["base_link", "shoulder_Link", "upperArm_Link", "foreArm_Link", "wrist1_Link", "wrist2_Link", "wrist3_Link"]
AUBO_JOINTS = ["shoulder_joint", "upperArm_joint", "foreArm_joint", "wrist1_joint", "wrist2_joint", "wrist3_joint"]

@dataclass
class JointInfo:
    name: str
    parent: str
    child: str
    origin_xyz: Tuple[float, float, float]
    origin_rpy: Tuple[float, float, float]
    axis_xyz: Tuple[float, float, float]
    lower: float
    upper: float

@dataclass
class LinkInfo:
    name: str
    mesh_path: Optional[Path] = None
    mesh_loaded: bool = False
    fallback_reason: str = ""
    vertices: Optional[np.ndarray] = None
    faces: Optional[np.ndarray] = None
    size: Tuple[float, float, float] = (0.08, 0.08, 0.12)

@dataclass
class RobotVisualModel:
    links: Dict[str, LinkInfo] = field(default_factory=dict)
    joints: List[JointInfo] = field(default_factory=list)
    gripper_meshes: Dict[str, LinkInfo] = field(default_factory=dict)
    messages: List[str] = field(default_factory=list)
    using_real_aubo_mesh: bool = False
    using_real_gripper_mesh: bool = False
    aubo_root: Optional[Path] = None
    gripper_root: Optional[Path] = None

class RobotModelLoader:
    """优先读取 assets，缺失时兼容上级目录；mesh 缺失时自动回退为程序化几何体。"""
    def __init__(self, project_root: str | Path):
        self.project_root = Path(project_root).resolve()
        self.assets = self.project_root / "assets"

    def load(self) -> RobotVisualModel:
        model = RobotVisualModel()
        aubo_root = self._first_existing([self.assets / "aubo_i10", self.project_root.parent / "aubo i10 meshes"])
        gripper_root = self._first_existing([self.assets / "pgc_300_60", self.project_root.parent / "pgc_300_60"])
        model.aubo_root = aubo_root; model.gripper_root = gripper_root
        model.joints, mesh_map = self._parse_aubo_urdf(aubo_root)
        model.links = self._load_aubo_links(aubo_root, mesh_map, model.messages)
        model.gripper_meshes = self._load_gripper(gripper_root, model.messages)
        model.using_real_aubo_mesh = any(l.mesh_loaded for l in model.links.values())
        model.using_real_gripper_mesh = any(l.mesh_loaded for l in model.gripper_meshes.values())
        aubo_loaded_count = sum(1 for l in model.links.values() if l.mesh_loaded)
        gripper_loaded_count = sum(1 for l in model.gripper_meshes.values() if l.mesh_loaded)
        model.messages.append(f"已加载 AUBO i10 真实外观模型（{aubo_loaded_count}/7 个link mesh，已做显示归一化）" if model.using_real_aubo_mesh else "未找到 AUBO i10 mesh，已使用程序化模型替代")
        model.messages.append(f"已加载 PGC_300_60 电爪模型（{gripper_loaded_count}/4 个mesh，已做显示归一化）" if model.using_real_gripper_mesh else "未找到 PGC_300_60 STL，已使用程序化夹爪模型替代")
        return model

    def _first_existing(self, paths: List[Path]) -> Optional[Path]:
        for p in paths:
            if p.exists(): return p
        return None

    def _parse_vec(self, text: Optional[str], default=(0.0,0.0,0.0)):
        if not text: return default
        vals = [float(x) for x in text.split()[:3]]
        return tuple(vals + list(default)[len(vals):])

    def _parse_aubo_urdf(self, aubo_root: Optional[Path]):
        urdf = aubo_root / "aubo_i10_gazebo_roscontrol.urdf" if aubo_root else None
        if urdf is None or not urdf.exists():
            return self._default_joints(), {}
        try:
            root = ET.parse(urdf).getroot(); mesh_map = {}; joints = []
            for link in root.findall("link"):
                name = link.attrib.get("name", "")
                mesh = link.find("./visual/geometry/mesh")
                if name and mesh is not None and mesh.attrib.get("filename"):
                    mesh_map[name] = self._resolve_aubo_mesh(mesh.attrib["filename"], aubo_root)
            for joint in root.findall("joint"):
                name = joint.attrib.get("name", "")
                if name not in AUBO_JOINTS: continue
                parent = joint.find("parent").attrib.get("link", "")
                child = joint.find("child").attrib.get("link", "")
                origin = joint.find("origin"); axis = joint.find("axis"); limit = joint.find("limit")
                joints.append(JointInfo(name,parent,child,self._parse_vec(origin.attrib.get("xyz") if origin is not None else None),self._parse_vec(origin.attrib.get("rpy") if origin is not None else None),self._parse_vec(axis.attrib.get("xyz") if axis is not None else None,(0,0,1)),float(limit.attrib.get("lower",-math.pi)) if limit is not None else -math.pi,float(limit.attrib.get("upper",math.pi)) if limit is not None else math.pi))
            return (joints if len(joints) > 0 else self._default_joints()), mesh_map
        except Exception:
            return self._default_joints(), {}

    def _resolve_aubo_mesh(self, filename: str, aubo_root: Path) -> Path:
        name = Path(filename.replace("package://aubo_description/meshes/aubo_i10/", "")).name
        preferred = aubo_root / "meshes" / name
        if preferred.suffix.lower() == ".stl":
            dae = preferred.with_suffix(".DAE")
            if dae.exists(): return dae
        return preferred

    def _default_joints(self):
        limits = [(-3.04,3.04),(-3.04,3.04),(-3.04,3.04),(-3.04,3.04),(-3.04,3.04),(-6.28,6.28)]
        parents = AUBO_LINKS[:-1]; children = AUBO_LINKS[1:]
        origins = [(0,0,0.12),(0,0,0.10),(0.22,0,0),(0.22,0,0),(0.10,0,0),(0.08,0,0)]
        return [JointInfo(n,p,c,o,(0,0,0),(0,0,1),lo,hi) for n,p,c,o,(lo,hi) in zip(AUBO_JOINTS,parents,children,origins,limits)]

    def _load_aubo_links(self, aubo_root, mesh_map, messages):
        sizes = {"base_link":(.18,.18,.10),"shoulder_Link":(.12,.12,.20),"upperArm_Link":(.36,.08,.08),"foreArm_Link":(.34,.07,.07),"wrist1_Link":(.12,.08,.08),"wrist2_Link":(.10,.07,.07),"wrist3_Link":(.09,.06,.06)}
        links = {}
        for name in AUBO_LINKS:
            candidates = []
            if mesh_map.get(name): candidates.append(mesh_map[name])
            if aubo_root is not None: candidates += [aubo_root/"meshes"/f"{name}.DAE", aubo_root/"meshes"/f"{name}.STL"]
            links[name] = self._load_mesh_or_fallback(name, candidates, sizes.get(name), messages, "link")
        return links

    def _load_gripper(self, gripper_root, messages):
        sizes={"pgc_300_60":(.12,.08,.05),"pgc_300_60_base":(.10,.07,.04),"pgc_300_60_left_finger":(.025,.025,.10),"pgc_300_60_right_finger":(.025,.025,.10)}
        result={}
        for name,size in sizes.items():
            candidates=[gripper_root/"meshes"/f"{name}.stl"] if gripper_root is not None else []
            result[name]=self._load_mesh_or_fallback(name,candidates,size,messages,"gripper")
        return result

    def _load_mesh_or_fallback(self, name, candidates, size, messages, kind):
        for path in candidates:
            if path is not None and path.exists():
                loaded = self._try_load_mesh(path, size)
                if loaded is not None:
                    v,f = loaded
                    if v is not None and len(v) > 0 and f is not None and len(f) > 0:
                        return LinkInfo(name,path,True,"",v,f,size)
        messages.append(f"{name} mesh 缺失或无法加载，已使用程序化模型替代")
        return LinkInfo(name,None,False,"fallback",None,None,size)

    def _try_load_mesh(self, path: Path, target_size):
        try:
            import trimesh
            mesh = trimesh.load_mesh(path, force='mesh')
            if mesh.is_empty: return None
            vertices = np.asarray(mesh.vertices, dtype=float)
            faces = np.asarray(mesh.faces, dtype=int)
            if vertices.size == 0 or faces.size == 0:
                return None
            # 不直接使用 CAD/DAE/STL 的原始坐标范围：不同模型常带有 mm/m 单位、
            # 大偏置或导出坐标系差异。先把 mesh 居中，再按该 link 的程序化尺寸
            # 等比缩放，保证真实外观 mesh 能稳定显示在 AUBO 运动链对应 link 上。
            v_min = np.min(vertices, axis=0)
            v_max = np.max(vertices, axis=0)
            center = (v_min + v_max) / 2.0
            extents = v_max - v_min
            max_extent = float(np.max(extents)) if extents.size > 0 else 0.0
            target_extent = float(max(target_size)) if target_size is not None else 0.1
            if max_extent <= 0:
                return None
            scale = target_extent / max_extent
            normalized = (vertices - center) * scale
            return normalized, faces
        except Exception:
            return None
