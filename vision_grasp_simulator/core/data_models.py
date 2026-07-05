from dataclasses import dataclass, asdict
from typing import List, Tuple, Optional

@dataclass
class DetectionResult:
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]
    center: Tuple[int, int]
    world: Optional[Tuple[float, float, float]] = None
    def to_dict(self): return asdict(self)

@dataclass
class RobotPose:
    x: float; y: float; z: float; r: float; p: float; yaw: float

@dataclass
class TrajectoryPoint:
    step: int
    joint_angles: List[float]
    pose: RobotPose
    gripper_closed: bool = False
