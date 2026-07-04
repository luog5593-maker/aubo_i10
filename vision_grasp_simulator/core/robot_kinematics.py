import math, numpy as np
from .data_models import RobotPose

class RobotKinematics:
    """六自由度机械臂简化DH/几何混合运动学模型，用于仿真演示。"""
    def __init__(self, cfg):
        self.link=cfg.get('link_lengths',[120,220,220,120,80,60]); self.limits=cfg.get('joint_limits',[])
        self.workspace_radius=cfg.get('workspace_radius',620); self.workspace_z=cfg.get('workspace_z',[0,620])
        self.home=np.array(cfg.get('home_angles',[0,-35,70,0,55,0]),float); self.joint_angles=self.home.copy()
    def reset(self): self.joint_angles=self.home.copy(); return self.joint_angles.tolist()
    def clamp(self, angles):
        a=np.array(angles,float)
        for i,(lo,hi) in enumerate(self.limits): a[i]=min(max(a[i],lo),hi)
        return a
    def forward(self, angles=None):
        a=np.radians(self.joint_angles if angles is None else angles); l=self.link
        j1,j2,j3,j4,j5,j6=a; pts=[np.array([0.,0.,0.]),np.array([0.,0.,l[0]])]
        r1=l[1]*math.cos(j2); z1=l[0]+l[1]*math.sin(j2)
        r2=r1+l[2]*math.cos(j2+j3); z2=z1+l[2]*math.sin(j2+j3)
        r3=r2+(l[3]+l[4]+l[5])*math.cos(j2+j3+j5); z3=z2+(l[3]+l[4]+l[5])*math.sin(j2+j3+j5)
        for r,z in [(r1,z1),(r2,z2),(r3,z3)]: pts.append(np.array([r*math.cos(j1),r*math.sin(j1),z]))
        pose=RobotPose(float(pts[-1][0]),float(pts[-1][1]),float(pts[-1][2]),math.degrees(j4),math.degrees(j5),math.degrees(j1+j6))
        return pts, pose
    def is_reachable(self, target):
        x,y,z=target; return math.hypot(x,y)<=self.workspace_radius and self.workspace_z[0]<=z<=self.workspace_z[1]
    def inverse(self, target, seed=None):
        if not self.is_reachable(target): return None, '目标点超出工作空间'
        x,y,z=target; l=self.link; yaw=math.degrees(math.atan2(y,x)); r=math.hypot(x,y)-(l[3]+l[4]+l[5]); dz=z-l[0]
        L1,L2=l[1],l[2]; d=math.hypot(r,dz)
        if d> L1+L2 or d<abs(L1-L2): return None, '逆运动学无可行解'
        c3=(d*d-L1*L1-L2*L2)/(2*L1*L2); c3=max(-1,min(1,c3)); j3=math.acos(c3)
        j2=math.atan2(dz,r)-math.atan2(L2*math.sin(j3),L1+L2*math.cos(j3))
        angles=[yaw,math.degrees(j2),math.degrees(j3),0,-(math.degrees(j2)+math.degrees(j3))/2,0]
        return self.clamp(angles).tolist(), 'OK'
