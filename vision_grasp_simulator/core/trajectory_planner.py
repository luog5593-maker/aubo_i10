import numpy as np
class TrajectoryPlanner:
    """MoveJ关节插值与MoveL笛卡尔直线插值规划。"""
    def movej(self, start, end, steps=40): return [np.linspace(start[i],end[i],steps).tolist() for i in range(len(start))]
    def movej_points(self,start,end,steps=40): return [list(p) for p in np.linspace(start,end,steps)]
    def movel_targets(self, start_xyz, end_xyz, steps=35): return [tuple(p) for p in np.linspace(start_xyz,end_xyz,steps)]
