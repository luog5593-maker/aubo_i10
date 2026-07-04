class GraspSimulator:
    """生成自动抓取仿真流程关键点。"""
    def __init__(self, cfg):
        self.pre_h=cfg.get('pre_grasp_height',120); self.lift_h=cfg.get('lift_height',160); self.place=tuple(cfg.get('place_point',[260,-260,90]))
    def sequence(self, target):
        x,y,z=target; pre=(x,y,z+self.pre_h); lift=(x,y,z+self.lift_h); place_pre=(self.place[0],self.place[1],self.place[2]+self.pre_h)
        return [('运动到预抓取点',pre,False),('下降到抓取点',(x,y,z),False),('虚拟夹爪闭合',(x,y,z),True),('抬升目标',lift,True),('移动到放置点上方',place_pre,True),('下降到放置点',self.place,True),('虚拟夹爪打开',self.place,False),('返回初始位姿',None,False)]
