class SafetyChecker:
    """自动抓取前的置信度、目标存在性、工作空间和IK安全判断。"""
    def __init__(self, min_confidence=0.5): self.min_confidence=min_confidence
    def check_detection(self, det):
        if det is None: return False,'没有检测到可抓取目标，禁止自动抓取'
        if det.confidence < self.min_confidence: return False,'检测置信度低于阈值，禁止抓取'
        return True,'OK'
    def check_target(self, robot, xyz):
        if not robot.is_reachable(xyz): return False,'目标超出机械臂工作空间'
        a,msg=robot.inverse(xyz); return (a is not None, msg)
