import cv2, numpy as np
from .data_models import DetectionResult

class VisionDetector:
    """YOLO优先、模拟检测兜底的目标检测模块。"""
    def __init__(self, weight_path='', confidence=0.5):
        self.weight_path=weight_path; self.confidence=confidence; self.model=None; self.mode='模拟检测'
        self.try_load_yolo(weight_path)
    def try_load_yolo(self, weight_path):
        self.model=None; self.mode='模拟检测'
        if not weight_path: return False
        try:
            from pathlib import Path
            if not Path(weight_path).exists(): return False
            from ultralytics import YOLO
            self.model=YOLO(weight_path); self.mode='YOLO检测'; return True
        except Exception:
            return False
    def detect(self, image, roi_second=True):
        if self.model is not None: return self._detect_yolo(image)
        return self._detect_mock(image, roi_second)
    def _detect_yolo(self, image):
        results=self.model(image, conf=self.confidence, verbose=False)[0]; dets=[]
        names=results.names
        for b in results.boxes:
            x1,y1,x2,y2=map(int,b.xyxy[0].tolist()); conf=float(b.conf[0]); cls=int(b.cls[0])
            if conf>=self.confidence: dets.append(DetectionResult(names.get(cls,str(cls)),conf,(x1,y1,x2,y2),((x1+x2)//2,(y1+y2)//2)))
        return dets
    def _detect_mock(self, image, roi_second=True):
        # 通过示例图的蓝橙膜库组件和绿色夹具区域模拟ROI二级识别。
        h,w=image.shape[:2]; dets=[]
        dets.append(DetectionResult('membrane_component',0.92,(int(w*.2),int(h*.25),int(w*.8),int(h*.75)),(w//2,h//2)))
        if roi_second:
            dets.append(DetectionResult('clamp',0.88,(int(w*.445),int(h*.46),int(w*.58),int(h*.63)),(int(w*.512),int(h*.545))))
            dets.append(DetectionResult('grasp_point',0.86,(int(w*.50)-8,int(h*.54)-8,int(w*.50)+8,int(h*.54)+8),(int(w*.50),int(h*.54))))
        return [d for d in dets if d.confidence>=self.confidence]
    def draw_results(self, image, detections):
        out=image.copy(); colors={'membrane_component':(255,120,20),'clamp':(0,180,0),'grasp_point':(0,0,255)}
        for d in detections:
            c=colors.get(d.class_name,(0,255,255)); x1,y1,x2,y2=d.bbox; cx,cy=d.center
            cv2.rectangle(out,(x1,y1),(x2,y2),c,2); cv2.circle(out,(cx,cy),5,(0,0,255),-1)
            cv2.putText(out,f'{d.class_name} {d.confidence:.2f}',(x1,max(20,y1-8)),cv2.FONT_HERSHEY_SIMPLEX,.55,c,2)
        return out
