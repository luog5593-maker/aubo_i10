import cv2, numpy as np
from pathlib import Path

def generate_demo_image(path=None, size=(640,480)):
    img=np.full((size[1],size[0],3),245,np.uint8)
    cv2.rectangle(img,(130,120),(510,360),(80,160,230),-1)
    cv2.putText(img,'membrane_component',(155,165),cv2.FONT_HERSHEY_SIMPLEX,0.8,(20,60,120),2)
    cv2.rectangle(img,(285,220),(370,300),(60,210,90),-1)
    cv2.circle(img,(328,260),6,(0,0,255),-1)
    cv2.putText(img,'clamp',(292,215),cv2.FONT_HERSHEY_SIMPLEX,0.7,(20,100,30),2)
    if path:
        Path(path).parent.mkdir(parents=True,exist_ok=True); cv2.imwrite(str(path),img)
    return img

def cv_to_qimage(img):
    from PySide6.QtGui import QImage
    rgb=cv2.cvtColor(img,cv2.COLOR_BGR2RGB); h,w,ch=rgb.shape
    return QImage(rgb.data,w,h,ch*w,QImage.Format_RGB888).copy()
