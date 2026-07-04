from PySide6.QtWidgets import QWidget,QVBoxLayout,QHBoxLayout,QPushButton,QLabel,QLineEdit,QDoubleSpinBox,QListWidget,QFileDialog,QMessageBox
from PySide6.QtGui import QPixmap
from PySide6.QtCore import Signal,Qt
import cv2
from utils.image_utils import generate_demo_image, cv_to_qimage
class ImagePanel(QWidget):
    detect_requested=Signal(); image_changed=Signal(object)
    def __init__(self):
        super().__init__(); self.image=None; self.detected_image=None
        lay=QVBoxLayout(self); row=QHBoxLayout()
        for name,fn in [('导入图像',self.load_image),('生成示例图像',self.demo),('打开摄像头',self.camera),('开始检测',lambda:self.detect_requested.emit()),('保存检测图',self.save_image)]:
            b=QPushButton(name); b.clicked.connect(fn); row.addWidget(b)
        lay.addLayout(row); self.weight=QLineEdit(); self.weight.setPlaceholderText('YOLO权重路径（为空则模拟检测）')
        sel=QPushButton('选择权重'); sel.clicked.connect(self.select_weight); lay.addWidget(self.weight); lay.addWidget(sel)
        self.conf=QDoubleSpinBox(); self.conf.setRange(0.05,0.99); self.conf.setSingleStep(0.05); self.conf.setValue(0.5); lay.addWidget(QLabel('置信度阈值')); lay.addWidget(self.conf)
        self.label=QLabel('图像显示区'); self.label.setAlignment(Qt.AlignCenter); self.label.setMinimumSize(360,270); self.label.setStyleSheet('border:1px solid #999;background:#f8f8f8'); lay.addWidget(self.label)
        self.list=QListWidget(); lay.addWidget(self.list)
    def show_img(self,img): self.label.setPixmap(QPixmap.fromImage(cv_to_qimage(img)).scaled(self.label.size(),Qt.KeepAspectRatio,Qt.SmoothTransformation))
    def load_image(self):
        p,_=QFileDialog.getOpenFileName(self,'导入图像','','Images (*.jpg *.png *.bmp)')
        if p: self.image=cv2.imread(p); self.show_img(self.image); self.image_changed.emit(self.image)
    def demo(self): self.image=generate_demo_image('sample_data/demo_image.png'); self.show_img(self.image); self.image_changed.emit(self.image)
    def camera(self):
        cap=cv2.VideoCapture(0); ok,frame=cap.read(); cap.release()
        if not ok: QMessageBox.warning(self,'提示','摄像头不可用，软件将继续使用本地图像或示例图像。'); return
        self.image=frame; self.show_img(frame); self.image_changed.emit(frame)
    def select_weight(self):
        p,_=QFileDialog.getOpenFileName(self,'选择YOLO权重','','Weights (*.pt *.onnx);;All (*)')
        if p: self.weight.setText(p)
    def set_results(self,img,dets):
        self.detected_image=img; self.show_img(img); self.list.clear()
        for d in dets: self.list.addItem(f'{d.class_name} conf={d.confidence:.2f} pixel={d.center} world={d.world}')
    def save_image(self):
        if self.detected_image is None: return
        p,_=QFileDialog.getSaveFileName(self,'保存检测图','outputs/images/detection.png','PNG (*.png)')
        if p: cv2.imwrite(p,self.detected_image)
