from PySide6.QtWidgets import QWidget,QVBoxLayout,QGridLayout,QSlider,QLineEdit,QPushButton,QLabel
from PySide6.QtCore import Qt,Signal
class ControlPanel(QWidget):
    joints_changed=Signal(list); movej=Signal(); movel=Signal(); auto_grasp=Signal(); reset=Signal(); export_log=Signal(); exit_app=Signal(); jog=Signal(str,float)
    def __init__(self):
        super().__init__(); lay=QVBoxLayout(self); self.sliders=[]; self.edits=[]; grid=QGridLayout()
        for i in range(6):
            s=QSlider(Qt.Horizontal); s.setRange(-180,180); e=QLineEdit('0'); e.setFixedWidth(55); self.sliders.append(s); self.edits.append(e)
            grid.addWidget(QLabel(f'J{i+1}'),i,0); grid.addWidget(s,i,1); grid.addWidget(e,i,2); s.valueChanged.connect(self._emit)
        lay.addLayout(grid); self.pose=[QLineEdit('0.00') for _ in range(6)]; pg=QGridLayout(); names=['X','Y','Z','r','p','y']
        for i,n in enumerate(names): pg.addWidget(QLabel(n),i//3*2,i%3); pg.addWidget(self.pose[i],i//3*2+1,i%3)
        lay.addLayout(pg); jogg=QGridLayout()
        for idx,name in enumerate(['X+','X-','Y+','Y-','Z+','Z-','r+','r-','p+','p-','y+','y-']):
            b=QPushButton(name); b.clicked.connect(lambda _,n=name:self.jog.emit(n,20.0)); jogg.addWidget(b,idx//2,idx%2)
        lay.addLayout(jogg)
        for name,sig in [('MoveJ',self.movej),('MoveL',self.movel),('自动抓取',self.auto_grasp),('复位',self.reset),('导出日志',self.export_log),('Exit',self.exit_app)]:
            b=QPushButton(name); b.clicked.connect(sig.emit); lay.addWidget(b)
    def _emit(self): self.joints_changed.emit([s.value() for s in self.sliders])
    def set_joints(self,angles):
        for s,e,a in zip(self.sliders,self.edits,angles): s.blockSignals(True); s.setValue(int(a)); s.blockSignals(False); e.setText(f'{a:.1f}')
    def set_pose(self,pose):
        for e,v in zip(self.pose,[pose.x,pose.y,pose.z,pose.r,pose.p,pose.yaw]): e.setText(f'{v:.2f}')
