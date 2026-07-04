from PySide6.QtWidgets import QWidget,QVBoxLayout
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
class RobotViewPanel(QWidget):
    def __init__(self):
        super().__init__(); lay=QVBoxLayout(self); self.fig=Figure(figsize=(5,4)); self.canvas=FigureCanvas(self.fig); lay.addWidget(self.canvas)
        self.target=None; self.pre=None; self.path=[]; self.gripper=False
    def draw(self,points):
        self.fig.clear(); ax=self.fig.add_subplot(111,projection='3d'); xs=[p[0] for p in points]; ys=[p[1] for p in points]; zs=[p[2] for p in points]
        ax.plot(xs,ys,zs,'-o',lw=4,color='#1f77b4'); ax.plot([-400,400,400,-400,-400],[-400,-400,400,400,-400],[0,0,0,0,0],color='gray')
        if self.target: ax.scatter(*self.target,c='red',s=60,label='目标点')
        if self.pre: ax.scatter(*self.pre,c='orange',s=50,label='预抓取点')
        if self.path: ax.plot([p[0] for p in self.path],[p[1] for p in self.path],[p[2] for p in self.path],'g--',label='抓取路径')
        ax.text(xs[-1],ys[-1],zs[-1], '夹爪闭合' if self.gripper else '夹爪打开')
        ax.set_xlim(-650,650); ax.set_ylim(-650,650); ax.set_zlim(0,700); ax.set_xlabel('X/mm'); ax.set_ylabel('Y/mm'); ax.set_zlabel('Z/mm'); ax.legend(loc='upper left')
        self.canvas.draw_idle()
