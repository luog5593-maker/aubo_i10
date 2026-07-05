from PySide6.QtWidgets import QWidget,QVBoxLayout,QLabel
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np, math

class RobotViewPanel(QWidget):
    """AUBO i10 + PGC_300_60 三维组合模型显示区，支持mesh与程序化模型回退。"""
    def __init__(self):
        super().__init__(); lay=QVBoxLayout(self)
        lay.addWidget(QLabel('AUBO i10 + PGC_300_60 视觉抓取仿真区'))
        lay.addWidget(QLabel('模型来源：AUBO i10 URDF + DAE/STL Mesh，PGC_300_60 STL Mesh'))
        self.fig=Figure(figsize=(5,4)); self.canvas=FigureCanvas(self.fig); lay.addWidget(self.canvas)
        self.target=None; self.pre=None; self.path=[]; self.gripper=False; self.finger_travel=0.03; self.tool_offset_xyz=[0,0,0.08]; self.model=None
    def set_model(self, model): self.model=model
    def set_gripper_config(self, tool_offset_xyz, finger_travel): self.tool_offset_xyz=tool_offset_xyz; self.finger_travel=finger_travel
    def draw(self,points=None,joint_angles=None):
        self.fig.clear(); ax=self.fig.add_subplot(111,projection='3d')
        ax.plot([-0.45,0.45,0.45,-0.45,-0.45],[-0.45,-0.45,0.45,0.45,-0.45],[0,0,0,0,0],color='gray')
        safe_angles = joint_angles if joint_angles is not None else [0,0,0,0,0,0]
        transforms = self._chain_transforms(safe_angles)
        if self.model is not None:
            self._draw_kinematic_skeleton(ax, transforms)
            for idx,name in enumerate(['base_link','shoulder_Link','upperArm_Link','foreArm_Link','wrist1_Link','wrist2_Link','wrist3_Link']):
                T = transforms.get(name, np.eye(4)); self._draw_part(ax,self.model.links.get(name),T,idx)
            self._draw_gripper(ax, transforms.get('wrist3_Link',np.eye(4)))
        elif points is not None and len(points) > 0:
            pts=np.asarray(points)/1000.0; ax.plot(pts[:,0],pts[:,1],pts[:,2],'-o',lw=4,color='#1f77b4')
        self._draw_markers(ax)
        ax.set_xlim(-0.7,0.7); ax.set_ylim(-0.7,0.7); ax.set_zlim(0,0.8); ax.set_xlabel('X/m'); ax.set_ylabel('Y/m'); ax.set_zlabel('Z/m')
        self.canvas.draw_idle()
    def _chain_transforms(self, angles_deg):
        T=np.eye(4); transforms={'base_link':T.copy()}
        if self.model is None or self.model.joints is None or len(self.model.joints) == 0: return transforms
        by_name={j.name:j for j in self.model.joints}
        for i,jname in enumerate(['shoulder_joint','upperArm_joint','foreArm_joint','wrist1_joint','wrist2_joint','wrist3_joint']):
            j=by_name.get(jname)
            if j is None: continue
            T=T @ self._translation(j.origin_xyz) @ self._rpy(j.origin_rpy) @ self._axis_rotation(j.axis_xyz, math.radians(float(angles_deg[i])))
            transforms[j.child]=T.copy()
        return transforms

    def _draw_kinematic_skeleton(self, ax, transforms):
        chain=['base_link','shoulder_Link','upperArm_Link','foreArm_Link','wrist1_Link','wrist2_Link','wrist3_Link']
        pts=np.array([transforms.get(name,np.eye(4))[:3,3] for name in chain])
        if pts.size > 0 and len(pts) > 0:
            ax.plot(pts[:,0],pts[:,1],pts[:,2],'-o',color='black',linewidth=1.2,markersize=3,alpha=0.65,label='AUBO运动链')
    def _draw_gripper(self, ax, wrist_T):
        base_T=wrist_T @ self._translation(self.tool_offset_xyz)
        open_gap=0.0 if self.gripper else self.finger_travel
        parts=[('pgc_300_60',base_T),('pgc_300_60_base',base_T),('pgc_300_60_left_finger',base_T @ self._translation((0,open_gap/2,0.07))),('pgc_300_60_right_finger',base_T @ self._translation((0,-open_gap/2,0.07)))]
        for i,(name,T) in enumerate(parts):
            info=self.model.gripper_meshes.get(name) if self.model is not None else None; self._draw_part(ax,info,T,10+i)
        p=base_T[:3,3]; ax.text(p[0],p[1],p[2]+0.08,'电爪闭合' if self.gripper else '电爪打开')
    def _draw_part(self, ax, info, T, color_idx):
        colors=['#4c78a8','#f58518','#54a24b','#e45756','#72b7b2','#b279a2','#ff9da6','#9d755d','#bab0ac','#2f4b7c','#665191','#a05195','#d45087','#ff7c43']
        if info is not None:
            sx,sy,sz=info.size
            if info.name in ('upperArm_Link','foreArm_Link'):
                T = T @ self._translation((sx/2,0,0))
            elif info.name in ('base_link','shoulder_Link'):
                T = T @ self._translation((0,0,sz/2))
            elif info.name in ('wrist1_Link','wrist2_Link','wrist3_Link'):
                T = T @ self._translation((sx/2,0,0))
        if info is not None and info.mesh_loaded and info.vertices is not None and len(info.vertices) > 0 and info.faces is not None and len(info.faces) > 0:
            verts=self._transform_vertices(info.vertices,T); faces=info.faces[:800]
            poly=Poly3DCollection(verts[faces], alpha=0.85, facecolor=colors[color_idx%len(colors)], edgecolor='none'); ax.add_collection3d(poly); return
        sx,sy,sz=(info.size if info is not None else (.08,.08,.08)); self._draw_box(ax,T,sx,sy,sz,colors[color_idx%len(colors)])
    def _draw_box(self,ax,T,sx,sy,sz,color):
        x=sx/2; y=sy/2; z=sz/2
        v=np.array([[-x,-y,-z],[x,-y,-z],[x,y,-z],[-x,y,-z],[-x,-y,z],[x,-y,z],[x,y,z],[-x,y,z]])
        faces=np.array([[0,1,2,3],[4,5,6,7],[0,1,5,4],[2,3,7,6],[1,2,6,5],[0,3,7,4]])
        vt=self._transform_vertices(v,T); ax.add_collection3d(Poly3DCollection(vt[faces], alpha=.8, facecolor=color, edgecolor='k', linewidth=.2))
    def _draw_markers(self,ax):
        if self.target is not None: ax.scatter(self.target[0]/1000,self.target[1]/1000,self.target[2]/1000,c='red',s=60,label='目标点')
        if self.pre is not None: ax.scatter(self.pre[0]/1000,self.pre[1]/1000,self.pre[2]/1000,c='orange',s=50,label='预抓取点')
        if self.path is not None and len(self.path) > 0:
            p=np.asarray(self.path)/1000.0; ax.plot(p[:,0],p[:,1],p[:,2],'g--',label='抓取路径')
        if self.target is not None or self.pre is not None or (self.path is not None and len(self.path) > 0): ax.legend(loc='upper left')
    def _transform_vertices(self,v,T):
        vh=np.c_[v,np.ones(len(v))]; return (T@vh.T).T[:,:3]
    def _translation(self,xyz):
        T=np.eye(4); T[:3,3]=np.asarray(xyz,dtype=float); return T
    def _rpy(self,rpy):
        r,p,y=rpy; cr,sr=math.cos(r),math.sin(r); cp,sp=math.cos(p),math.sin(p); cy,sy=math.cos(y),math.sin(y)
        Rx=np.array([[1,0,0,0],[0,cr,-sr,0],[0,sr,cr,0],[0,0,0,1]]); Ry=np.array([[cp,0,sp,0],[0,1,0,0],[-sp,0,cp,0],[0,0,0,1]]); Rz=np.array([[cy,-sy,0,0],[sy,cy,0,0],[0,0,1,0],[0,0,0,1]])
        return Rz@Ry@Rx
    def _axis_rotation(self,axis,theta):
        a=np.asarray(axis,dtype=float); n=np.linalg.norm(a); a=a/n if n else np.array([0,0,1.]); x,y,z=a; c=math.cos(theta); s=math.sin(theta); C=1-c
        R=np.array([[x*x*C+c,x*y*C-z*s,x*z*C+y*s,0],[y*x*C+z*s,y*y*C+c,y*z*C-x*s,0],[z*x*C-y*s,z*y*C+x*s,z*z*C+c,0],[0,0,0,1]])
        return R
