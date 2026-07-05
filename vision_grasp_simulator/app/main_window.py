from PySide6.QtWidgets import QMainWindow,QWidget,QVBoxLayout,QSplitter,QMessageBox
from PySide6.QtCore import Qt
from app.widgets.image_panel import ImagePanel
from app.widgets.robot_view_panel import RobotViewPanel
from app.widgets.control_panel import ControlPanel
from app.widgets.log_panel import LogPanel
from core.vision_detector import VisionDetector
from core.coordinate_transform import CoordinateTransformer
from core.robot_kinematics import RobotKinematics
from core.trajectory_planner import TrajectoryPlanner
from core.grasp_simulator import GraspSimulator
from core.safety_checker import SafetyChecker
from core.robot_model_loader import RobotModelLoader
from utils.file_manager import load_json, save_json, save_csv, timestamp_name
from utils.logger import AppLogger
from pathlib import Path

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__(); self.setWindowTitle('基于YOLO的机械臂视觉识别与抓取仿真系统 V1.0'); self.resize(1500,900)
        self.logger=AppLogger(); robot_cfg=load_json('config/robot_config.json'); self.robot=RobotKinematics(robot_cfg); vc=load_json('config/vision_config.json'); sc=load_json('config/safety_config.json')
        self.project_root=Path(__file__).resolve().parents[1]; self.visual_model=RobotModelLoader(self.project_root).load()
        self.transformer=CoordinateTransformer(vc['pixel_to_mm'],vc['origin_offset'],vc['default_grasp_height']); self.planner=TrajectoryPlanner(); self.grasp=GraspSimulator(robot_cfg); self.safety=SafetyChecker(sc['min_confidence'])
        self.image_panel=ImagePanel(); self.view=RobotViewPanel(); self.view.set_model(self.visual_model); self.view.set_gripper_config(robot_cfg.get('tool_offset_xyz',[0,0,0.08]), robot_cfg.get('finger_travel',0.03)); self.ctrl=ControlPanel(); self.log=LogPanel(); self.logger.add_listener(self.log.append)
        top=QSplitter(Qt.Horizontal); top.addWidget(self.image_panel); top.addWidget(self.view); top.addWidget(self.ctrl); top.setSizes([420,650,360])
        central=QWidget(); lay=QVBoxLayout(central); lay.addWidget(top,4); lay.addWidget(self.log,1); self.setCentralWidget(central)
        self.detections=[]; self.current_image=None; self.last_target=None; self.trajectory=[]
        self.image_panel.detect_requested.connect(self.detect); self.image_panel.image_changed.connect(lambda img:setattr(self,'current_image',img))
        self.ctrl.joints_changed.connect(self.set_joints); self.ctrl.reset.connect(self.reset); self.ctrl.exit_app.connect(self.close); self.ctrl.auto_grasp.connect(self.auto_grasp); self.ctrl.movej.connect(self.movej_home); self.ctrl.movel.connect(self.movel_target); self.ctrl.export_log.connect(self.export_logs); self.ctrl.jog.connect(self.jog)
        self._apply_urdf_joint_limits(); self.reset(); self.logger.log('启动','软件启动成功，未连接真实机械臂、ROS或深度相机。')
        for msg in self.visual_model.messages: self.logger.log('模型加载', msg)
    def _apply_urdf_joint_limits(self):
        import math
        if getattr(self, 'visual_model', None) and self.visual_model.joints:
            limits=[(math.degrees(j.lower), math.degrees(j.upper)) for j in self.visual_model.joints[:6]]
            self.ctrl.set_joint_limits(limits); self.robot.limits=limits
    def refresh(self):
        pts,pose=self.robot.forward(); self.ctrl.set_joints(self.robot.joint_angles); self.ctrl.set_pose(pose); self.view.draw(pts, self.robot.joint_angles)
    def set_joints(self,angles): self.robot.joint_angles=self.robot.clamp(angles); self.refresh()
    def reset(self): self.robot.reset(); self.view.path=[]; self.view.gripper=False; self.refresh(); self.logger.log('复位','机械臂已回到初始位姿')
    def detect(self):
        if self.current_image is None: self.image_panel.demo(); self.current_image=self.image_panel.image
        detector=VisionDetector(self.image_panel.weight.text(), self.image_panel.conf.value()); dets=detector.detect(self.current_image)
        for d in dets: d.world=self.transformer.pixel_to_world(*d.center,self.current_image.shape)
        self.detections=dets; img=detector.draw_results(self.current_image,dets); self.image_panel.set_results(img,dets)
        target=next((d for d in dets if d.class_name in ('grasp_point','clamp')), None); self.last_target=target.world if target is not None else None; self.view.target=self.last_target; self.refresh()
        self.logger.log('目标检测',f'{detector.mode}完成，目标数量{len(dets)}', result=[d.to_dict() for d in dets])
    def animate_to(self,target,gripper=False):
        angles,msg=self.robot.inverse(target)
        if angles is None: self.logger.log('运动失败',msg); return False
        for a in self.planner.movej_points(self.robot.joint_angles,angles,25):
            self.robot.joint_angles=self.robot.clamp(a); self.view.gripper=gripper; self.trajectory.append({'joint_angles':list(map(float,self.robot.joint_angles)),'target':target,'gripper':gripper}); self.refresh()
        return True
    def movej_home(self): self.animate_to((350,0,260)); self.logger.log('MoveJ','完成关节空间插值运动')
    def movel_target(self):
        if self.last_target is None: QMessageBox.warning(self,'提示','请先完成检测并获得目标点'); return
        _,pose=self.robot.forward()
        for p in self.planner.movel_targets((pose.x,pose.y,pose.z),self.last_target,20):
            if not self.animate_to(p): return
        self.logger.log('MoveL','完成末端直线插值运动')
    def jog(self,name,step):
        _,pose=self.robot.forward(); xyz=[pose.x,pose.y,pose.z]; idx='XYZ'.find(name[0])
        if idx>=0: xyz[idx]+= step if name[1]=='+' else -step; self.animate_to(tuple(xyz)); self.logger.log('位姿微调',name)
    def auto_grasp(self):
        target_det=next((d for d in self.detections if d.class_name in ('grasp_point','clamp')), None); ok,msg=self.safety.check_detection(target_det)
        if not ok: QMessageBox.warning(self,'安全提示',msg); self.logger.log('安全判断',msg); return
        ok,msg=self.safety.check_target(self.robot,target_det.world)
        if not ok: QMessageBox.warning(self,'安全提示',msg); self.logger.log('安全判断',msg); return
        self.view.target=target_det.world; self.view.pre=(target_det.world[0],target_det.world[1],target_det.world[2]+120); self.view.path=[]
        for text,point,closed in self.grasp.sequence(target_det.world):
            prompt = {'运动到预抓取点':'正在移动到预抓取点','下降到抓取点':'正在下降到抓取点','虚拟夹爪闭合':'电爪闭合','抬升目标':'抓取完成，正在抬升','移动到放置点上方':'正在移动到放置点','虚拟夹爪打开':'电爪打开','返回初始位姿':'自动抓取流程结束'}.get(text, text)
            self.logger.log('自动抓取',prompt, target=point, gripper=closed)
            if point is None: self.reset(); continue
            self.view.path.append(point)
            if not self.animate_to(point,closed): return
        self.logger.log('自动抓取','流程完成')
    def export_logs(self):
        save_csv('outputs/logs/'+timestamp_name('run_log','.csv'), self.logger.records, ['time','operation','message'])
        save_json('outputs/trajectories/'+timestamp_name('trajectory','.json'), self.trajectory)
        self.logger.log('导出','已导出CSV日志和JSON轨迹')
