# 基于YOLO的机械臂视觉识别与抓取仿真系统 V1.0

本系统来源于机械臂视觉抓取实验流程，当前版本为脱离真实硬件的仿真验证版本。软件不连接真实AUBO机械臂、不依赖ROS/Gazebo/RViz/MoveIt、不调用真实深度相机或机械臂SDK，可在普通Windows电脑上运行。

## 安装与运行
```bash
pip install -r requirements.txt
python main.py
```

若未安装`ultralytics`或未提供YOLO权重，系统自动启用“模拟检测模式”。

## 完整演示流程
1. 点击“生成示例图像”。
2. 点击“开始检测”，系统输出膜库组件、夹具和抓取点。
3. 检查右侧末端位姿和中间三维机械臂模型。
4. 点击“自动抓取”，观察预抓取、下降、夹爪闭合、抬升、放置、复位流程。
5. 点击“导出日志”，保存CSV日志和JSON轨迹。

## 主要文件
- `main.py`：程序入口。
- `app/main_window.py`：主界面和模块调度。
- `app/widgets/`：图像、三维视图、控制、日志四类界面组件。
- `core/vision_detector.py`：YOLO调用与模拟检测。
- `core/coordinate_transform.py`：像素到工作台坐标转换。
- `core/robot_kinematics.py`：六自由度简化运动学。
- `core/trajectory_planner.py`：MoveJ/MoveL轨迹插值。
- `core/grasp_simulator.py`：自动抓取流程生成。
- `core/safety_checker.py`：安全判断。
