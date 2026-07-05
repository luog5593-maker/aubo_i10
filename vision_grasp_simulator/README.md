# 基于YOLO的机械臂视觉识别与抓取仿真系统 V1.0

本系统来源于机械臂视觉抓取实验流程，当前版本为脱离真实硬件的仿真验证版本。软件不连接真实AUBO机械臂、不依赖ROS/Gazebo/RViz/MoveIt、不调用真实深度相机或机械臂SDK，可在普通Windows电脑上运行。

## 安装与运行
```bash
pip install -r requirements.txt
python main.py
```

若未安装`ultralytics`或未提供YOLO权重，系统自动启用“模拟检测模式”。

## AUBO i10 与 PGC_300_60 模型资源
程序优先从项目内部 `assets/` 目录读取模型文件：

- `assets/aubo_i10/aubo_i10_gazebo_roscontrol.urdf`
- `assets/aubo_i10/aubo_i10.xacro`
- `assets/aubo_i10/meshes/`
- `assets/pgc_300_60/pgc_300_60.xacro`
- `assets/pgc_300_60/meshes/`

请手动复制本地资源：

1. 将 `aubo i10 meshes` 中的 `aubo_i10.xacro`、`aubo_i10_gazebo_roscontrol.urdf`、`meshes/` 复制到 `vision_grasp_simulator/assets/aubo_i10/`。
2. 将 `pgc_300_60` 中的 `pgc_300_60.xacro`、`meshes/` 复制到 `vision_grasp_simulator/assets/pgc_300_60/`。

若 `assets/` 中找不到模型，程序会兼容尝试读取上一级目录中的 `../aubo i10 meshes/` 与 `../pgc_300_60/`。若 DAE/STL 缺失或加载失败，三维仿真区会自动使用程序化模型替代，软件不会崩溃。

当前版本支持 AUBO i10 URDF 参数解析、DAE/STL 外观模型加载、PGC_300_60 电爪 STL 模型加载、电爪开合仿真，以及 AUBO i10 与电爪联合运动仿真。

## 完整演示流程
1. 点击“生成示例图像”。
2. 点击“开始检测”，系统输出膜库组件、夹具和抓取点。
3. 检查右侧末端位姿和中间“AUBO i10 + PGC_300_60 视觉抓取仿真区”。
4. 点击“自动抓取”，观察预抓取、下降、电爪闭合、抬升、移动到放置点、电爪打开、复位流程。
5. 点击“导出日志”，保存CSV日志和JSON轨迹。

## 主要文件
- `main.py`：程序入口。
- `app/main_window.py`：主界面和模块调度。
- `app/widgets/`：图像、三维视图、控制、日志四类界面组件。
- `core/robot_model_loader.py`：AUBO i10 URDF解析、DAE/STL mesh路径映射、PGC_300_60模型加载与程序化回退。
- `core/vision_detector.py`：YOLO调用与模拟检测。
- `core/coordinate_transform.py`：像素到工作台坐标转换。
- `core/robot_kinematics.py`：六自由度简化运动学。
- `core/trajectory_planner.py`：MoveJ/MoveL轨迹插值。
- `core/grasp_simulator.py`：自动抓取流程生成。
- `core/safety_checker.py`：安全判断。
