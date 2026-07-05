# assets 模型资源目录

本目录用于放置 AUBO i10 与 PGC_300_60 的外观模型资源。为避免 PR 包含大型或不受支持的二进制文件，仓库默认只提交目录说明和 `.gitkeep`。

请将本地资源复制到以下目录：

- 将 `aubo i10 meshes/aubo_i10.xacro`、`aubo i10 meshes/aubo_i10_gazebo_roscontrol.urdf` 和 `aubo i10 meshes/meshes/` 复制到 `vision_grasp_simulator/assets/aubo_i10/`。
- 将 `pgc_300_60/pgc_300_60.xacro` 和 `pgc_300_60/meshes/` 复制到 `vision_grasp_simulator/assets/pgc_300_60/`。

程序会优先读取 `assets/` 下的模型；若不存在，会兼容尝试读取项目上一级目录中的 `../aubo i10 meshes/` 和 `../pgc_300_60/`。如果 mesh 缺失或无法解析，三维视图会自动使用程序化几何体替代，不会影响软件启动。
