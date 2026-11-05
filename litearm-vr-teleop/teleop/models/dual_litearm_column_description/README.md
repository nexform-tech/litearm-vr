# dual_litearm_column_description

双臂侧装立柱的 URDF 描述包：两根 LiteArm 7-DOF 机械臂背靠背安装在侧装立柱两侧。

## 系统配置

- **基座**：侧装立柱
- **左臂**：相对立柱基座安装于 (0, 0.055, 0)
- **右臂**：相对立柱基座安装于 (0, -0.055, 0)
- **总自由度**：14（每臂 7 关节）

## 关节名

- 左臂：`left_Joint1` … `left_Joint7`
- 右臂：`right_Joint1` … `right_Joint7`

## 包内容

| 目录 | 内容 |
|------|------|
| `urdf/` | xacro 源（`litearm_macro.xacro` 宏 + `dual_litearm_column.xacro` 侧装立柱主文件 + `dual_litearm_upright.xacro` 正装变体）+ 生成好的 `dual_litearm_column.urdf` / `dual_litearm_upright.urdf` |
| `meshes/` | 高精度 STL（RViz 用，含立柱 `column_base_link.STL`） |
| `meshes_lowpoly/` | 降采样 STL（pybullet 渲染用） |
| `launch/` | RViz / Gazebo 启动文件（ROS2 `.launch.py`） |
| `config/` | 双臂关节配置 |
| `rviz/` | RViz 配置文件 |

## 依赖

- `xacro` — 处理 xacro 文件（已由 `urdf/dual_litearm_column.urdf` 预生成，运行时直接加载 urdf）

## 用法

RViz 显示（ROS2）：

```bash
ros2 launch dual_litearm_column_description display.launch.py
```

实际遥操入口见项目根 `launch/run_teleop_webxr.py --dual`（`--dual` 加载本包）。其中
`--dual --upright` 加载**双臂正装变体**（两只独立竖直臂、无立柱，URDF 为同目录
`dual_litearm_upright.urdf`，与侧装复用同一 `litearm_macro.xacro` 与 `meshes/`）。

## 架构

xacro 宏实例化两套相同的 LiteArm 关节链（不同前缀）：
- `litearm_macro.xacro` — LiteArm 关节链的可复用宏
- `dual_litearm_column.xacro` — 主文件，包含立柱 + 实例化两臂
- `dual_litearm_upright.xacro` — 双臂正装变体：无立柱，两臂正装挂在一个轻量共同根下
