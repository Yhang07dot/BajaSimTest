# Baja 感知算法仿真测试平台

Ubuntu 22.04 / ROS 2 Humble / Gazebo Harmonic。车辆沿固定闭环低速行驶，算法通过 ROS 消息读取雷达、前视图像，输出检测结果，由平台显示并与随机场景真值比较。

本仓库为独立 Git 仓库；不依赖 `/home/yaoh/baja_cloud_sim-2.7` 运行，不自动下载在线模型，不默认录制 rosbag。

功能是否完成、哪些仍待扩展，见 [功能清单](docs/features.md)。

## 文档入口与当前范围

| 文档 | 用途 |
|---|---|
| [功能与任务状态](docs/features.md) | 区分已实现、已验证、未完成和暂缓 |
| [算法输入输出](docs/interfaces.md) | 消息类型、类别、坐标、时间戳和空结果约定 |
| [当前评分规则](docs/evaluation.md) | 对照代码可复算的基线，正式门限仍需确认 |
| [车速与视角](docs/controls.md) | 配置、启动参数、RViz鼠标操作 |
| [后续会话交接](docs/handoff.md) | 后续任务顺序、文件范围与验收要求 |
| [验收记录](docs/acceptance.md) | 已实际跑过的构建、测试、速度和GUI检查 |

本阶段聚焦算法接入与评分，**真实性增强暂缓**。停车位只有已有几何和区域真值，不在当前专用评分中；是否扩展尚未决定。后续任务可以交给独立会话，但需先统一评分规则和输出契约。

## 快速运行

```bash
cd /home/yaoh/BajaSimTest/baja_perception_sim
./build.sh
./run.sh
```

启动 Gazebo 和两个 RViz2 窗口：

- **DEBUG**：以车辆为中心显示原始三维点云、相机画面、算法框与算法边线，不配置真值对象或真值边线。
- **EVALUATION**：场景总览、绿色真值框、橙色算法框、真值/算法边线，以及白色 **LIVE SCORE** 评分文字。

没有接入算法时显示“等待算法”，不把真值当成识别结果。默认车速 **1.0m/s**，比原来快一倍。

```bash
./run.sh --headless                  # 无 GUI，仍计算传感器
./run.sh --seed 7 --obstacles 40      # 改变随机布局
./run.sh --example                   # 启动最小算法接入示例
./run.sh --no-rviz                   # 只开 Gazebo
./run.sh --rviz-mode debug           # 只开无真值的调试 RViz
./run.sh --rviz-mode evaluation      # 只开真值对照与评分 RViz
./run.sh --speed 2                   # 临时调速，默认上限3m/s
./run.sh --speed 3                   # 已完成当前场景的3m/s整圈测试
./run.sh --algorithm baja_algorithm_template  # 加载算法工作区模板
```

`--example` 仅演示订阅点云和发布合法空结果，不包含识别算法。出现目标时示例召回率为 0，这也是验证评分管道的方式。实际算法接入时不要开启该开关，避免同话题多个发布者。

第二个终端：

```bash
source install/setup.bash
ros2 run baja_perception_sim inspect_sensors
ros2 topic echo /simulation/status
ros2 topic echo /evaluation/summary
./score.sh                           # 易读的实时评分，无需 source ROS
```

已经开着仿真时，执行 `./debug_rviz.sh` 可以单独增加一个无真值的调试窗口。

永久车速在 `src/baja_perception_sim/config/scenario.yaml` 的 `vehicle.speed`，默认1.0；`vehicle.max_speed`默认3.0。命令行 `--speed` 优先。修改后重启即可，无需重新编译。

两个RViz现在默认启用 **Move Camera＋Orbit**：在右侧三维区域左键拖动旋转，中键/Shift＋左键平移，滚轮缩放。Views中有 `Reset view` 初始视角。旧窗口需要重新加载配置或重启。详见 [车速与视角操作](docs/controls.md)。

## 算法放在哪里

放到 **`src/algorithms/`**。已经提供能构建、能由平台启动的 `baja_algorithm_template` 包，在 [node.py](src/algorithms/baja_algorithm_template/baja_algorithm_template/node.py) 的 `process_cloud()` 中替换空检测即可。

```text
src/algorithms/
├── README.md                         放置和接入步骤
├── baja_algorithm_template/           可直接修改的算法模板
│   ├── baja_algorithm_template/node.py 识别逻辑
│   ├── launch/perception.launch.py    统一启动入口
│   └── models/                       权重文件位置说明
└── <团队自己的 ROS 2 包>/             Python/C++ 均可
```

放入包后执行 `./build.sh`，再运行 `./run.sh --algorithm <包名>`。自有包需安装 `launch/perception.launch.py`；可用模板替换，也可在另一个终端独立启动已有算法。

完整说明见 [算法目录说明](src/algorithms/README.md) 和 [输入输出约定](docs/interfaces.md)。模板与 `--example` 均只演示消息，尚不包含识别模型，二者不能同时启动。

## 默认场景

配置文件：[scenario.yaml](src/baja_perception_sim/config/scenario.yaml)。

| 项目 | 默认配置 |
|---|---|
| 环线 | 152.83m，宽 12m，转弯中心半径 10m |
| 运动 | 1.0m/s，物理车辆、真值定位、Pure Pursuit 循迹 |
| 边界 | 沿行驶方向左蓝右红；锥桶高 30cm、中心间距约 50cm |
| 雷达 | 名义离平地 1.5m，16 线，−15°..+15°，10Hz |
| 采样 | 2000×16；32,000 采样位置/帧；有效点数随环境变化 |
| 地面盲区 | 水平平地约 5.6m；没有设置 5.6m 硬距离截断 |
| 相机 | 前视，1280×720，20Hz，90° 水平视场 |
| 随机障碍 | 32 个箱体、桶、锥桶、人形，避开中央通行范围 |
| 特殊路面 | 正弦起伏、真实凹坑、横向半椭圆凸起 |
| 交通目标 | 斑马线、锥桶停车位、固定红灯、限速 20 标志 |

障碍物用整个占地的外接圆检查与**整条中心线**的距离，默认最小净距 2.4m，覆盖车辆半对角线及循迹余量；放不下时生成器报错。运行中定位过期或偏离中心线超过 0.65m 时停止下发运动速度。此保证针对配置中的车辆尺寸和静态障碍；改变车辆或路线后应重新运行验收。

随机对象、模型及评分标签来自同一份生成记录：`generated/current/scene/scene.json`。所有模型本地生成，轻量人形是几何模型，适合接口/几何测试，不能代表真实视觉数据分布。

## 评测与输出

评分有三个查看入口：

1. 根目录运行 **`./score.sh`**：每秒刷新检出/分类分数、Precision/Recall/F1、边线覆盖率、缺帧和错误消息。`./score.sh --once` 只显示一次。
2. **EVALUATION RViz** 的 **LIVE SCORE**：白色文字显示实时分数和等待状态，话题 `/evaluation/score_marker`。
3. 文件 **`generated/current/results/summary.json`**，以及 ROS 话题 `/evaluation/summary`。

`--run-dir` 自定义结果目录时，评分查看也使用 `./score.sh --run-dir <同一目录>`。停止仿真后文件保留，查看器会标记为离线报告。

当前实现：

以下是当前实现的基线，不代表验收门限已最终确认；检出F1、分类F1和边线指标分别报告，没有覆盖所有任务的综合总分。

- 障碍检出和类别识别分别计算 Precision / Recall / F1，F1×100 为对应分数。
- 按采集时间匹配真值；一对一中心距离匹配，重复框计误检。
- 仅对前方 ROI 内、确实有雷达返回的随机目标/交通设施评分；未返回目标单独忽略。边界锥桶不纳入障碍 TP/FN，另测边界。
- 中心和尺寸误差、仿真时间下的结果延迟、缺帧及不合法消息统计。
- 边界比较到有限折线段的距离，报告 0.5m 内覆盖率和已覆盖点平均误差。

正式横向比较前需先决定边界锥桶的处理：当前未加入障碍真值，也没有ignore规则，因此预测为障碍可能计误检。该问题已列入 [交接待确认项](docs/handoff.md)。

详细范围与限制见 [评测规则](docs/evaluation.md)。当前不提供 AP、三维 IoU、二维图像或特殊路面区域评分；相应算法接口与路面真值已预留。交通灯为固定红灯，限速牌为固定 20，尚无状态随机化。

## 小示例和短录制

实时 `--example` 已足够验证接入，无需 rosbag。需要交换小样本时：

```bash
source install/setup.bash
./scripts/record_short.sh             # 只录点云/TF，默认 2 秒
```

脚本限制 1..5 秒，不包含原始图像、真值和大地图；点云名义约 6.4MB/s，2 秒约 13MB 加少量消息开销。录制物默认被 Git 忽略。

回放时先关闭实时仿真；`ros2 bag play recordings/<目录> --clock`，然后以 `use_sim_time:=true` 启动算法。时钟由回放工具生成，避免同时录制并重放两路时钟。本录制不包含评分真值，主要用于输入兼容性调试。

## 验证

```bash
./test.sh
source install/setup.bash
python3 scripts/validate_live.py --sim-seconds 110
./scripts/validate_speeds.sh          # 分别验证2m/s和3m/s，结果分目录保存
```

实时验收使用独立 Gazebo partition / ROS domain，启动并清理自己的仿真，输出传感器、轨迹与评分检查到 `generated/validation/report.json`；另存单张相机 JPEG、单帧压缩点云和轨迹，不生成 rosbag。按新默认 1.0m/s，整圈验证可用 `python3 scripts/validate_live.py --algorithm baja_algorithm_template --sim-seconds 170 --output generated/validation_speed1`。

## 依赖与目录

需要现有 ROS 2 Humble、Gazebo Harmonic、兼容的 `ros_gz`（本机为 `ros-humble-ros-gzharmonic-*`）、`vision_msgs`、`rviz2`、NumPy / SciPy / PyYAML，以及 `gz-transport13` / `gz-msgs10` 开发库以构建原生点云桥。测试的参考转换器另使用 `gz.msgs10` Python 消息；验收保存 JPEG 需 Pillow。不要混装冲突的 Fortress/Garden/Harmonic 桥接包。

```text
src/baja_sim_interfaces/      边线、路面区域消息
src/baja_gz_bridge/           C++ 原生点云桥接
src/baja_perception_sim/      场景、车辆、传感器桥接、循迹和评测
src/algorithms/               算法团队放置 ROS 包的位置和可运行模板
docs/                        算法接口和评测规则
scripts/                     小样本录制和实际运行验收
tests/                       场景安全、消息转换与评分测试
generated/                   本机生成场景与运行结果（不入库）
```

Gazebo 使用 GPU 渲染传感器，无界面运行也需要可用的 EGL/OpenGL 环境。ROS 与 Gazebo 需要本地网络通信权限。启动默认 Gazebo partition 为 `baja_perception`；多实例运行需要分别配置 `GZ_PARTITION`、`ROS_DOMAIN_ID` 和 `--run-dir`。

车辆模板改编自用户提供的 `baja_cloud_sim-2.7` 车辆模型；传感器与平台代码在此仓库独立维护。外部资料见 [设计记录](docs/design.md)。
