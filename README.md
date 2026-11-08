# 感知算法练习与仿真评测

在Gazebo闭环道路中完成两项任务：三维目标检测、道路左右边线估计。车辆由平台循迹行驶，算法读取传感器数据，按统一ROS接口输出结果，平台与真值比较评分。

| 文档 | 内容 |
|---|---|
| [题目要求](docs/tasks.md) | 检测对象、边线定义、作答限制和提交内容 |
| [输入输出接口](docs/interfaces.md) | ROS话题、消息类型、坐标、时间戳与类别 |
| [评分依据](docs/evaluation.md) | 有效目标、匹配、误检/漏检扣分、边线几何分 |

## 环境与运行

需要Ubuntu 22.04、ROS 2 Humble、Gazebo Harmonic及兼容的ros_gz桥、vision_msgs、rviz2、NumPy、SciPy、PyYAML。原生点云桥构建需要gz-transport13、gz-msgs10开发库；运行验收保存JPEG需要Pillow。无GUI模式也需要可用的EGL/OpenGL渲染环境。

在仓库根目录执行：

```bash
./build.sh
./run.sh
```

默认打开Gazebo和两个RViz窗口：DEBUG只显示传感器与算法结果；EVALUATION显示真值对照和评分。算法未输出时显示等待。传感器与场景均由本地资源生成，不需要在线下载模型。

```bash
./run.sh --headless                         # 无GUI，仍计算传感器
./run.sh --rviz-mode debug                  # 只打开无真值RViz
./run.sh --rviz-mode evaluation             # 只打开评测RViz
./run.sh --no-rviz                          # 不启动RViz
./run.sh --seed 7 --obstacles 40             # 随机布局可复现
./run.sh --speed 2                          # 单次车速覆盖
./run.sh --algorithm baja_algorithm_template # 接入算法包
./run.sh --example                          # 上一条的快捷方式
```

模板只发布空结果，不包含识别算法。实现算法后使用自己的包名；每个结果话题只能有一个算法发布者，--example与--algorithm不能同时使用。

## 算法放置与启动

```text
src/algorithms/<算法包名>/
├── package.xml
├── launch/perception.launch.py   # 必须安装的统一启动入口
├── <Python模块或C++源码>
└── <参数和模型文件，按需创建并声明安装>
```

可以直接修改 [模板node.py](src/algorithms/baja_algorithm_template/baja_algorithm_template/node.py) 的process_cloud()，也可增加独立Python/C++ ROS 2包。根目录./build.sh会递归构建src下的包。算法包需安装launch/perception.launch.py，在其中启动算法、设置use_sim_time=true、完成输入输出话题remap；使用 `./run.sh --algorithm <算法包名>` 加载。

也可先启动仿真，再在另一个终端source install/setup.bash后独立启动算法。权重文件按需放入算法包或使用外部参数路径，不要求提交数据集或大rosbag。

## 场景与参数

配置：[scenario.yaml](src/baja_perception_sim/config/scenario.yaml)。

| 项目 | 默认值 |
|---|---|
| 道路 | 152.83m闭环、宽12m、转弯中心半径10m |
| 边界物标 | 左蓝右红锥桶，高0.30m，中心间距约0.50m |
| 车辆 | 1m/s，沿中心线循迹；默认最高3m/s |
| 雷达 | 名义离地1.5m，16线、360度、垂直−15至+15度 |
| 点云 | 10Hz名义频率，16×2000采样位置/帧，XYZIR |
| 相机 | 前视1280×720，20Hz名义频率，水平视场90度 |
| 随机目标 | 32个箱、桶、目标锥桶、人形，避开通行区域 |
| 其他场景 | 固定红灯、限速20、斑马线、起伏/坑/凸起；无停车位 |

雷达最下方束与平地交点约5.6m，这是安装高度和角度形成的几何盲区，不是5.6m以内全删点。有效返回数随遮挡和场景变化，不保证每帧有32000个有效点。

永久车速为vehicle.speed，默认1.0m/s；上限为vehicle.max_speed，默认3.0m/s。命令行--speed优先，改配置后重启即可。障碍生成检查整条轨迹的占地间隙；车辆偏离中心线超过0.65m或定位过期会停止。

## 调试与查看评分

在右侧三维区域使用Move Camera：左键旋转、中键或Shift＋左键平移、滚轮缩放；Views中的Reset view恢复初始视角。独立执行./debug_rviz.sh可为运行中的仿真增加调试窗口。

```bash
./score.sh                                      # 每秒刷新
./score.sh --once                               # 单次显示
./score.sh --run-dir generated/my_run            # 对应自定义运行目录
```

还可查看EVALUATION RViz的LIVE SCORE、ROS `/evaluation/summary` 或文件 `generated/current/results/summary.json`。三维检出分、边线几何分分别报告，语义分类另报，没有加权总分。精确规则以 [评分依据](docs/evaluation.md) 为准。

## 测试与短录制

```bash
./test.sh
source install/setup.bash
ros2 run baja_perception_sim inspect_sensors
python3 scripts/validate_live.py --algorithm baja_algorithm_template --sim-seconds 170
./scripts/validate_speeds.sh
```

inspect_sensors连接已有仿真；validate_live自己启动并清理隔离仿真，报告与单帧样本写入generated/validation。validate_speeds复用它检查2/3m/s。可选 `python3 scripts/validate_rviz_controls.py` 在X11中操作其创建的临时窗口检查视角控制。

可选 `./scripts/record_short.sh` 默认只录2秒点云与TF，允许1～5秒，不默认生成大包。回放时先关闭实时仿真，再执行 `ros2 bag play recordings/<目录> --clock`。该短录制用于输入兼容性调试，不含评分真值，不能单独完成离线评分。

## 目录

- src/algorithms：组员算法和模板。
- src/baja_perception_sim：场景、循迹、真值、评分及显示。
- src/baja_gz_bridge：C++点云桥；src/baja_sim_interfaces：消息定义。
- scripts、tests：运行检查与自动测试；examples：消息结构样例。
- generated、recordings、build、install、log：本机产物，不入Git。

ROS与Gazebo需要本地通信权限。多实例使用不同GZ_PARTITION、ROS_DOMAIN_ID和--run-dir。本项目提供仿真练习与在线评分，不把仿真分数直接视为实车部署认证。
