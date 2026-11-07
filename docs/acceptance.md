# 验收记录

本记录区分平台、接口和算法识别效果。以下均为已运行的验证；当前没有真实感知算法精度验收。生成报告留在本机，不进入Git。

## 当前基线

- 环境：Ubuntu 22.04.5、ROS 2 Humble、Gazebo Sim 8.15.0；GUI检查使用本机X11。
- 四个包构建通过：baja_sim_interfaces、baja_gz_bridge、baja_perception_sim、baja_algorithm_template。
- 当前24项pytest通过，覆盖场景复现、通行间隙、高速参数、点云转换、误检扣分、边界带背景、多画/漏画/左右颠倒边线、ROI裁剪及采样密度独立性。旧16项记录属于此前基线。
- 真实传感器输出：16×2000 XYZIR点云、lidar_link；1280×720 rgb8相机图像、camera_optical_frame，fx=640。
- 模板能通过 `--algorithm baja_algorithm_template` 加载；输出空结果，出现有效目标时得到零召回，未用真值伪造识别结果。
- 评分JSON、ROS摘要、评分Marker和 `./score.sh` 均实际接通；没有算法时显示等待。

## 当前场景的整圈运行

seed=42，默认随机障碍与特殊路面保留；最大/平均误差为到离散中心线采样点的距离，不是连续曲线精确误差。

| 速度 | 仿真时长 | 实际行驶距离 | 最大/平均距离 | 报告 |
|---|---|---|---|---|
| 1m/s | 170s | 169.73m | 0.197m / 0.095m | generated/validation_speed1/report.json |
| 2m/s | 90s | 179.11m | 0.197m / 0.092m | generated/validation_speed2/report.json |
| 3m/s | 60s | 177.94m | 0.196m / 0.092m | generated/validation_speed3/report.json |

均超过152.83m的一圈，全部运行检查通过，未触发偏离停止保护。当前默认1m/s，上限3m/s。测试不代表所有seed或接入真实算法后的负载已经验证。

传感器有名义频率和监测接收频率之分。1m/s验收监测到约9.51Hz点云、25个算法缺帧，报告原样保留，不宣称零丢帧。

## RViz交互

- DEBUG配置不包含真值对象/边线显示，使用base_link；EVALUATION使用map并显示真值和评分。
- 两个窗口均默认MoveCamera＋Orbit，实际执行左键旋转、滚轮缩放、中键平移。
- 保存临时配置并比较Yaw、Distance、Focal Point，两个视图的三项交互检查均通过。
- 报告为 `generated/rviz_controls/report.json`；不据此声称其他平台和全部GUI布局均已验收。

## 复验命令

```bash
./build.sh
./test.sh
source install/setup.bash
python3 scripts/validate_live.py --algorithm baja_algorithm_template --sim-seconds 170 --output generated/validation_speed1
./scripts/validate_speeds.sh
python3 scripts/validate_rviz_controls.py
```

实时测试需要本地ROS/Gazebo通信权限和可用渲染环境。最后一项会操作其自己创建的临时RViz窗口。默认生成目录会被重复运行覆盖。

## 未验收与暂缓

- 尚未验收：真实算法精度/实时负载、全部随机布局、多平台GPU和其他GUI操作。
- 后续先实现：从传感器真正输出边线和三维框的示范算法；当前模板仍为空结果。
- 不在本轮范围：独立二维与特殊路面区域评分、动态灯色和数字测试集。
- 已确定：边界锥桶不是目标，报成障碍计FP；边线按几何F1评价。停车位已删除。
- 用户已暂缓：实车反射率、运动畸变、相机成像、悬架等真实性增强，不作为近期交付要求。

## 两任务收敛后的复验

- 生成器测试确认新SDF没有停车位模型，scene.json没有车位区域标签。
- 误检案例：10TP＋1FP为95.24分，误检扣4.76分；背景带预测不能误匹配附近真目标。
- 边线案例验证准确线满分、空线零分、漏画降覆盖率、多画错线降精确率、左右颠倒零分；不要求检测锥桶。
- 修改后以1m/s实际运行25秒，233帧点云、494帧图像、240个已评分帧，12项运行检查通过；所有进程正常退出。报告为generated/two_task_check/report.json。
- 本次记录2个算法缺帧和8个真值不完整跳过帧，未把真值缺失计为算法漏检。空模板两题均为0分，符合预期。
- 2/3m/s完整圈数据为此前场景版本实测，本次删除停车位后只做了上述短时运行和生成器/评分测试，不声称重新跑过高速整圈。
