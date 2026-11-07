# 验收记录

本记录区分平台、接口和算法识别效果。以下均为已运行的验证；当前没有真实感知算法精度验收。生成报告留在本机，不进入Git。

## 当前基线

- 环境：Ubuntu 22.04.5、ROS 2 Humble、Gazebo Sim 8.15.0；GUI检查使用本机X11。
- 四个包构建通过：baja_sim_interfaces、baja_gz_bridge、baja_perception_sim、baja_algorithm_template。
- 16项pytest通过，覆盖场景复现、通行间隙、高速参数与转向、姿态插值、点云转换、匹配计分、调试视图和评分显示。
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
- 尚未实现：二维与特殊路面评分、动态灯色和数字测试集；具体任务见handoff.md。
- 当前评分存在边界锥桶未纳入目标也未ignore的问题，正式算法比较前需约定并解决。
- 用户已暂缓：实车反射率、运动畸变、相机成像、悬架等真实性增强，不作为近期交付要求。
