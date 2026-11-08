# 算法接入约定 v0.2

目标：算法独立 ROS 2 节点，启动后订阅传感器并发布结果，无需读取场景文件或调用 Gazebo 服务。`use_sim_time=true`，不订阅 `/ground_truth/*`；真值话题隔离仅为约定，不是访问控制。

接入前需同时约定**消息类型、类别字典、几何含义、坐标系、时间戳和空结果行为**。只返回类别字符串、日志或已画框图片不能直接参与当前三维评分。

| 算法输出 | ROS类型 | 最小必需内容 | 当前能否评分 |
|---|---|---|---|
| 三维目标 | vision_msgs/Detection3DArray | 时间戳、base_link、类别、置信度、框中心/姿态/尺寸 | 能，检出与分类分开 |
| 道路边线 | baja_sim_interfaces/RoadBoundaryArray | 时间戳、base_link、左右属性、置信度、有序三维点 | 能 |

当前只要求上面两项任务的输出。相机可作为三维算法的辅助输入；不要求额外提交二维框。二维及路面区域类型仅为历史扩展，不是本轮待交付任务。消息字段兼容v0.1，评分JSON升级为schema_version=2。

当前数值门限是基线，正式验收规则仍需确认，见 [evaluation.md](evaluation.md)。

## 输入

| 话题 | 类型 | 坐标系 | QoS |
|---|---|---|---|
| `/sensors/lidar/points` | `sensor_msgs/PointCloud2` | `lidar_link` | Best Effort, Keep Last 5 |
| `/sensors/front_camera/image_raw` | `sensor_msgs/Image` | `camera_optical_frame` | 订阅建议 Best Effort |
| `/sensors/front_camera/camera_info` | `sensor_msgs/CameraInfo` | `camera_optical_frame` | 订阅建议 Best Effort |
| `/tf` | `tf2_msgs/TFMessage` | map→base_link | TF 标准 |
| `/tf_static` | `tf2_msgs/TFMessage` | 车体→传感器 | TF 标准，Transient Local |
| `/clock` | `rosgraph_msgs/Clock` | — | 标准 |

点云为有组织的 height=16 / width=2000，布局小端：x/y/z/intensity 为 float32，offset=0/4/8/12；ring 为 uint16，offset=16，point_step=20。无回波位置保留非有限值，`is_dense=false`。自行滤除非有限值；不可认为有效点数恒定为 32,000。

ring=0..15 表示从最低到最高的线束顺序，角度 −15°、−13°、…、+15°。它不是供应商数据包的发射次序。单帧为瞬时整圈采样，没有旋转扫描畸变或逐点时间；intensity 是 Gazebo 值，未标定为实物反射强度。

名义平地安装：base_link 高约 0.457m，lidar_link 相对 base_link 为 (0,0,1.043)m，camera_link 为 (1.25,0,0.743)m。真实车体随地形运动，实际变换以 TF 为准。

## 三维检测（当前评分接口）

话题 `/perception/objects_3d`，`vision_msgs/Detection3DArray`，Reliable / Keep Last 10。

外层与每个 Detection3D 的 header 均使用输入采集时间戳、`frame_id=base_link`。每个目标必须有：

- `bbox.center`：框几何中心，不是底面中心；位置单位米，朝向为归一化四元数。
- `bbox.size`：框本地 x/y/z 三轴的完整边长，均为正。
- `results[0].hypothesis.class_id`：类别；v0.1 只评价第一项。
- `results[0].hypothesis.score`：0..1；无意义的默认值应避免。
- `results[0].pose.pose` 建议与 bbox.center 一致，当前评分采用 bbox。
- `id`：可选的算法跟踪 ID，不能使用真值 ID；当前不评价跟踪。

基础类别：`box`, `barrel`, `cone`, `person`, `unknown_obstacle`。交通设施暂用 `traffic_light_red`、`speed_limit_20`，将状态/数字编码为明确类别；未来改属性消息须提升接口版本。

| class_id | 含义 |
|---|---|
| box | 随机箱体 |
| barrel | 随机桶 |
| cone | 随机目标锥桶；路边蓝红边界锥桶是背景，报成障碍计误检 |
| person | 人形目标 |
| unknown_obstacle | 仅判断为障碍、未分类；可参加检出分，不能视作类别正确 |
| traffic_light_red | 当前固定红灯目标 |
| speed_limit_20 | 当前限速20目标 |

类别名称需完全一致。评测器不自动转换其他非空名称，名称不一致时分类不能命中。仅有几何检测能力的算法可用unknown_obstacle，并明确只比较检出分，不与语义算法混比分类分。

base_link：x前、y左、z上；目标位置必须在**采集时刻**的车体坐标下。结果发布延迟不改变 header.stamp。未检测到目标也发布 `detections=[]`，一帧一条结果。

示例结构：

可复制的独立文件见 [box_result.yaml](../examples/box_result.yaml)。其中固定时间戳仅用于说明结构，实际输出须使用当前输入帧时间。

```yaml
header:
  stamp: {sec: 12, nanosec: 0}
  frame_id: base_link
detections:
  - header:
      stamp: {sec: 12, nanosec: 0}
      frame_id: base_link
    results:
      - hypothesis: {class_id: box, score: 0.92}
    bbox:
      center:
        position: {x: 8.0, y: 3.0, z: 0.4}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      size: {x: 0.8, y: 0.6, z: 0.8}
    id: ""
```

融合算法以雷达采集时刻为参考输出；图像同步方法及容差随算法配置记录。v0.1 评分精确匹配点云时间戳，不替算法进行近似时间匹配。仅相机三维算法若要参与本评分，也需要输出到雷达帧参考时刻；独立相机评分后续扩展。

## 道路边线（当前评分接口）

话题 `/perception/road_boundaries`，`baja_sim_interfaces/RoadBoundaryArray`，Reliable / Keep Last 10。header 同雷达参考时刻，base_link。

每段 RoadBoundary：side=LEFT(1) / RIGHT(2)，confidence=0..1，points 为地面三维点，至少 2 个，按曲线顺序排列。不知道或看不清的一段留空隙，用多段消息表达；不要跨越空白强行连接。整帧没有结果发布空数组。评分只比较左右边线的水平几何，不要求提交锥桶检测。

边界定义为沿道路锥桶**底面中心**的曲线。内部可以用任意模型拟合，输出统一采样成折线即可。

## 保留扩展（不属于本轮任务）

- `/perception/objects_2d`：`vision_msgs/Detection2DArray`，图像原始时间戳 / camera_optical_frame，框单位像素。像素原点左上，u右v下；不可把像素坐标放进三维框。
- `/perception/surface_regions`：`baja_sim_interfaces/SurfaceRegionArray`，base_link。类型 `sine_wave / crater / speed_bump / crosswalk`；polygon 为有序轮廓，首尾隐式闭合；height 为估计凸起正高度/凹陷负深度，未知用 NaN。正弦区域height为峰谷差。不要求算法实现该扩展。

## 开发流程

1. 把算法放在 `src/algorithms/`；可直接修改 [模板 node.py](../src/algorithms/baja_algorithm_template/baja_algorithm_template/node.py)，目录与启动规则见 [算法目录说明](../src/algorithms/README.md)。
2. 执行 `./build.sh`，用 `./run.sh --algorithm baja_algorithm_template`（或自己的包名）一起启动。另一终端独立启动时先 source 工作空间。
3. 在无真值 DEBUG RViz 中观察原始三维点云、`/perception/markers` 与 `/perception/boundary_markers`；在 EVALUATION RViz 或 `./score.sh` 中查看分数。
4. 不合法的 header、置信度、四元数、尺寸会增加 `invalid_messages`，对应输入帧超时后记缺帧。

本仓库例子是接口骨架，不包含识别算法。输入回放可用短录制；无需下载大样本。启动真实算法时关闭 `--example`，每个输出话题只保留一个算法发布者。

DEBUG 视图以 base_link 为固定坐标，不显示全局真值位置、真值框或真值边线。后台评分器仍使用真值；这不是 ROS 话题访问控制。
