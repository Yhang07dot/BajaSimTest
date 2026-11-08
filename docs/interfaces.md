# 算法输入输出接口

## 输入话题

| 话题 | 消息类型 | 坐标/用途 |
|---|---|---|
| /sensors/lidar/points | sensor_msgs/PointCloud2 | lidar_link，主要输入 |
| /sensors/front_camera/image_raw | sensor_msgs/Image | camera_optical_frame，可选输入 |
| /sensors/front_camera/camera_info | sensor_msgs/CameraInfo | 相机内参与标定尺寸 |
| /tf、/tf_static | tf2_msgs/TFMessage | 坐标变换 |
| /clock | rosgraph_msgs/Clock | 仿真时间 |

传感器订阅建议使用Best Effort、Keep Last；点云发布队列深度5。TF使用标准QoS，静态TF为Transient Local。结果发布使用Reliable、Keep Last 10。

点云有组织结构height=16、width=2000，point_step=20，小端布局：

| 字段 | 类型 | 字节偏移 |
|---|---|---|
| x/y/z | float32 | 0/4/8 |
| intensity | float32 | 12 |
| ring | uint16 | 16 |

无回波保留非有限值，is_dense=false，算法需自行过滤。ring=0至15按垂直角−15、−13、…、+15度排列，不是供应商数据包发射次序。intensity未作实物反射率标定，单帧是瞬时整圈扫描，无逐点时间。

## 坐标和时间

base_link为x前、y左、z上；camera_optical_frame为x右、y下、z前。名义平地安装为雷达离地1.5m、相机离地1.2m，实际变换以TF为准。

结果header.stamp必须来自对应点云采集时刻，不能用处理完成时间替代。结果位置属于该采集时刻的base_link。相机融合算法以雷达时刻为参考，自行完成输入同步与坐标转换；评分器按时间戳精确匹配，不自动做近似帧配对。

## 三维目标输出

话题 `/perception/objects_3d`，类型 `vision_msgs/Detection3DArray`。

外层与每个Detection3D的header均使用相同采集时间戳及frame_id=base_link。每个目标必须包含：

| 字段 | 含义 |
|---|---|
| bbox.center | 包围框几何中心位置及归一化朝向四元数，不能使用底面中心 |
| bbox.size | 框自身x/y/z三轴的完整边长，单位米，必须为正 |
| results[0].hypothesis.class_id | 统一类别字符串，只评价第一项假设 |
| results[0].hypothesis.score | 0至1置信度 |
| id | 可选的算法跟踪ID，不要求与真值一致 |

results[0].pose.pose建议与bbox.center一致，当前评分采用bbox。类别名称必须精确一致：

| class_id | 含义 |
|---|---|
| box | 随机箱体 |
| barrel | 桶 |
| cone | 随机目标锥桶；不包括路边边界锥桶 |
| person | 人形 |
| traffic_light_red | 固定红灯交通设施 |
| speed_limit_20 | 限速20交通设施 |
| unknown_obstacle | 仅判断为障碍，可参加检出分，不视作语义分类正确 |

无检测也发布detections=[]。可复制的消息结构见 [box_result.yaml](../examples/box_result.yaml)，其中固定时间戳只作形状说明，实际运行需使用输入帧时间戳。

## 道路边线输出

话题 `/perception/road_boundaries`，类型 `baja_sim_interfaces/RoadBoundaryArray`。

header与点云参考时刻一致，frame_id=base_link。每段RoadBoundary包含：

| 字段 | 要求 |
|---|---|
| side | LEFT=1，RIGHT=2 |
| confidence | 0至1 |
| points | 地面三维点，至少2个，按曲线顺序排列 |

多段边线分别放在boundaries数组中，不跨未知区间连线。无结果发布boundaries=[]。当前评分使用水平XY位置，不要求锥桶检测列表，也不评价边界高度精度。

## 使用边界

当前任务只使用上述两种结果。SurfaceRegionArray等保留消息不属于提交要求；不要求二维框。算法不能读取/订阅/ground_truth/*或场景物体文件。DEBUG仅隐藏真值显示，后台真值话题没有访问控制。

运行与包放置方法见 [README](../README.md)，检测对象与提交要求见 [tasks.md](tasks.md)，评分门限与公式见 [evaluation.md](evaluation.md)。
