# 算法放在这里

这里是算法团队的工作区；仿真与评分代码位于旁边的 `baja_perception_sim`，算法不需要改它们。

```text
src/algorithms/
├── README.md
├── baja_algorithm_template/          可直接运行的 Python ROS 2 模板
│   ├── package.xml                   ROS 依赖
│   ├── setup.py / setup.cfg          安装与节点入口
│   ├── resource/baja_algorithm_template
│   ├── launch/perception.launch.py   平台统一启动入口
│   ├── models/README.md              模型权重放置说明
│   └── baja_algorithm_template/
│       ├── __init__.py
│       └── node.py                   在这里实现识别逻辑
└── your_perception_package/          也可放团队自己的 Python/C++ ROS 2 包
```

## 先跑通模板

在仓库根目录：

```bash
./build.sh
./run.sh --algorithm baja_algorithm_template
```

模板只订阅真实点云并发布空检测和空边线；它没有识别能力，不读取真值，因此出现目标时得到零召回。将识别代码写入 `baja_algorithm_template/node.py` 的 `process_cloud()`，输出给 `/perception/objects_3d` 和 `/perception/road_boundaries`。Python / C++ 或独立进程均可，最终只需满足 [接口约定](../../docs/interfaces.md)。

## 接入已有算法包

1. 把 ROS 2 包放到 `src/algorithms/<包名>/`；使用独立依赖包时也放在这个目录下，保持包名唯一。
2. 包内提供并安装 `launch/perception.launch.py`，在其中启动算法、设置 `use_sim_time=True`、完成话题 remap。算法内部话题可以自定，外部接口按平台文档统一。
3. 根目录 `./build.sh` 会递归构建这些包。
4. 启动 `./run.sh --algorithm <包名>`。平台自动 include 该包的 `launch/perception.launch.py`。

也可以先运行 `./run.sh`，再在已 source `install/setup.bash` 的终端独立启动算法。`--example` 和 `--algorithm` 不能同时使用，每个结果话题只保留一个算法发布者。

二维和特殊路面接口也可发布，但当前自动评分覆盖三维目标与道路边线。权重文件应由算法团队单独管理，不要求把数据集或 rosbag 放进仓库。
