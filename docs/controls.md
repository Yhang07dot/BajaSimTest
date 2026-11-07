# 车速和 RViz 视角

## 车速

永久默认值在 `src/baja_perception_sim/config/scenario.yaml`：

```yaml
vehicle:
  speed: 1.0       # 目标速度，单位 m/s
  max_speed: 3.0   # 配置上限，同时设置 Gazebo 车辆限速
```

单次启动覆盖默认值：

```bash
./run.sh --speed 2
./run.sh --speed 3
./run.sh --speed 3 --algorithm baja_algorithm_template
```

命令行优先于YAML。修改后重启仿真即可，不需要重编译。当前不是运行中即时调速，也没有车速滑块。默认仍为1m/s；2m/s约7.2km/h，3m/s约10.8km/h。

原来生成器限制1m/s，现已使用 `max_speed` 校验，控制器改成按曲率限制转向。固定0.3rad/s上限在3m/s的10m半径弯道无法留出纠偏余量；目前上限随速度缩放，保持相同的可用转向曲率。

已在当前场景、seed42、保留特殊路面的条件下分别完成2和3m/s整圈以上测试。其他布局、车辆参数和真实感知算法负载下仍应重新验收。默认max_speed=3；车辆模型的配置校验最多允许5，但超过3尚无本项目验收数据。

## RViz 交互

重启 `./run.sh`，或者关闭旧调试窗口后运行 `./debug_rviz.sh`，载入更新后的配置。两个配置都包含工具栏，默认选中 **Move Camera**，Views面板使用 **Orbit**。

鼠标操作在**右侧三维渲染区域**中进行，相机图像面板和左侧属性表不会旋转三维场景：

| 操作 | 效果 |
|---|---|
| 左键拖动 | 旋转观察角度 |
| 中键拖动，或Shift＋左键拖动 | 平移观察中心 |
| 滚轮 | 缩放 |
| 切回工具栏Move Camera | 从选择等工具返回相机操作 |
| Views中载入Reset view | 恢复初始观察位置 |

DEBUG以base_link为固定参考，跟随车辆但允许自由旋转/平移/缩放；EVALUATION以map为参考，可以自由观察场景。固定参考坐标系不等于锁死相机。

先前配置遗漏Tools列表。RViz载入配置时会清除原工具并重新从Tools加载；现在显式加入MoveCamera、Interact、Select和FocusCamera，同时加宽窗口，给三维区域留出空间。依据：[RViz ToolManager源码](https://github.com/ros2/rviz/blob/humble/rviz_common/src/rviz_common/tool_manager.cpp)、[Orbit鼠标处理源码](https://github.com/ros2/rviz/blob/humble/rviz_default_plugins/src/rviz_default_plugins/view_controllers/orbit/orbit_view_controller.cpp)。
