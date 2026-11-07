# 模型文件

需要权重时可放在此目录，或通过 ROS 参数指定外部路径。模板不需要任何权重。
若需要随 ROS 包安装，须在 setup.py（Python）或 CMakeLists.txt（C++）中声明安装规则，
运行时用 ament_index 查找路径，避免写死开发机绝对路径。
