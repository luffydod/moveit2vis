# 安装python第三方库
apt install python3-pip -y
pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
# pip install pyside6 xacro ultralytics --break-system-packages
pip install pyside6 xacro ultralytics NodeGraphQt --break-system-packages
pip install -U colcon-common-extensions vcstool --break-system-packages
```

# 运行测试(Run test)

```shell
# Shell A
source install/setup.bash
ros2 launch panda_moveit_config gazebo_obb.launch.py

# Shell B 调试用，在vscode中要安装ROS（Microsoft）、Python等模块，还要在Python文件中选择Python编译器
# source install/setup.bash
# ros2 launch panda_moveit_config arm_control.launch.py

# Shell C
source install/setup.bash
ros2 launch yolov8_obb yolov8_obb.launch.py

# Shell D
source install/setup.bash
cd src/ui_controller/
python3 main.py
```