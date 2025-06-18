# gz_moveit2(jazzy)

## build

```bash
colcon build --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"
```

## convert to sdf

```bash
# 编译后运行
cd /path/to/panda_description/scripts

./xacro2sdf.bash
```

## launch

```bash
ros2 launch panda_moveit_config ex_gz_control.launch.py
```