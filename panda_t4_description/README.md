# panda_t4_description

本项目基于panda_description和irobot_create_description，将轻量版的TurtleBot4作为Franka Emika Panda机械臂的移动底座，构建了一个集成式机器人系统。

- 集成后的机器人描述文件：urdf/panda.urdf.xacro
- 通过转换脚本生成：
    - urdf/panda.urdf（统一机器人描述格式）
    - panda/model.sdf（gazebo仿真描述格式）

## 已知问题

### SDF文件7生成警告

在生成SDF文件和启动Gazebo仿真时，系统产生以下警告信息（当前版本未解决）：

```bash
# 多条类似警告，涉及各种传感器的noise元素
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="base_link"]/sensor[@name="cliff_front_left"]/noise:<data-string>:L103]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="base_link"]/sensor[@name="cliff_front_right"]/noise:<data-string>:L143]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="base_link"]/sensor[@name="cliff_side_left"]/noise:<data-string>:L183]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="base_link"]/sensor[@name="cliff_side_right"]/noise:<data-string>:L223]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="ir_intensity_front_center_left"]/sensor[@name="ir_intensity_front_center_left"]/noise:<data-string>:L560]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="ir_intensity_front_center_right"]/sensor[@name="ir_intensity_front_center_right"]/noise:<data-string>:L636]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="ir_intensity_front_left"]/sensor[@name="ir_intensity_front_left"]/noise:<data-string>:L712]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="ir_intensity_front_right"]/sensor[@name="ir_intensity_front_right"]/noise:<data-string>:L788]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="ir_intensity_left"]/sensor[@name="ir_intensity_left"]/noise:<data-string>:L864]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="ir_intensity_right"]/sensor[@name="ir_intensity_right"]/noise:<data-string>:L940]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="ir_intensity_side_left"]/sensor[@name="ir_intensity_side_left"]/noise:<data-string>:L1016]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/joint[@name="wheel_drop_left_joint"]/physics/ode/provide_feedback:<data-string>:L1806]: XML Element[provide_feedback], child of element[ode], not defined in SDF. Copying[provide_feedback] as children of [ode].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/joint[@name="wheel_drop_right_joint"]/physics/ode/provide_feedback:<data-string>:L1927]: XML Element[provide_feedback], child of element[ode], not defined in SDF. Copying[provide_feedback] as children of [ode].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="base_link"]/sensor[@name="cliff_front_left"]/noise:<data-string>:L103]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="base_link"]/sensor[@name="cliff_front_right"]/noise:<data-string>:L143]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="base_link"]/sensor[@name="cliff_side_left"]/noise:<data-string>:L183]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="base_link"]/sensor[@name="cliff_side_right"]/noise:<data-string>:L223]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="ir_intensity_front_center_left"]/sensor[@name="ir_intensity_front_center_left"]/noise:<data-string>:L560]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="ir_intensity_front_center_right"]/sensor[@name="ir_intensity_front_center_right"]/noise:<data-string>:L636]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="ir_intensity_front_left"]/sensor[@name="ir_intensity_front_left"]/noise:<data-string>:L712]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="ir_intensity_front_right"]/sensor[@name="ir_intensity_front_right"]/noise:<data-string>:L788]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="ir_intensity_left"]/sensor[@name="ir_intensity_left"]/noise:<data-string>:L864]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="ir_intensity_right"]/sensor[@name="ir_intensity_right"]/noise:<data-string>:L940]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/link[@name="ir_intensity_side_left"]/sensor[@name="ir_intensity_side_left"]/noise:<data-string>:L1016]: XML Element[noise], child of element[sensor], not defined in SDF. Copying[noise] as children of [sensor].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/joint[@name="wheel_drop_left_joint"]/physics/ode/provide_feedback:<data-string>:L1806]: XML Element[provide_feedback], child of element[ode], not defined in SDF. Copying[provide_feedback] as children of [ode].
[gazebo-1] Warning [Utils.cc:132] [/sdf/model[@name="panda"]/joint[@name="wheel_drop_right_joint"]/physics/ode/provide_feedback:<data-string>:L1927]: XML Element[provide_feedback], child of element[ode], not defined in SDF. Copying[provide_feedback] as children of [ode].
```

### 插件加载错误

仿真运行时出现以下插件加载失败：

```bash
[gazebo-1] [Err] [SystemLoader.cc:92] Failed to load system plugin [libgazebo_ros_create_cliff_sensor.so] : Could not find shared library.
[gazebo-1] [Err] [SystemLoader.cc:92] Failed to load system plugin [libgazebo_ros_create_cliff_sensor.so] : Could not find shared library.
[gazebo-1] [Err] [SystemLoader.cc:92] Failed to load system plugin [libgazebo_ros_create_cliff_sensor.so] : Could not find shared library.
[gazebo-1] [Err] [SystemLoader.cc:92] Failed to load system plugin [libgazebo_ros_create_cliff_sensor.so] : Could not find shared library.
[gazebo-1] [Err] [SystemLoader.cc:92] Failed to load system plugin [libgazebo_ros_create_bumper.so] : Could not find shared library.
[gazebo-1] [Err] [SystemLoader.cc:92] Failed to load system plugin [libgazebo_ros_create_imu.so] : Could not find shared library.
[gazebo-1] [Err] [SystemLoader.cc:92] Failed to load system plugin [libgazebo_ros_create_ir_intensity_sensor.so] : Could not find shared library.
[gazebo-1] [Err] [SystemLoader.cc:92] Failed to load system plugin [libgazebo_ros_create_ir_intensity_sensor.so] : Could not find shared library.
[gazebo-1] [Err] [SystemLoader.cc:92] Failed to load system plugin [libgazebo_ros_create_ir_intensity_sensor.so] : Could not find shared library.
[gazebo-1] [Err] [SystemLoader.cc:92] Failed to load system plugin [libgazebo_ros_create_ir_intensity_sensor.so] : Could not find shared library.
[gazebo-1] [Err] [SystemLoader.cc:92] Failed to load system plugin [libgazebo_ros_create_ir_intensity_sensor.so] : Could not find shared library.
[gazebo-1] [Err] [SystemLoader.cc:92] Failed to load system plugin [libgazebo_ros_create_ir_intensity_sensor.so] : Could not find shared library.
[gazebo-1] [Err] [SystemLoader.cc:92] Failed to load system plugin [libgazebo_ros_create_ir_intensity_sensor.so] : Could not find shared library.
```

这些错误表明系统无法找到TurtleBot4/iRobot Create相关的传感器插件库，这可能会影响仿真中传感器功能的正常工作。