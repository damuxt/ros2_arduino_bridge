# ros2_arduino_bridge

对ROS1中的ros_arduino_bridge的上位机内容做了修改，以适配ROS2。



1. 请先按照ros_arduino_bridge的下位机内容设计完毕了小车底盘。
   小车底盘设计课程资料：https://www.bilibili.com/video/BV1Ub4y1a7PH/
2. 请确保上位机已经安装了ROS2机器人操作系统。

### 使用准备

#### 1.硬件准备

通过USB数据线将机器人底盘连接到上位机，并打开底盘开关。上位机终端下执行指令：

```
ll /dev/ttyACM*
```

如正常，将输出类似如下的结果：

```
crw-rw---- 1 root dialout 166, 0  4月 27 20:15 /dev/ttyACM0
```

PS：如果物理连接无异常，Ubuntu系统显示`无法访问 ‘/dev/ttyACM‘: 没有那个文件或目录`，那么可能是brltty驱动占用导致的，可以运行`sudo apt remove brltty `然后重新插拔一下设备即可解决问题。

#### 2.系统准备

ros2_arduino_bridge是依赖于python-serial功能包的，请先在上位机安装该功能包，安装命令:

```
sudo pip install --upgrade pyserial
```

#### 3.软件安装

将此软件包下载到你的ROS2工作空间下的src目录。

#### 4.环境配置

终端下进入`ros2_arduino_bridge/scripts`,并执行指令：

```
bash create_udev_rules.sh
```

该指令将为Arduino端口绑定一个固定名称，重新将底盘与上位机连接，执行如下指令：

```
ll /dev | grep -i ttyACM
```

如正常，将输出类似如下的结果：

```
lrwxrwxrwx   1 root root           7  4月 27 20:16 myarduino -> ttyACM0
crwxrwxrwx   1 root dialout 166,   0  4月 27 20:16 ttyACM0
```

#### 5.构建功能包

工作空间下调用如下指令，构建功能包：

```
colcon build --packages-select ros2_arduino_bridge
```

### 使用流程

#### 1.配置参数

在功能包下提供了机器人底盘相关参数的配置文件`ros2_arduino_bridge/params`，参数内容可以自行修改（如果是初次使用，建议使用默认），该文件内容如下：

```
/ros2_arduino_node:
  ros__parameters:
    # pid 参数
    Kd: 45
    Ki: 0
    Ko: 50
    Kp: 8
    accel_limit: 0.5 # 加速限制
    base_controller_rate: 10
    base_controller_timeout: 1.0
    base_frame: base_footprint #基坐标系
    baud: 57600 # 波特率
    encoder_resolution: 3960 
    gear_reduction: 1
    motors_reversed: false
    port: /dev/myarduino # Arduino端口
    rate: 50
    timeout: 0.5
    use_sim_time: false
    wheel_diameter: 0.065 # 轮胎直径
    wheel_track: 0.21 # 轮间距
    max_vel_x: 0.18 # 最大线速度
    min_vel_x: -0.18 # 最小线速度
    max_vel_th: 0.8 # 最大角速度
    min_vel_th: -0.8 # 最小角速度
```

#### 2.启动launch文件

工作空间下，调用如下指令启动launch文件：

```
ros2 launch ros2_arduino_bridge ros2_arduino.launch.py 
```

#### 3.控制底盘运动

上位机上，启动键盘控制节点：

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

接下来，就可以通过键盘控制机器人运动了。
