# 电机驱动考核说明

## 考核形式

本次考核需求梯度队员使用同一个控制器（hero_chassis_controller）驱动实际电机，提交所有代码（仓库），并进行最终答辩。

## 需求及评分细则

**总分: 100**

### 1. 简单了解电机常识

**分数: 5**

要求：

1. 理解电流与输出扭矩的关系
2. 理解减速箱提高扭矩的代价
3. 理解编码器的安装位置与校准条件
4. 了解齿隙与死区对控制的影响

参考资料：

- [Motor Structure and Installation:bilibili](https://www.bilibili.com/video/BV12t4y1V7cT/)
- [Motor‑Encoder‑Backlash Introduction:bilibili](https://www.bilibili.com/video/BV1kL411C7wH/)
- [M3508:rm-bbs-wiki](https://bbs.robomaster.com/wiki/20204847/817322?source=7#head1)
- [C620:rm-bbs-wiki](https://bbs.robomaster.com/wiki/20204847/817324?source=7)

### 2. 使用 CAN 报文控制电机

**分数: 15**

要求：

1. 了解 CAN 总线通信
2. 通过说明书了解数据帧的格式与字段含义，以及指令值与电流、力矩的换算关系
3. 正确完成实物接线（可向控制组队员寻求帮助）
4. 使用 `cansend` 循环发送控制报文实现电机正反转（实物操作必须有控制组队员在场）
5. 使用 `candump` 接收并解析反馈帧，得到转速、电流、温度

参考资料：

- [CAN_bus:wiki](https://en.wikipedia.org/wiki/CAN_bus)

- [SocketCAN_Tutorials:elinux](https://elinux.org/CAN_Bus#SocketCAN_Tutorials)
- [M3508:rm-bbs-wiki](https://bbs.robomaster.com/wiki/20204847/817322?source=7#head1)

- [C620:rm-bbs-wiki](https://bbs.robomaster.com/wiki/20204847/817324?source=7)

### 3. 理解 interface 的注册与使用

**分数: 10**

要求：

1. 理解 `hardware_interface` 在 ros_control 中的位置与职责
2. 理解 `EffortJointInterface` 的注册方式（transmission loader）
3. 以你写的 hero_chassis_controller 为例，说明控制器如何取得 handle，并通过它读取状态、写入命令

参考资料：

- [ros_control](http://wiki.ros.org/ros_control)

- [hardware_interface:github‑wiki](https://github.com/ros-controls/ros_control/wiki/hardware_interface) 

- [transmission_interface:github‑wiki](http://wiki.ros.org/transmission_interface) 

  务必好好体会仓库中的示例代码，理解它将对你的考核有极大帮助
  
- [simple_chassis_controller]

### 4. 理解 rm_hw 的设计思路

**分数: 15**

要求：

1. 理解 `RmRobotHW` 、`RmRobotHWLoop` 的类成员意义
2. 学习其中 `EffortJointInterface` 的注册方式，并思考你写的 hero_chassis_controller 如何通过它与 `RmRobotHW` 交互
3. 对照 `2. 使用 CAN 报文控制电机`，说明 `CanBus` 中反馈帧与指令帧的转换

参考资料：

- [rm_hw](https://github.com/rm-controls/rm_control/tree/master/rm_hw)
- [rm_hw_namespace:rm‑control‑docs](https://rm-control.netlify.app/namespacerm__hw)
- [socketcan_interface:wiki](https://wiki.ros.org/socketcan_interface)

### 5. 完善工作空间与功能包

**分数: 10**

要求：

1. 在任务一的工作空间中，继续设计本次任务中包的架构与依赖关系，**并说明设计依据**
2. 正确填写 **CMakeList.txt**, **package.xml**, **README.md**, 在完成后面的需求时记更新
3. 说明控制器的复用性，以及如何区分仿真和实车

参考资料：

- [catkin/CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt)
- [catkin/package.xml](http://wiki.ros.org/catkin/package.xml)
- [pluginlib](http://wiki.ros.org/pluginlib)

### 6. 使用 PID 控制电机的速度

**分数: 25**

要求：

1. 正确编写 config 文件和 launch 文件
2. 成功启动 rm_hw 并加载 `hero_chassis_controller`
3. 订阅 `/cmd_vel`，解算后驱动 M3508 电机正反转（实物操作必须有控制组队员在场）
4. 使用 PlotJuggler、RViz 观测电机运行数据，**依据数据**判断异常行为、调试控制参数（例如pid）

参考资料：

- [actuator_coefficient:rm_hw](https://github.com/rm-controls/rm_control/blob/master/rm_hw/config/actuator_coefficient.yaml)
- [hw_config_template:rm_hw](https://github.com/rm-controls/rm_control/blob/master/rm_hw/config/hw_config_template.yaml)
- [PlotJuggler:wiki](https://wiki.ros.org/plotjuggler)
- [rviz:wiki](http://wiki.ros.org/rviz)

### 7. 理解 actuator 与 joint 的映射关系

**分数: 20**

要求：

1. 理解 joint 与 actuator 的区别，以及为什么需要 transmission
2. 说明 joint 与 actuator 之间位置、速度、力矩的换算关系

参考资料：

-  [urdf/joint:roswiki](https://wiki.ros.org/urdf/XML/joint)

- [transmission_interface:roswiki](http://wiki.ros.org/transmission_interface)

