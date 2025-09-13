# KeyboardRemoteControl

## 说明

本程序用于在局域网内，通过Windows系统发送端的键盘输入实现对Ubuntu系统接收端的ROS2机器人的远程控制。

## 使用方法

1. 编译KeyboardRemoteControl Windows客户端源代码或下载已发布的客户端。
2. 将KeyboardRemoteControl_Ubuntu_RCLCPP_Reciever/keyboard_ctrl编译为ROS2功能包。
3. 运行发送端和接收端。

## 控制按键

w - 前进
s - 后退
a - 左转
d - 右转
shift - 加速
esc - 退出
q - 托管
e - 接管  