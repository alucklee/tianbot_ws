# ROS局部规划器

## 配套视频课程
![课程封面](./media/cover.jpg)
<img src="./media/QR.jpg" width="200" height="200" alt="课程地址">

## 课程地址
Bilibili：[《ROS局部规划器开发课程》](https://www.bilibili.com/cheese/play/ss61282) 

## 系统版本

- ROS Melodic (Ubuntu 18.04)
- ROS Noetic (Ubuntu 20.04)

## 课前准备
1. 获取源码:
```
cd ~/catkin_ws/src/
git clone https://github.com/6-robot/wpr_simulation.git
git clone https://github.com/6-robot/wpb_home.git
```
2. 安装依赖项:  

ROS Noetic (Ubuntu 20.04)
```
cd ~/catkin_ws/src/wpr_simulation/scripts
./install_for_noetic.sh
cd ~/catkin_ws/src/wpb_home/wpb_home_bringup/scripts
./install_for_noetic.sh
```
ROS Melodic (Ubuntu 18.04)
```
cd ~/catkin_ws/src/wpr_simulation/scripts
./install_for_melodic.sh
cd ~/catkin_ws/src/wpb_home/wpb_home_bringup/scripts
./install_for_melodic.sh
```
3. 编译
```
cd ~/catkin_ws
catkin_make
```