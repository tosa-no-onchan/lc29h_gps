## lc29h_gps  
  
  ROS2 対応 lc29h GPS publish node.  
  gps : [AE-GYSFDMAXB](https://akizukidenshi.com/catalog/g/gK-09991/)  
  
  ubuntu 22.04  
  PC and OrangePi 5 (Armbian and ubuntu 22.04)  
  ros2:humble  
  
  参照: [自作 Turtlebot3 自律走行に向けたプログラム。#14](http://www.netosa.com/blog/2022/09/-turtlebot3-14.html)  

#### 1. down load and build.  

    $ cd ~/colcon_ws/src  
    $ git clone https://github.com/tosa-no-onchan/lc29h_gps.git  
    $ cd ..  
    $ colcon build --symlink-install [--parallel-workers 1] --packages-select lc29h_gps  
    $ . install/setup.bash  

#### 2. run on SBC(Jetson Nano 2G)  

    $ sudo chmod 777 /dev/ttyUSB0  
    $ ros2 launch lc29h_gps lc29h_gps.launch.py  

#### 3. check on Remote PC  

    $ sudo ufw disable  
    $ ros2 topic list  
    /fix  
    /parameter_events  
    /rosout  

    $ ros2 topic echo /fix  

