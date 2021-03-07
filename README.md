# learn-ros
http://wiki.ros.org/cn/ROS/Tutorials

## 1. 创建工作空间:
http://wiki.ros.org/catkin/Tutorials/create_a_workspace <br>
```
$ source /opt/ros/noetic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ echo $ROS_PACKAGE_PATH
接下来就可以使用workspace了
```

## 2. 创建catkin程序包 
http://wiki.ros.org/cn/ROS/Tutorials/CreatingPackage#cn.2FROS.2FTutorials.2Fcatkin.2FCreatingPackage.A.2BUhte.2Bk4ATio-catkin.2Begtej1MF- <br>
```
cd ~/catkin_ws/src
# 以下命令会创建创建一个名为'beginner_tutorials'的新程序包，这个程序包依赖于std_msgs、roscpp和rospy
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

## 3. 编译catkin程序包
```
$ source /opt/ros/melodic/setup.bash
$ cd ~/catkin_ws && make_catkin
```
