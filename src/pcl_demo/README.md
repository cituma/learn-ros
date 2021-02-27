# pcl demo
使用时, 将 pcl_demo 目录链接到 ~/catkin_ws/src/ 下. <br>
编译 <br>
```cd ~/catkin_ws && catkin_make```
<br> 运行 <br>
```cd ~/catkin_ws/src && roslaunch pcl_demo/launch/start.launch```

roslaunch启动rviz:<br>
launch文件中加入以下代码:<br>
```<node name="rviz" pkg="rviz" type="rviz" args="-d $(find pcl_create)/rviz/pcl_create.rviz" required="true" />```
<br> rviz配置文件的保存: <br>
首先通过rosrun rviz rviz打开图形界面，在图形界面中做好配置， 然后点击左上角File -> Save Config as 到 pcl_create/rviz/pcl_create.rviz
