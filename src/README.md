# UR5_easyhandeye_D435i_Calibration
> 使用UR5和D435i实现手眼标定,借用visp进行手眼标定,其内部的easy_handeye是旧版本的,新版本因为适配了Ubuntu20和python3从而存在一些问题.
>
> 主要参考链接为:https://zhuanlan.zhihu.com/p/127803592

## 1. 环境配置

手眼标定主要需要使用到:

### 1.0 相机标定

> 相机标定主要在camera_calibration中实现.
>
> 另外,标定的结果需要保存到realsense中.这个主要需要借用Intel的工具

#### 1.0.1 内参标定

```
roslaunch realsense2_camera rs_camera.launch color_width:=1920 color_height:=1080
rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.03 image:=/camera/color/image_raw camera:=/camera/color --no-service-check
```

标定过程中的时间会比较久,耐心等待即可.(5-10分钟).之后会直接在calibration/calibration_files中生成对应的相机内参文件



#### 1.0.2 内参写入

这里主要需要使用Intel内部自带的工具讲camera_info写入到Intelrealsense中

```
#1: 安装Intel对应插件
sudo apt-get install librscalibrationtool

#2: 读入realsense自带的相机内参
cd到calibration_files中
Intel.Realsense.CustomRW -r -f camera_info.xml
(如果说没有检测到序列号,重新插拔Realsense)

#3: 基于保存的transforms.yaml更改cameras_info.xml
cd到RWxml.py中,执行此python(路径默认保存过了)
python RWxml.py

#4: 写入到realsense中
cd到calibration_files中
Intel.Realsense.CustomRW -w -f camera.info.xml
```





#### 1.0.3 内参效果验证

**注:这个方法存在问题,最好的内参验证方法应该是获取了Pose之后重新投影到目标位置上去**



将两个aruco固定到一个板上,aruco基于camera_info同时读取这两个aruco的pose,然后获取这两个aruco的变换关系(即之间的距离),如果之间的变换距离是准确的,即认为标定是准确的.

(不过这个感觉其实也并不能说明什么问题,真正能够证明内参标定准确的还是讲点进行映射,然后看检测到的点和映射的点之间的差距)

##### (1) 二维码绘制

此处需要在一页上画两个二维码

https://tn1ck.github.io/aruco-print/

两个二维码分别为1和2的ID,另外尺寸为0.1cm



##### (2) 进行两个二维码的追踪

```
roslaunch aruco_ros check_calibration.launch
rosrun aruco_ros check_calibration.py
```





### 1.1 aruco_ros

#### 1.1.1 制作标定板

> ID:520,尺寸:100mm的在doc中的Online ArUco markers generator.pdf中,直接打印即可

在线生成aruco网站: https://chev.me/arucogen/

**注意,其中的Dictionary必须使用Original ArUco**

另外,减下来的时候,白边也可以留长一点



#### 1.1.2 单独aruco_ros使用

进行二维码识别,从而知道对应位置.这个想要跑起来,主要是启动aruco_ros/aruco_ros/launch/single.launch.这里面需要remap一下camera_info和image两个参数

```xml
<launch>

    <arg name="markerId"        default="520"/>
    <arg name="markerSize"      default="0.1"/>    <!-- in m -->
    <arg name="eye"             default="right"/>
    <arg name="marker_frame"    default="aruco_marker_frame"/>
    <arg name="ref_frame"       default=""/>  <!-- leave empty and the pose will be published wrt param parent_name -->
    <arg name="corner_refinement" default="LINES" /> <!-- NONE, HARRIS, LINES, SUBPIX -->


    <node pkg="aruco_ros" type="single" name="aruco_single">
        <remap from="/camera_info" to="/camera/color/camera_info" />
        <remap from="/image" to="/camera/color/image_raw" />
        <param name="image_is_rectified" value="True"/>
        <param name="marker_size"        value="$(arg markerSize)"/>
        <param name="marker_id"          value="$(arg markerId)"/>
        <param name="reference_frame"    value="camera_color_frame"/>   <!-- frame in which the marker pose will be refered -->
        <param name="camera_frame"       value="camera_color_frame"/>
        <param name="marker_frame"       value="$(arg marker_frame)" />
        <param name="corner_refinement"  value="$(arg corner_refinement)" />
    </node>

</launch>


```

- from /camera_info和from /image的to需要映射到开启摄像头的camera_info和image_raw





### 1.2 universial robot和ur_modern_driver

这里面直接把universial robot中的ur_driver替换成了ur_modern_driver进行控制

对于其他的机械臂,本质上是需要去监听到对应的几个tf_tree即可完成任务

另外,这里面的底层配置需要进行许多软件的安装(moveit等等).主要是基于rosdep进行配置

```
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y
```



### 1.3. realsense

这个其实可以直接sudo apt get 安装

```
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
sudo apt-get install ros-kinetic-realsense2-camera
```



### 1.4 vision_visp

这里面提供了手眼标定的参数调用,需要注意的是,Ubuntu16和Ubuntu18有专门对应的包,需要进行适配

另外,需要对应安装visp对应的驱动:

```
sudo apt-get install ros-kinetic-visp
```



### 1.5 easy_handeye

这个包其实一点都不easy,里面很多东西其实并没有说明白,需要进行重新配置等.另外也因为在适配Ubuntu20的,所以很多代码存在问题,(也是因此在这里保存了他所对应的包)

#### 1.5.1 UR5+D435i

这个组合可以直接进行使用.机械臂运动没有使用自动运动,怕导致末端

```xml
<arg name="freehand_robot_movement" value="true" />
```

若为true,即为自己拿着机械臂懂,如果为false,则是机械臂自己运动



#### 1.5.2 其他机械臂

主要是需要在launch文件中指定坐标系,坐标系的指定主要是查看tf_tree,有的就可以使用

```
rosrun rqt_tf_tree rqt_tf_tree
```

里面存在的,在tf中发布的坐标系,就可以进行使用



其他的机械臂,需要把他们的关节发布,然后在ur5_d435i_calibration.launch文件中进行关节的改变.另外,这里面的moveit可能是可以不用的(之后有其他机械臂可以测试一下.如果实在比较麻烦,也可以直接自己修改,调用visp的包就好,不使用easy_handeye.GUI如果麻烦可以考虑就用OpenCV进行结果保存)

```xml        <arg name="tracking_base_frame" value="camera_link" />
<arg name="tracking_base_frame" value="camera_link" />        
<arg name="tracking_marker_frame" value="camera_marker" />
<arg name="robot_base_frame" value="base" />
<arg name="robot_effector_frame" value="ee_link" />
```





## 2. 运行

> 主要以UR5和D435i为例

```
roslaunch easy_handeye ur_realsense_calibration.launch marker_size:=0.1 marker_id:=520
```

另外启动rqt_image_view,接受aruco_ros的result的topic,确保ID看得到

```
rosrun rqt_image_view rqt_image_view
```

一共会弹出3个窗口,Rviz进行机械臂末端查看,easy_handeye可以进行take sample结果记录,最终这个窗口compute即可获得最终结果
