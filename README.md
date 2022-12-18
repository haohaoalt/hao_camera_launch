<!--
 * @Author: zhanghao
 * @Date: 2022-11-29 20:04:36
 * @LastEditTime: 2022-12-18 18:05:20
 * @FilePath: /hao_camera_launch/README.md
 * @Description: 
-->
# hao_camera_launch
## 00 2022-12-18 push & del in hao007
这是一个包含单目、双目、RGBD相机功能包的工作空间
```
git clone git clone git@github.com:haohaoalt/hao_camera_launch.git
cd hao_camera_launch 
catkin_make
```
## 01 usb_cam

```
source devel setup.bash
roslaunch usb_cam usb_cam-test.launch
```
```xml
<launch>
  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
<!--节点的名字叫做usb_cam，然后运行一个叫usb_cam_node的可执行文件，这个文件在ros的lib里面，找不到源码文件，只有这个包装好可执行文件-->
 
    <param name="video_device" value="/dev/video0" />
<!--摄像头的编号，类型：string-->
 
    <param name="image_width" value="640" />
<!--图像的横向分辨率，类型int-->
 
    <param name="image_height" value="480" />
<!--图像的纵向分辨率，类型int-->
 
    <param name="pixel_format" value="yuyv" />
<!--像素编码，可选值：mjepg、yuyv、uyvy，类型：string-->    
 
    <param name="camera_frame_id" value="usb_cam" />
<!--摄像头坐标系，类型：string-->
 
    <param name="io_method" value="mmap"/>
<!--IO通道，可选值：mmap、read、userptr，类型：string-->
 
  </node>
  <node name="image_view" pkg="image_view" type="image_view" respawn="false" output="screen">
    <remap from="image" to="/usb_cam/image_raw"/>
<!--话题的名字映射为/usb_cam/image_raw-->
 
    <param name="autosize" value="true" />
  </node>
</launch>
```
framerate：类型-int；默认值-30；帧率
brightness：类型-int；默认值-32；亮度-0~255
saturation：类型-int；默认值-32；饱和度-0~255
contrast：类型-int；默认值-32；对比度-0~255
sharpness：类型-int；默认值-22；清晰度-0~255
autofocus：类型-bool；默认值-false；自动对焦
focus：类型-int；默认值-51；焦点（非自动对焦状态下有效）
camera_info_url：类型-string；默认值- —；摄像头校准文件路径
camera_name：类型-string；默认值-“head_camera”；摄像头名字

## 02 realsense camera

### 01 源码安装

```
git clone https://github.com/IntelRealSense/librealsense
cd librealsense
sudo apt-get install libudev-dev pkg-config libgtk-3-dev
sudo apt-get install libusb-1.0-0-dev pkg-config
sudo apt-get install libglfw3-dev
sudo apt-get install libssl-dev
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger 
mkdir build
cd build
cmake ../ -DBUILD_EXAMPLES=true
make
sudo make install
```

注意L515接入USB3.0端口

进入librealsense/build/examples/capture，测试效果：

`./rs-capture` 

或直接使用realsense-viewer工具查看效果：
`realsense-viewer`


### 02 安装RealSense-Ros

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
cd ..
catkin_init_workspace
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

note:
rs_camera.launch中添加

```
  <rosparam>
      /camera/motion_module/global_time_enabled: true
      /camera/l500_depth_sensor/global_time_enabled: true
      /camera/rgb_camera/global_time_enabled: true
  </rosparam>
```

取消开启rviz的warning
使用rviz可视化，注意L515接入USB3.0端口
新打开一个终端2，通过ros节点启动
`roslaunch realsense2_camera rs_camera.launch`
新打开一个终端3，运行rviz，实现点云可视化`rviz`
可根据应用需求修改launch文件；
`rostopic list` 指令查看当前所有的主题，启动前启动后对比，得出输出topic；

```
/camera/color/camera_info
/camera/color/image_raw
/camera/color/image_raw/compressed
/camera/color/image_raw/compressed/parameter_descriptions
/camera/color/image_raw/compressed/parameter_updates
/camera/color/image_raw/compressedDepth
/camera/color/image_raw/compressedDepth/parameter_descriptions
/camera/color/image_raw/compressedDepth/parameter_updates
/camera/color/image_raw/theora
/camera/color/image_raw/theora/parameter_descriptions
/camera/color/image_raw/theora/parameter_updates
/camera/color/metadata
/camera/depth/camera_info
/camera/depth/image_rect_raw
/camera/depth/image_rect_raw/compressed
/camera/depth/image_rect_raw/compressed/parameter_descriptions
/camera/depth/image_rect_raw/compressed/parameter_updates
/camera/depth/image_rect_raw/compressedDepth
/camera/depth/image_rect_raw/compressedDepth/parameter_descriptions
/camera/depth/image_rect_raw/compressedDepth/parameter_updates
/camera/depth/image_rect_raw/theora
/camera/depth/image_rect_raw/theora/parameter_descriptions
/camera/depth/image_rect_raw/theora/parameter_updates
/camera/depth/metadata
/camera/extrinsics/depth_to_color
/camera/l500_depth_sensor/parameter_descriptions
/camera/l500_depth_sensor/parameter_updates
/camera/motion_module/parameter_descriptions
/camera/motion_module/parameter_updates
/camera/realsense2_camera_manager/bond
/camera/rgb_camera/parameter_descriptions
/camera/rgb_camera/parameter_updates
/clicked_point
/diagnostics
/initialpose
/move_base_simple/goal
/rosout
/rosout_agg
/tf
/tf_static
```

rosbag record 指令录制指定topic的bag包，实现点云数据保存。

### 03 标定

 常见的标定板有：aprilgrid、checkerboard、circlegrid
开启相机

```
roslaunch realsense2_camera rs_camera.launch
//使用12*9棋盘格
rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.03 image:=/camera/color/image_raw camera:=/camera/color --no-service-check

```


标定结果中camera matrix为相机内参矩阵；distortion为畸变系数矩阵；rectification为矫正矩阵，一般为单位矩阵；projection为外部世界坐标系到像平面的投影矩阵。

使用kalibar进行标定

```
sudo apt-get install python3-setuptools python3-rosinstall ipython3  libeigen3-dev libboost-all-dev doxygen libopencv-dev ros-noetic-vision-opencv ros-noetic-image-transport-plugins ros-noetic-cmake-modules python-software-properties software-properties-common libpoco-dev python-matplotlib python3-scipy python3-git python3-pip libtbb-dev libblas-dev liblapack-dev python3-catkin-tools libv4l-dev 

sudo pip install python-igraph
```

sudo apt-get install -y python3-dev python-pip python-scipy \
    python-matplotlib ipython python-wxgtk4.0 python-tk python-igraph


rosrun kalibr kalibr_calibrate_cameras --bag /home/hao007/camera_calib.bag --topics /camera/color/image_raw --models pinhole-radtan --target /home/hao007/aprilgrid.yaml

 rosbag record -O apriltag.bag /image
rosrun topic_tools throttle messages /camera/color/image_raw 4.0 /image
roslaunch realsense2_camera rs_camera.launch