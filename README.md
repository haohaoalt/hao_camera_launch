# hao_camera_launch
这是一个包含单目、双目、RGBD相机功能包的工作空间

## How to build
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
framerate：类型-int；默认值-30；帧率\
brightness：类型-int；默认值-32；亮度-0~255\
saturation：类型-int；默认值-32；饱和度-0~255\
contrast：类型-int；默认值-32；对比度-0~255\
sharpness：类型-int；默认值-22；清晰度-0~255\
autofocus：类型-bool；默认值-false；自动对焦\
focus：类型-int；默认值-51；焦点（非自动对焦状态下有效）\
camera_info_url：类型-string；默认值- —；摄像头校准文件路径\
camera_name：类型-string；默认值-“head_camera”；摄像头名字\

## 02 realsense camera(L515,D435I,D455)

### 2-1 源码安装

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

进入librealsense/build/examples/capture，测试效果：`./rs-capture` 或直接使用realsense-viewer工具查看效果：
`realsense-viewer`

### 2-2 安装RealSense-Ros

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

### 2-3 realsence设置
```shell
# 摄像头所支持的分辨率，对应的格式和频率即可看到
rs-enumerate-devices 

```


## 03 标定

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

sudo apt-get install -y python3-dev python-pip python-scipy python-matplotlib ipython python-wxgtk4.0 python-tk python-igraph


rosrun kalibr kalibr_calibrate_cameras --bag /home/hao007/camera_calib.bag --topics /camera/color/image_raw --models pinhole-radtan --target /home/hao007/aprilgrid.yaml

rosbag record -O apriltag.bag /image
rosrun topic_tools throttle messages /camera/color/image_raw 4.0 /image
roslaunch realsense2_camera rs_camera.launch

```




## 04 数据集录制
### 4-1 mono
```
rosbag record -O mono_xiang.bag /camera/camera_info /camera/image_raw /camera/image_raw/compressed
```

### 4-2 stereo
```
rosbag record -O stereo.bag /camera/color/camera_info /camera/color/image_raw /camera/color/image_raw/compressed /camera/aligned_depth_to_color/image_raw
```
### 4-3 rgbd
```
sudo apt-get install ros-melodic-rgbd-launch
# 启动相机
roslaunch realsense2_camera rs_rgbd.launch
```

```
rosbag record -O rgbd_d455.bag /camera/color/image_raw /camera/color/image_raw/compressed /camera/aligned_depth_to_color/image_raw

```

`rostopic echo /camera/color/camera_info`

```
height: 720
width: 1280
D: [-0.054022323340177536, 0.06301917135715485, -0.00014683687186334282, 0.0007357324357144535, -0.019964847713708878]
K: [634.971923828125, 0.0, 644.484130859375, 0.0, 634.470458984375, 361.453125, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [634.971923828125, 0.0, 644.484130859375, 0.0, 0.0, 634.470458984375, 361.453125, 0.0, 0.0, 0.0, 1.0, 0.0]
```

```
height: 480
width: 640
D: [-0.054022323340177536, 0.06301917135715485, -0.00014683687186334282, 0.0007357324357144535, -0.019964847713708878]
K: [380.983154296875, 0.0, 322.6904602050781, 0.0, 380.6822509765625, 240.8718719482422, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [380.983154296875, 0.0, 322.6904602050781, 0.0, 0.0, 380.6822509765625, 240.8718719482422, 0.0, 0.0, 0.0, 1.0, 0.0]

```

录制record
```
rosbag record -O 1026d455.bag /camera/color/camera_info /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /camera/color/image_rect_color /camera/depth/image_rect_raw
```

