<!--
 * @Author: zhanghao
 * @Date: 2022-11-29 20:04:36
 * @LastEditTime: 2022-12-02 22:08:42
 * @FilePath: /hao_camera_launch/README.md
 * @Description: 
-->
# hao_camera_launch
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
```
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