# Module for coloring lidar clouds.

The module receives images from the camera and the lidar cloud. Using calibration matrices, each lidar point is projected onto the image and takes its color from the corresponding pixel in the image. At the output, the module publishes a colored lidar cloud. If a point misses the image when projected, it is not included in the final colored cloud. Also during operation, an image with projected lidar points is shown.

The module operates in two modes:
- online mode: receives real-time images and lidar clouds and publishes colored clouds.
![](/images/painted_cloud_0.png)


- offline mode: colors bag files, while images from the camera are pre-segmented using the `segmentation_trt` module. As a result, the output creates a copy of the processed bag file with clouds colored in accordance with the segmentation results.
![](/images/painted_cloud_1.png)



# Explanation of calibration matrices
When the module starts, it initializes the calibration matrices in accordance with the matrices specified in the `calibration.yml` file (`Lidar2Camera` and `Base2Lidar`). The coordinates of the incoming lidar cloud can be in the lidar coordinate system (in which case the lidar points are converted to the camera coordinate system using the `Lidar2Camera` matrix), or in the base coordinate system of the car (in which case the `Base2Lidar` matrix is also used to first transform the lidar points into the lidar coordinate system).

During operation, the module also “listens” to published transformations using `tf2_ros::TransformListener` (topics `tf` and `tf_static`). If the required transformation from `lidar_frame` to `camera_frame` exists in the transformation tree, then the lidar points will be transformed according to the last published transformation. `Lidar2Camera` and `Base2Lidar` will not be used. This way you can monitor the calibration process in real time. If there is no such transformation, then `Lidar2Camera` and `Base2Lidar` will be used, as well as adjustments from the `tuning.yml` file.

## Dependencies
1.perception_pcl
The perception_pcl module is required for this to work:
```sh
git clone https://github.com/ros-perception/perception_pcl
```
It is recommended to select a specific commit:
```sh
git checkout 135a2c29e3f46904f586e963cc4a4ec3fe594a9c
```
(this recommendation is specified in the cloud_to_image module, on the basis of which this module was made. Not tested with other commits)

2. Eigen
https://eigen.tuxfamily.org/index.php?title=Main_Page

3.segmentation_trt
To work offline, you need a segmentation module: https://gitlab.nami.local/self-driving-group/computervision/ros2_visualcortex/-/tree/develop/segmentation_trt

4.painting_services
In order to be sure to color all the clouds in the bag file and save all the timestamps, the offline mode works not with topics, but with services. painting_services includes descriptions of these services.

## Assembly
Minimum assembly for online mode:
```sh
colcon build --symlink-install --packages-select pcl_conversions
colcon build --symlink-install --packages-select pcl_ros
colcon build --symlink-install --packages-select perception_pcl
colcon build --symlink-install --packages-select painting_services
colcon build --symlink-install --packages-select paint_cloud
```
`--symlink-install` is needed to avoid rebuilding the package when the tuning.yml file changes.


For offline mode, you need to additionally install the `visual_cortex` and `segmentation_trt` packages:

Make sure visual_cortex/VisualCortex is not empty and points to VisualCortex. If the folder is empty, you can create a symlink:
```sh
cd visual_cortex
sudo ln -s /home/projects/VisualCortex VisualCortex
```

Then build the packages:
```sh
colcon build --symlink-install --packages-select visual_cortex
colcon build --symlink-install --packages-select segmentation_trt
```

## Launch in online mode
1. specify calibration information in the `calibration.yml` file
- image dimensions `height` and `width`
- internal calibration `K`
- distortion coefficients `D`
- transition matrix from the lidar coordinate system to the camera system `Lidar2Camera`
- transition matrix from the main coordinate system of the car to the coordinate system of the lidar `Base2Lidar`

2. Specify the required paths and names in the file `paint_cloud.launch.xml`
- `tuning_file_path`: path to the `tuning.yml` file, which specifies the calibration adjustment (so that you can change the calibration parameters while the module is running)
- `calibration_file_path`: path to the file with calibrations `calibration.yml`
- `lidar_frame`: name of the lidar coordinate system
- `base_frame`: name of the main coordinate system (for example, base_link)
- `camera_frame`: name of the camera coordinate system
- `image_topic`: topic with an image
- `cloud_topic`: topic with lidar cloud

While the module is running, you can change the parameters in the `tuning.yml` file:
- `tx`, `ty`, `tz`, `rx`, `ry`, `rz` - corrective movements and turns. If they are zero, the conversions will occur based on the `Lidar2Camera` and `Base2Lidar` matrices in the `calibration.yml` file.
- `color_mode` = 0 or 1 - dot coloring mode. 0 - by intensity. 1 - in range.
- `{intensity_color_min, intensity_color_max}`, `{distance_color_min, distance_color_max}` - lower and upper thresholds for iridescence
   colors in the appropriate mode.
- `alpha` - transparency
- `radius` - radius of the projected point
- `distance_max` - maximum range of displaying points along the radius


For testing, you can run some bag file with a video stream and lidar clouds (for example, this file was tested the most: smb://r4/aidata/VideoAi/ros_logs/foxy/nami/2021/2021_11_08_TestRazmetka/Razmetka_08.11.2021_bonus_ploshad/Razmetka_08 .11.2021_bonus_ploshad_0.db3).

Launch:
```sh
ros2 bag play path_to_your_bag_files/Razmetka_08.11.2021_bonus_ploshad/Razmetka_08.11.2021_bonus_ploshad_0.db3 -l
ros2 launch paint_cloud paint_cloud.launch.xml
```
## Run in offline mode
1. specify calibration information in the `calibration.yml` file
- image dimensions `height` and `width`
- internal calibration `K`
- distortion coefficients `D`
- transition matrix from the lidar coordinate system to the camera system `Lidar2Camera`
- transition matrix from the main coordinate system of the car to the coordinate system of the lidar `Base2Lidar`

2. Specify the required paths and names in the paint_rosbag.launch.xml file
- `tuning_file_path`: path to the `tuning.yml` file, which specifies the calibration adjustment (so that you can change the calibration parameters while the module is running)
- `calibration_file_path`: path to the file with calibrations `calibration.yml`
- `lidar_frame`: name of the lidar coordinate system
- `base_frame`: name of the main coordinate system (for example, base_link)
- `camera_frame`: name of the camera coordinate system
- `image_topic`: topic with an image
- `cloud_topic`: topic with lidar cloud
- `bag_file_path`: path to the bag file
- `painted_bag_directory`: directory where the result will be saved


Before launching, you need to specify in paint_rosbag.launch.xml some bag file with a video stream and lidar clouds (for example, this file was tested the most: smb://r4/aidata/VideoAi/ros_logs/foxy/city/2021/2021_11_10_CityMap/ City_2/City_2_0.db3).

Launch:
```sh
ros2 launch paint_cloud paint_rosbag.launch.xml
```

## Notes
- In offline mode, processing is quite slow (about 5 seconds to process 1 second in a bag file on my laptop). Most likely the fact is that services in ROS2 work slower than the publisher-subscriber pattern
- When coloring bag files, the new bag file with colored clouds does not include topics with images, so that the files do not take up too much space. If you need to include images in the bag file, you can make changes to paint_rosbag.cpp:
```
   for (const auto& t:topics){
     if (t.type != "sensor_msgs/msg/Image"){ //remove this condition and create all topics
       writer.create_topic(t);
     }
   }
```
- To correctly display colored clouds in rviz2, you will most likely need to change the frame in the Fixed Frame field, and also change the Reliability Policy to Best Effort. It is also recommended to set size=0.1 to make the clouds more visible. An example of these settings can be seen in the second screen from the top.
