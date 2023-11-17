#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <utility>
#include <vector>
#include <stdexcept>

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp_components/register_node_macro.hpp"
#include "paint_cloud_component.hpp"
#include "sensor_msgs/msg/point_cloud2.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/transform_datatypes.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

#include "opencv2/core/eigen.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#define EIGEN_MPL2_ONLY
#include "Eigen/Core"
#include "Eigen/Geometry"

#include <chrono>
#include <numeric>
#include <thread>

 using namespace std::chrono_literals;

PaintCloudComponent::PaintCloudComponent(const rclcpp::NodeOptions & options) 
  : Node("paint_cloud_node", options) 
{
  tuningFilePath_ = declare_parameter("tuning_file_path", "");
  std::cout << "tuning_file_path: " << tuningFilePath_ << std::endl;

  std::string calibrationFilePath = declare_parameter("calibration_file_path", "");
  std::cout << "calibration_file_path: " << calibrationFilePath << std::endl;

  lidarFrame_ = declare_parameter("lidar_frame", "lidar_top");
  std::cout << "lidar_frame: " << lidarFrame_ << std::endl;

  baseFrame_ = declare_parameter("base_frame", "base_link");
  std::cout << "base_frame: " << baseFrame_ << std::endl;

  cameraFrame_ = declare_parameter("camera_frame", "some_camera");

  initializeCalibrationParameters(calibrationFilePath);

  cv::startWindowThread();
  cv::namedWindow("image_with_lidar_points",cv::WINDOW_NORMAL);
  cv::resizeWindow("image_with_lidar_points", width_, height_);

  rclcpp::SensorDataQoS qos;
  qos.keep_last(1);
  qos.best_effort();
  std::string imageTopic = declare_parameter("image_topic", "");
  std::string cloudTopic = declare_parameter("cloud_topic", "");
  imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(imageTopic, qos, std::bind(&PaintCloudComponent::imageCallback, this,std::placeholders::_1));
  cloudSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloudTopic, qos, std::bind(&PaintCloudComponent::cloudCallback, this,std::placeholders::_1));
  cloudWithSegPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloudTopic + "_rgb", qos);

  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transformListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
}

void PaintCloudComponent::imageCallback(sensor_msgs::msg::Image::UniquePtr image){
  inputImageMsg_ = std::move(image);
}

std::pair<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image> PaintCloudComponent::processCloudAndImage(sensor_msgs::msg::PointCloud2::SharedPtr cloud, sensor_msgs::msg::Image::SharedPtr inputImageMsg, bool lookupTransform){
  if (!inputImageMsg){
    std::cout << std::endl << "waiting for first image..." << std::endl;
    return {{}, {}};
  }

  if (cloud->header.frame_id != baseFrame_ && cloud->header.frame_id != lidarFrame_) {
    RCLCPP_FATAL(this->get_logger(), "Unknown point cloud frame");
  }

  geometry_msgs::msg::TransformStamped cloudToCameraTransformROS;
  if (lookupTransform && tfBuffer_->_frameExists(cameraFrame_) && tfBuffer_->_frameExists(cloud->header.frame_id) && tfBuffer_->canTransform(cameraFrame_, cloud->header.frame_id, tf2::TimePointZero)){
    try
    {
        cloudToCameraTransformROS = tfBuffer_->lookupTransform(cameraFrame_, cloud->header.frame_id, tf2::TimePointZero);
        printEigenTransform(cloudToCameraTransformROS);
    }
    catch(tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "TF exception:\n%s", ex.what());
    }
  }
  else{
    Eigen::Matrix4d transform = getTransformFromTuningFile(cloud, inputImageMsg);
    cloudToCameraTransformROS = getRosTransformFromEigen(transform);
  }

  getColorSettings(colorParams_);
  sensor_msgs::msg::PointCloud2 transformedCloud;
  tf2::doTransform(*cloud, transformedCloud, cloudToCameraTransformROS);

  cv::Mat imageCV(inputImageMsg->height, inputImageMsg->width, encodingToMatType(inputImageMsg->encoding), const_cast<uchar*>(inputImageMsg->data.data()));

  //projections of lidar points onto the image plane
  std::vector<Eigen::Vector3d> projectedLidarPoints;
  projectedLidarPoints.reserve(transformedCloud.data.size());

  //distances of projected lidar points
  std::vector<double> distanceArray;
  distanceArray.reserve(transformedCloud.data.size());

  pcl::PointCloud<pcl::PointXYZRGB> out_cloud = createPaintedCloud(transformedCloud, cloud, imageCV, projectedLidarPoints, distanceArray);
  cv::Mat imageUndistortedWithLidarPointsCV = createImageWithLidarPoints(imageCV, projectedLidarPoints, distanceArray);

	sensor_msgs::msg::PointCloud2 cloudMsg;
	pcl::toROSMsg(out_cloud, cloudMsg);
	cloudMsg.header = cloud->header;
  sensor_msgs::msg::Image imageMsg = toRosImageMsg(imageUndistortedWithLidarPointsCV, inputImageMsg);

//  static int count = 0;
//    std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
//    int64 T1 = now.count();
//    int64 T2 = (static_cast<int64>(inputImageMsg->header.stamp.sec))*1000000000 + static_cast<int64>(inputImageMsg->header.stamp.nanosec);
//    int64 dT = T1 -T2;
//    times.push_back(dT);
//    count++;
//    if(count==100)
//    {
//      std::cout << "AVG TIME " << ((std::accumulate(times.begin(), times.end(), 0.0)) / times.size())/1000000000 <<std::endl;
//      times.clear();
//      count = 0;
//    }

  return {std::move(cloudMsg), std::move(imageMsg)};
}


cv::Mat PaintCloudComponent::createImageWithLidarPoints(cv::Mat imageCV, const std::vector<Eigen::Vector3d>& projectedLidarPoints, const std::vector<double>& distanceArray){
  cv::Mat imageUndistortedCV;

  if (imageCV.channels() == 1) {
    imageCV = imageCV*20;
    cv::cvtColor(imageCV, imageCV, cv::COLOR_GRAY2RGB, 3);
  }

  cv::remap(imageCV, imageUndistortedCV, map1_, map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

  cv::Mat imageUndistortedWithLidarPointsCV = imageUndistortedCV.clone();

  int count = 0;
  for (const auto & point : projectedLidarPoints) {
    int row = point[1];
    row = std::max(0, row);
    row = std::min(height_ - 1, row);
    int col = point[0];
    col = std::max(0, col);
    col = std::min(width_ - 1, col);
    drawCircle(distanceArray[count], imageUndistortedWithLidarPointsCV, point);
    count++;
  }

  cv::addWeighted(imageUndistortedCV, colorParams_.alpha, imageUndistortedWithLidarPointsCV, 1.0 - colorParams_.alpha , 0.0, imageUndistortedWithLidarPointsCV);
  cv::imshow("image_with_lidar_points", imageUndistortedWithLidarPointsCV);
  cv::waitKey(1);

  return imageUndistortedWithLidarPointsCV;
}

pcl::PointCloud<pcl::PointXYZRGB> PaintCloudComponent::createPaintedCloud(const sensor_msgs::msg::PointCloud2& transformedCloud, sensor_msgs::msg::PointCloud2::SharedPtr cloud, cv::Mat& imageCV, std::vector<Eigen::Vector3d>& projectedLidarPoints, std::vector<double>& distanceArray){
    if(imageCV.channels() == 4)
    {
        cv::cvtColor(imageCV, imageCV, cv::COLOR_RGBA2RGB);
    }
  double distance;
  int cloudCounter = -1;
  pcl::PointCloud<pcl::PointXYZRGB> out_cloud;
  pcl::PointCloud<pcl::PointXYZ> in_cloud;
	pcl::fromROSMsg(*cloud, in_cloud);

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformedCloud, "x"),
                                                    iter_y(transformedCloud, "y"), 
                                                    iter_z(transformedCloud, "z"),
                                                    iter_intensity(transformedCloud, "intensity");
      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity){

    ++cloudCounter;
    if (*iter_z <= 0.0) {
      continue;
    }

    distance = pow(pow(*iter_x, 2) + pow(*iter_y, 2) + pow(*iter_z, 2), 0.5);
    if (distance > colorParams_.distanceMax) {
      continue;
    }

    Eigen::Vector3d projected_point = cameraIntrinsicMatrixEigen_ * Eigen::Vector3d(*iter_x, *iter_y, *iter_z);
    Eigen::Vector3d normalized_projected_point = Eigen::Vector3d(projected_point.x()/projected_point.z(), projected_point.y()/projected_point.z(), static_cast<float>(*iter_intensity));

    //if a cloud point lies in the image plane
    if(0 <= static_cast<int>(normalized_projected_point.x()) &&
       static_cast<int>(normalized_projected_point.x()) <= static_cast<int>(width_) - 1 &&
       0 <= static_cast<int>(normalized_projected_point.y()) &&
       static_cast<int>(normalized_projected_point.y()) <= static_cast<int>(height_) - 1) 
      {
          projectedLidarPoints.push_back(normalized_projected_point);
          distanceArray.push_back(pow(pow(*iter_x, 2) + pow(*iter_y, 2) + pow(*iter_z, 2), 0.5));

          int row = normalized_projected_point[1];
          row = std::max(0, row);
          row = std::min(height_ - 1, row);
          int col = normalized_projected_point[0];
          col = std::max(0, col);
          col = std::min(width_ - 1, col);

          uchar val;
          cv::Vec3b val_bgr;
          if (imageCV.channels() == 1){
            val = imageCV.at<uchar>(row, col);
          }
          else{
            val_bgr = imageCV.at<cv::Vec3b>(row, col);
          }

          pcl::PointXYZRGB colored_3d_point;
          colored_3d_point.x = in_cloud.points[cloudCounter].x;
          colored_3d_point.y = in_cloud.points[cloudCounter].y;
          colored_3d_point.z = in_cloud.points[cloudCounter].z;

          // painting
          // if there are 3 channels, get the colors directly from the input image. 
          // If there is only one channel (like in segmentation mask), colors are hardcoded here (TODO: probably it's better to take these colors from a config file)
          if (imageCV.channels() == 3){
            colored_3d_point.r = val_bgr[2]; 
            colored_3d_point.g = val_bgr[1]; 
            colored_3d_point.b = val_bgr[0];
          }

          else if (val == 1 || val == 3) {colored_3d_point.r = 128; colored_3d_point.g = 128; colored_3d_point.b = 0;} // people and bikes -> yellow
          else if (val == 2 || val == 7) {colored_3d_point.r = 0; colored_3d_point.g = 128; colored_3d_point.b = 128;} // vehicles (cars, moto) -> cyan
          else if (val == 4) {colored_3d_point.r = 85; colored_3d_point.g = 55; colored_3d_point.b = 20;} // traffic lights -> brown
          else if (val == 5) {colored_3d_point.r = 128; colored_3d_point.g = 0; colored_3d_point.b = 128;} // signs -> magenta
          else if (val == 6) {colored_3d_point.r = 0; colored_3d_point.g = 64; colored_3d_point.b = 0;} // road -> green
          else if (val == 8) {colored_3d_point.r = 0; colored_3d_point.g = 0; colored_3d_point.b = 128;} // zebra -> blue
          else if (val == 9 || val == 10 || val == 11) {colored_3d_point.r = 128; colored_3d_point.g = 0; colored_3d_point.b = 0;} // lanes (dashed, solid, double solid) -> red
          else {colored_3d_point.r = 128; colored_3d_point.g = 128; colored_3d_point.b = 128;} // other points -> grey

          out_cloud.points.push_back(colored_3d_point);
    }
  }
  return out_cloud;
}

void PaintCloudComponent::cloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr cloud){
//  std::this_thread::sleep_for(50ms);
//  static int count = 0;
//    std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
//    int64 T1 = now.count();
//    int64 T2 = (static_cast<int64>(inputImageMsg_->header.stamp.sec))*1000000000 + static_cast<int64>(inputImageMsg_->header.stamp.nanosec);
//    int64 dT = T1 -T2;
//    times.push_back(dT);
//    count++;
//    if(count==100)
//    {
//      std::cout << "AVG TIME " << ((std::accumulate(times.begin(), times.end(), 0.0)) / times.size())/1000000000 <<std::endl;
//      times.clear();
//      count = 0;
//    }
  auto cloudAndImage = processCloudAndImage(move(cloud), inputImageMsg_);
  cloudWithSegPub_->publish(std::move(cloudAndImage.first));
}

std::pair<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image> PaintCloudComponent::projectLidarPointsToImage(sensor_msgs::msg::PointCloud2::SharedPtr cloud, sensor_msgs::msg::Image::SharedPtr inputImageMsg){
   return processCloudAndImage(cloud, inputImageMsg, false);
}

void PaintCloudComponent::drawCircle(const double & distance, const cv::Mat & image, const Eigen::Vector3d & point)
{
  switch (colorParams_.colorMode) {
    case ColorMode::INTENSITY:
    {
      float min_intensity = colorParams_.intensityColorMin;
      float max_intensity = colorParams_.intensityColorMax;

      float diff_intensity = max_intensity - min_intensity;

      float value = 1.0 - (point.z() - min_intensity) / diff_intensity;

      value = std::min(value, 1.0f);
      value = std::max(value, 0.0f);

      float h = value * 5.0f + 1.0f;
      int i = floor(h);
      float f = h - i;
      if (!(i & 1)) f = 1 - f; // if i is even
      float n = 1 - f;
      Eigen::Vector3d color;
      if (i <= 1) color[0] = n, color[1] = 0, color[2] = 1;
      else if (i == 2) color[0] = 0, color[1] = n, color[2] = 1;
      else if (i == 3) color[0] = 0, color[1] = 1, color[2] = n;
      else if (i == 4) color[0] = n, color[1] = 1, color[2] = 0;
      else if (i >= 5) color[0] = 1, color[1] = n, color[2] = 0;

      cv::circle(
              image, cv::Point(static_cast<int>(point.x()), static_cast<int>(point.y())), colorParams_.radius,
              cv::Scalar(static_cast<int>(color.z() * 255), static_cast<int>(color.y() * 255),
                         static_cast<int>(color.x() * 255)), -1);
      break;
    }
    case ColorMode::DISTANCE:
    {
      float min_value = colorParams_.distanceColorMin;
      float max_value = colorParams_.distanceColorMax;

      float diff_value = max_value - min_value;

      float value = 1.0 - (distance - min_value) / diff_value;

      value = std::min(value, 1.0f);
      value = std::max(value, 0.0f);

      float h = value * 5.0f + 1.0f;
      int i = floor(h);
      float f = h - i;
      if (!(i & 1)) f = 1 - f; // if i is even
      float n = 1 - f;
      Eigen::Vector3d color;
      if (i <= 1) color[0] = n, color[1] = 0, color[2] = 1;
      else if (i == 2) color[0] = 0, color[1] = n, color[2] = 1;
      else if (i == 3) color[0] = 0, color[1] = 1, color[2] = n;
      else if (i == 4) color[0] = n, color[1] = 1, color[2] = 0;
      else if (i >= 5) color[0] = 1, color[1] = n, color[2] = 0;

      cv::circle(
              image, cv::Point(static_cast<int>(point.x()), static_cast<int>(point.y())), colorParams_.radius,
              cv::Scalar(static_cast<int>(color.z() * 255), static_cast<int>(color.y() * 255),
                         static_cast<int>(color.x() * 255)), 3, 4);

      break;
    }
    default:
      std::cout << "Unknown mode" << std::endl;
      break;
  }
}

std::string PaintCloudComponent::matTypeToEncoding(int matType)
{
  std::string Res;
  switch (matType)
  {
    case CV_8UC1:
      Res = "mono8";
      break;

    case CV_8UC3:
      Res = "bgr8";
      break;

    case CV_16SC1:
      Res = "mono16";
      break;

    case CV_8UC4:
      Res = "rgba8";
      break;

    default:
      throw std::runtime_error("Unsupported encoding type");
  }
  return Res;
}

sensor_msgs::msg::Image PaintCloudComponent::toRosImageMsg(const cv::Mat& image, const sensor_msgs::msg::Image::ConstSharedPtr& inputImageMsg)
{
  sensor_msgs::msg::Image msg;
  msg.header = inputImageMsg->header;
  msg.height = image.rows;
  msg.width = image.cols;
  msg.encoding = matTypeToEncoding(image.type());
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.step);
  msg.data.assign(image.datastart, image.dataend);

  return msg;
}

int PaintCloudComponent::encodingToMatType(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "bgra8") {
      return CV_8UC4;

}
  throw std::runtime_error("Unsupported mat type");
}

Eigen::Matrix4d PaintCloudComponent::getTransformFromTuningFile(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud, const sensor_msgs::msg::Image::ConstSharedPtr& inputImageMsg){
  cv::FileStorage fs(tuningFilePath_, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    std::string errorMessage = "Failed to open file: " + tuningFilePath_;
    throw std::runtime_error(errorMessage);
  }
  double tx = 0.0;
  double ty = 0.0;
  double tz = 0.0;
  double rx = 0.0;
  double ry = 0.0;
  double rz = 0.0;
  fs["tx"] >> tx;
  fs["ty"] >> ty;
  fs["tz"] >> tz;
  fs["rx"] >> rx;
  fs["ry"] >> ry;
  fs["rz"] >> rz;

  tf2::Quaternion q_rot;
  q_rot.setRPY(rx, ry, rz);

  geometry_msgs::msg::TransformStamped transform;
  transform.child_frame_id = "/lidar_top";
  transform.header = inputImageMsg->header;
  transform.transform.translation.x = tx;
  transform.transform.translation.y = ty;
  transform.transform.translation.z = tz;
  transform.transform.rotation.x = q_rot.x();
  transform.transform.rotation.y = q_rot.y();
  transform.transform.rotation.z = q_rot.z();
  transform.transform.rotation.w = q_rot.w();

  Eigen::Matrix3d rotation;

  rotation = Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX());

  Eigen::Vector3d tranform;
  tranform << tx, ty, tz;

  Eigen::Matrix4d correction_matrix;
  correction_matrix.setIdentity();
  correction_matrix.block<3,3>(0,0) = rotation;
  correction_matrix.block<3,1>(0,3) = tranform;

  Eigen::Matrix4d correctedTransformation;
  if (cloud->header.frame_id == baseFrame_) {
    correctedTransformation = correction_matrix * baseToCameraTransform_;
  }
  else if (cloud->header.frame_id == lidarFrame_) {
    correctedTransformation = correction_matrix * lidarToCameraTransform_;
  }
  else{
    RCLCPP_FATAL(this->get_logger(), "Unknown point cloud frame");
  }

  return correctedTransformation;
}

void PaintCloudComponent::getColorSettings(ColorParams & colorParams)
{
  cv::FileStorage fs(tuningFilePath_, cv::FileStorage::READ);

  if (!fs.isOpened())
  {
    std::string errorMessage = "Failed to open file: " + tuningFilePath_;
    throw std::runtime_error(errorMessage);
  }

  fs["color_mode"] >> colorParams.colorMode;
  fs["intensity_color_min"] >> colorParams.intensityColorMin;
  fs["intensity_color_max"] >> colorParams.intensityColorMax;
  fs["distance_color_min"] >> colorParams.distanceColorMin;
  fs["distance_color_max"] >> colorParams.distanceColorMax;
  fs["alpha"] >> colorParams.alpha;
  fs["radius"] >> colorParams.radius;
  fs["distance_max"] >> colorParams.distanceMax;
}

void PaintCloudComponent::initializeCalibrationParameters(const std::string & calibrationFilePath)
{
  if (calibrationFilePath.empty())
  {
    RCLCPP_FATAL(this->get_logger(), "Calibration file path is empty: '%s'.", calibrationFilePath.c_str());
    rclcpp::shutdown();
  }

  cv::FileStorage fs(calibrationFilePath, cv::FileStorage::READ);

  if (!fs.isOpened())
  {
    RCLCPP_FATAL(this->get_logger(), "Cannot open file calibration file '%s'", calibrationFilePath.c_str());
    rclcpp::shutdown();
  }

  cv::Mat lidarToCameraTransformCV;
  fs["Lidar2Camera"] >> lidarToCameraTransformCV;
  if (lidarToCameraTransformCV.empty())
  {
    RCLCPP_FATAL(this->get_logger(), "An initial external calibration matrix is required for the adjustment");
    rclcpp::shutdown();
  }
  cv::cv2eigen(lidarToCameraTransformCV, lidarToCameraTransform_);

  std::cout << std::endl << "lidarToCameraTransform_:" << std::endl;
  std::cout << lidarToCameraTransform_ << std::endl << std::endl;

  fs["width"] >> width_;
  fs["height"] >> height_;

  std::cout << "Width: " << width_ << std::endl;
  std::cout << "Height: " << height_ << std::endl;

  std::cout << std::endl << "Image Size: " << cv::Size(width_, height_) << std::endl;

  cv::Mat camera_intrinsic_matrix(3, 3, CV_64FC1);
  fs["K"] >> camera_intrinsic_matrix;

  cv::cv2eigen(camera_intrinsic_matrix, cameraIntrinsicMatrixEigen_);

  std::cout << std::endl << "Camera Intrinsic Matrix:" << std::endl;
  std::cout << camera_intrinsic_matrix << std::endl << std::endl;

  cv::Mat distCoeffs(1, 5, CV_64FC1);
  fs["D"] >> distCoeffs;

  std::cout << std::endl << "Distortion Coefficients:" << std::endl;
  std::cout << distCoeffs << std::endl << std::endl;

  cv::initUndistortRectifyMap(camera_intrinsic_matrix, distCoeffs, cv::Mat(), camera_intrinsic_matrix, cv::Size(width_, height_), CV_16SC2, map1_, map2_);


  if(lidarFrame_ != baseFrame_)
  {
    cv::Mat base2LidarCV(1, 6, CV_64FC1); // x, y, z, rotAroundX, rotAroundY, rotAroundZ
    fs["Base2Lidar"] >> base2LidarCV;
    Eigen::Matrix4d baseToLidarTransform;
    cv::cv2eigen(base2LidarCV, baseToLidarTransform);

    std::cout << std::endl << "baseToLidarTransform:" << std::endl;
    std::cout << baseToLidarTransform << std::endl << std::endl;

    if(!lidarToCameraTransform_.isZero())
    {
      baseToCameraTransform_ = lidarToCameraTransform_ * baseToLidarTransform;
      std::cout << std::endl << "baseToCameraTransform_:" << std::endl;
      std::cout << baseToCameraTransform_ << std::endl << std::endl;
    }
    else
    {
      RCLCPP_FATAL(this->get_logger(), "lidarToCameraTransform_ is not initialized");
    }
  }
}

geometry_msgs::msg::TransformStamped PaintCloudComponent::getRosTransformFromEigen(const Eigen::Matrix4d& matrix){
  Eigen::Quaterniond quat(matrix.block<3,3>(0,0));

  geometry_msgs::msg::TransformStamped rosTransform;
  rosTransform.transform.translation.x = matrix(0,3);
  rosTransform.transform.translation.y = matrix(1,3);
  rosTransform.transform.translation.z = matrix(2,3);
  rosTransform.transform.rotation.x = quat.x();
  rosTransform.transform.rotation.y = quat.y();
  rosTransform.transform.rotation.z = quat.z();
  rosTransform.transform.rotation.w = quat.w();

  return rosTransform;
}

void PaintCloudComponent::printEigenTransform(const geometry_msgs::msg::TransformStamped rt)
{
    Eigen::Matrix4d matrix;
    matrix.setIdentity();
    matrix(0,3) = rt.transform.translation.x;
    matrix(1,3) = rt.transform.translation.y;
    matrix(2,3) = rt.transform.translation.z;

    Eigen::Quaterniond q;
    q.x() = rt.transform.rotation.x;
    q.y() = rt.transform.rotation.y;
    q.z() = rt.transform.rotation.z;
    q.w() = rt.transform.rotation.w;
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    matrix.block<3,3>(0,0) = R;

    std::string sep = "\n----------------------------------------\n";
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout << matrix.format(CleanFmt) << sep;
}
RCLCPP_COMPONENTS_REGISTER_NODE(PaintCloudComponent)
