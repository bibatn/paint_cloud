#include <memory>
#include <vector>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#define EIGEN_MPL2_ONLY //disable non MPL2 compatible features, or in other words disable the features which are still under the LGPL.
#include "Eigen/Core"
#include "Eigen/Geometry"

enum class ColorMode
{
  INTENSITY,
  DISTANCE
};

struct ColorParams
{
  ColorMode colorMode = ColorMode::INTENSITY;
  float intensityColorMin = 0.0;
  float intensityColorMax = 255.0;
  float distanceColorMin = 0.0;
  float distanceColorMax = 150.0;
  float alpha = 0.0;
  int radius = 3;
  double distanceMax = 150.0;
};

class PaintCloudComponent : public rclcpp::Node
{
public:
  explicit PaintCloudComponent(const rclcpp::NodeOptions& options);
  std::pair<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image> projectLidarPointsToImage(sensor_msgs::msg::PointCloud2::SharedPtr cloud, sensor_msgs::msg::Image::SharedPtr inputImageMsg);
private:
  void initializeCalibrationParameters(const std::string& calibrationFilePath);
  void drawCircle(const double& distance, const cv::Mat& image, const Eigen::Vector3d& point);
  Eigen::Matrix4d getTransformFromTuningFile(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud, const sensor_msgs::msg::Image::ConstSharedPtr& inputImageMsg);
  void getColorSettings(ColorParams& colorParams);
  sensor_msgs::msg::Image toRosImageMsg(const cv::Mat& image, const sensor_msgs::msg::Image::ConstSharedPtr& inputImageMsg);
  std::string matTypeToEncoding(int matType);
  int encodingToMatType(const std::string& encoding);
  void imageCallback(sensor_msgs::msg::Image::UniquePtr msg);
  void cloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr msg);
  pcl::PointCloud<pcl::PointXYZRGB> createPaintedCloud(const sensor_msgs::msg::PointCloud2& transformedCloud, sensor_msgs::msg::PointCloud2::SharedPtr cloud, cv::Mat& imageCV, std::vector<Eigen::Vector3d>& projectedLidarPoints, std::vector<double>& distanceArray);
  std::pair<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image> processCloudAndImage(sensor_msgs::msg::PointCloud2::SharedPtr cloud, sensor_msgs::msg::Image::SharedPtr inputImageMsg, bool lookupTransform=true);
  cv::Mat createImageWithLidarPoints(cv::Mat imageCV, const std::vector<Eigen::Vector3d>& projectedLidarPoints, const std::vector<double>& distanceArray);
  geometry_msgs::msg::TransformStamped getRosTransformFromEigen(const Eigen::Matrix4d& matrix);
  void printEigenTransform(const geometry_msgs::msg::TransformStamped rt);
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudWithSegPub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub_;
  std::string imageTopic;

  Eigen::Matrix3d cameraIntrinsicMatrixEigen_;
  ColorParams colorParams_;
  int width_ = 1280;
  int height_ = 1024;

  cv::Mat map1_, map2_;
  std::string tuningFilePath_;
  std::string lidarFrame_;
  std::string baseFrame_;
  std::string cameraFrame_;
  Eigen::Matrix4d lidarToCameraTransform_;
  Eigen::Matrix4d baseToCameraTransform_;
  std::vector<int64> times;

  sensor_msgs::msg::Image::SharedPtr inputImageMsg_;

  std::shared_ptr<tf2_ros::TransformListener> transformListener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
};

