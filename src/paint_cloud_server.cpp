#include "rclcpp/rclcpp.hpp"
#include "painting_services/srv/segment_image.hpp"
#include "painting_services/srv/paint_cloud.hpp"

#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.h"

#include <memory>

#include "paint_cloud_component.hpp"

std::shared_ptr<PaintCloudComponent> node;

void processRequest(const std::shared_ptr<painting_services::srv::PaintCloud::Request> request, std::shared_ptr<painting_services::srv::PaintCloud::Response> response)
{ 
  std::shared_ptr<sensor_msgs::msg::Image> im = std::make_shared<sensor_msgs::msg::Image>();
  std::shared_ptr<sensor_msgs::msg::PointCloud2> pc = std::make_shared<sensor_msgs::msg::PointCloud2>();
  *im = request->image;
  *pc = request->pc;
  auto res = node->projectLidarPointsToImage(pc, im);
  response->coloredpc = res.first;
  response->image_with_lidar_points = res.second;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node = std::make_shared<PaintCloudComponent>(node_options);
  rclcpp::Service<painting_services::srv::PaintCloud>::SharedPtr service = node->create_service<painting_services::srv::PaintCloud>("paint_cloud", &processRequest);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to paint your clouds.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
