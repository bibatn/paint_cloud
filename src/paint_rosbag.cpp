// C/C++
#include <cstdio>

// STD
#include <iostream>

// ROS
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <deque>

#include <opencv2/highgui/highgui.hpp>

#include "painting_services/srv/segment_image.hpp"
#include "painting_services/srv/paint_cloud.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"

#include <math.h>

//#include "test_msgs/msg/basic_types.hpp"

#include "rosbag2_cpp/info.hpp"

using namespace std::chrono_literals;
using namespace std;

using rosbag2_cpp::converter_interfaces::SerializationFormatConverter;

int encoding2mat_type(const std::string & encoding)
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

int main(int argc, char ** argv)
{
  std::deque<std::shared_ptr<sensor_msgs::msg::PointCloud2>> pointCloudQueue;
  int len = 0;
  //node init
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("segmentation_client");

  //paths
  std::string bagFilePath = node->declare_parameter("bag_file_path", "");
  std::string paintedBagDir = node->declare_parameter("painted_bag_directory", "");
  bagFilePath = "/run/user/1000/gvfs/smb-share:server=r4,share=aidata/VideoAi/ros_logs/foxy/city/2023/2023-07/2023-07-10_Redin/2023-07-10_Redin_2_db3";
  paintedBagDir = "/home/m/projects/bag_files/2023-07-10_Redin_2_db3";

  std::cout << "bagFilePath: " << bagFilePath << " " << argc << std::endl; 
  std::cout << "paintedBagDir: " << paintedBagDir << " " << argc << std::endl; 

  //clients initialization
  rclcpp::Client<painting_services::srv::SegmentImage>::SharedPtr clientSegmentImage = node->create_client<painting_services::srv::SegmentImage>("segment_image");
  auto requestSegmentImage = std::make_shared<painting_services::srv::SegmentImage::Request>();
  rclcpp::Client<painting_services::srv::PaintCloud>::SharedPtr clientPaintCloud = node->create_client<painting_services::srv::PaintCloud>("paint_cloud");
  auto requestPaintCloud = std::make_shared<painting_services::srv::PaintCloud::Request>();

  //waiting for the segmentation and painting services
  while (!clientSegmentImage->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "segmentation service not available, waiting again...");
  }
  while (!clientPaintCloud->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "painting service not available, waiting again...");
  }
  
  //initialize a bagfile reader
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_cpp::StorageOptions storage_options{};
  storage_options.uri = bagFilePath;
  storage_options.storage_id = "sqlite3";
  rosbag2_cpp::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  reader.open(storage_options, converter_options);
  auto topics = reader.get_all_topics_and_types();
  
  rosbag2_cpp::Info info;
  const auto metadata = info.read_metadata(bagFilePath, "sqlite3");
  std::cout << "MESSAGE COUNT: " << metadata.message_count << " " << metadata.duration.count() / 1000000000.0 << std::endl;
  double bagDuration = metadata.duration.count() / 1000000000.0;
  

  for (auto t:topics){
    std::cout << "meta name: " << t.name << std::endl;
    std::cout << "meta type: " << t.type << std::endl;
    std::cout << "meta serialization_format: " << t.serialization_format << std::endl;
    //std::cout << "message count: " << t.message_count << std::endl;
  }
  
  //initialize a bagfile writer where the colored point cloud will be saved
  rosbag2_cpp::writers::SequentialWriter writer;
  rosbag2_cpp::StorageOptions so = {paintedBagDir, "sqlite3", 0, 0};
  rosbag2_cpp::ConverterOptions co = {"cdr", "cdr"};
  writer.open(so, co);
  for (const auto& t:topics){
    if (1){
      writer.create_topic(t);
    }
  }

  //additional topic for colored point clouds
  std::string cloudTopic = node->declare_parameter("cloud_topic", "pc");
  cloudTopic = "/sensing/lidar/top/compensated/pointcloud";

  std::string paintedCloudTopic = cloudTopic + "_colored";
  rosbag2_storage::TopicMetadata tm;
  tm.name = paintedCloudTopic;
  tm.type = "sensor_msgs/msg/PointCloud2"; 
  tm.serialization_format = "cdr";
  writer.create_topic(tm);

  //prepare
  std::string imageTopic = node->declare_parameter("image_topic", "");
  int64_t firstMessageTime = 0; //for timings
  sensor_msgs::msg::Image lastSegMask;

  imageTopic = "/CF/compressed";

  //keep reading messages from the bag file
  uint64_t messageCounter = 0;
  while (reader.has_next()){
    // serialized data
    auto serialized_message = reader.read_next();
    if (firstMessageTime == 0){
        firstMessageTime = serialized_message->time_stamp;
    }

    messageCounter++;
    
    if (serialized_message->topic_name == imageTopic)
    { //message from camera
      //get image
      std::cout << "processed: " << ((serialized_message->time_stamp) - firstMessageTime) / 1000000000.0 << "/" << bagDuration << " seconds ("
                << messageCounter << "/" <<  metadata.message_count << " messages)" << std::endl;
      rclcpp::SerializedMessage serializedCameraMsg(*serialized_message->serialized_data);
      rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializationImage;
      sensor_msgs::msg::Image lastCameraImage;
      sensor_msgs::msg::CompressedImage lastCompressedCameraImage;
      serializationImage.deserialize_message(&serializedCameraMsg, &lastCompressedCameraImage);
      cv::Mat a = cv::imdecode(lastCompressedCameraImage.data, cv::IMREAD_COLOR);
      lastCameraImage.header = lastCompressedCameraImage.header;
      lastCameraImage.encoding = "bgr8";
      lastCameraImage.height = a.rows;
      lastCameraImage.width = a.cols;
      lastCameraImage.is_bigendian = false;
      lastCameraImage.data.assign(a.datastart, a.dataend);
      
      //segment the image
      requestSegmentImage->image = lastCameraImage;
      auto result = clientSegmentImage->async_send_request(requestSegmentImage);
      if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service segment_image");
      }
      lastSegMask = result.get()->imageseg;

      writer.write(serialized_message);
    }
    else if (serialized_message->topic_name == cloudTopic && lastSegMask.height > 0)
    { //message from lidar
      sensor_msgs::msg::PointCloud2 extractedPointCloud;
      std::shared_ptr<sensor_msgs::msg::PointCloud2> extractedPointCloudPtr(new sensor_msgs::msg::PointCloud2);
      rclcpp::SerializedMessage serializedPointCloud(*serialized_message->serialized_data);
      rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializationPC;
      serializationPC.deserialize_message(&serializedPointCloud, extractedPointCloudPtr.get());

      pointCloudQueue.push_front(extractedPointCloudPtr);
      len++;
      if(len > 6)
          pointCloudQueue.pop_back();

      deque<shared_ptr<sensor_msgs::msg::PointCloud2>>::iterator iteratorOnMinDeviation;
      int64 minDeviation = std::numeric_limits<int64>::max();
      for(deque<shared_ptr<sensor_msgs::msg::PointCloud2>>::iterator it = pointCloudQueue.begin(); it!=pointCloudQueue.end(); it++)
      {
          int64 T1 = (static_cast<int64>((*it)->header.stamp.sec))*1000000000 + static_cast<int64>((*it)->header.stamp.nanosec);
          int64 T2 = (static_cast<int64>(lastSegMask.header.stamp.sec))*1000000000 + static_cast<int64>(lastSegMask.header.stamp.nanosec);
          if(std::abs(T1 - T2) < minDeviation)
          {
              minDeviation = std::abs(T1 - T2);
              iteratorOnMinDeviation = it;
          }
      }
      if(minDeviation > 50000000)
          continue;
      
      //paint point cloud with segmentation colors
      requestPaintCloud->image = lastSegMask;
      requestPaintCloud->pc = *iteratorOnMinDeviation->get();
      auto result = clientPaintCloud->async_send_request(requestPaintCloud);
      if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service segment_image");
      } 
      sensor_msgs::msg::PointCloud2 paintedPointCloud = result.get()->coloredpc;
      sensor_msgs::msg::Image imageWithLidarPoints = result.get()->image_with_lidar_points;
      rclcpp::SerializedMessage serializedPaintedPointCloud;
      rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializationpc;
      serializationpc.serialize_message(&paintedPointCloud, &serializedPaintedPointCloud);
      
      //write painted point cloud to bag file
      auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
      bag_message->time_stamp = serialized_message->time_stamp + 1; //add 1ns so that painted pointcloud has a different timestamp
      bag_message->topic_name = paintedCloudTopic;
      bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(&serializedPaintedPointCloud.get_rcl_serialized_message(), [](rcutils_uint8_array_t * /* data */) {});
      writer.write(serialized_message);
      writer.write(bag_message);
      
    }
    else { //some other message (just copy it)
      writer.write(serialized_message);
    }
  }
  rclcpp::shutdown();
  return 0;
}
