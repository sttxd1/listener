#ifndef LISTENER_H
#define LISTENER_H

#include <memory>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/imu.hpp" // Include IMU message header
#include "boost/lexical_cast.hpp"

class Listener : public rclcpp::Node
{
public:
  explicit Listener(const rclcpp::NodeOptions & options);

private:
  // Renamed from ImageCallback to ListenerCallback
  void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);
  // New callback method for IMU messages
  void ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg);
  //
  void DepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg);

  void InfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cameraInfo_msg);

  // Subscriber for image messages
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;

  // New subscriber for IMU messages
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfo_sub_;

  // Member variables for output path, file prefix, and input topic
  std::string output_path_, file_prefix_;
    const std::string image_topic_ = "/camera/color/image_raw";
    const std::string depth_topic_ = "/camera/aligned_depth_to_color/image_raw";

};

#endif // LISTENER_H
