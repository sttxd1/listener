#ifndef LISTENER_H
#define LISTENER_H

#include <memory>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/imu.hpp" // Include IMU message header
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "boost/lexical_cast.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class Listener : public rclcpp::Node
{
public:
  explicit Listener(const rclcpp::NodeOptions & options);

private:
  // Renamed from ImageCallback to ListenerCallback
  void ImageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & image_msg);
  // New callback method for IMU messages
  void ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg);

  // Subscriber for image messages
  // image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr color_compressed_sub_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;

  // New subscriber for IMU messages
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Member variables for output path, file prefix, and input topic
  std::string output_path_, file_prefix_;
    // const std::string image_topic_ = "/camera/color/image_raw/compressed";

};

#endif // LISTENER_H
