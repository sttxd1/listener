#include "listener/listener.hpp" 

Listener::Listener(const rclcpp::NodeOptions & options) : Node("listener", options) {
    // Initialize parameters for output path and prefix
    output_path_ = this->declare_parameter<std::string>("output/path", "/tmp/");
    file_prefix_ = this->declare_parameter<std::string>("output/prefix", "");
    bool compressed = this->declare_parameter<bool>("compressed", false);

    std::string image_transport = "raw";
    if (compressed) {
        image_transport = "compressed";
    }

    // Subscribe to the image topic using a hard-coded topic name
    image_sub_ = image_transport::create_subscription(this,
                                                    image_topic_,
                                                    std::bind(&Listener::ImageCallback, this, std::placeholders::_1),
                                                    image_transport,
                                                    rmw_qos_profile_sensor_data);
    depth_sub_ = image_transport::create_subscription(this,
                                                    depth_topic_,
                                                    std::bind(&Listener::DepthCallback, this, std::placeholders::_1),
                                                    image_transport,
                                                    rmw_qos_profile_sensor_data);

    RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s", depth_topic_.c_str());
    // Subscribe to the IMU topic
    // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    //     "/camera/imu", 10, std::bind(&Listener::ImuCallback, this, std::placeholders::_1),rmw_qos_profile_sensor_data);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/camera/imu", rclcpp::SensorDataQoS(), std::bind(&Listener::ImuCallback, this, std::placeholders::_1));

    cameraInfo_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/color/camera_info", rclcpp::SensorDataQoS(), std::bind(&Listener::InfoCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to Info topic: /camera/color/camera_info");
    RCLCPP_INFO(this->get_logger(), "Subscribed to IMU topic: /camera/imu");
}

void Listener::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg) {
    //image processing logic here
    RCLCPP_INFO(this->get_logger(), "Received image message");
}

void Listener::DepthCallback(const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg) {
    //image processing logic here
    RCLCPP_INFO(this->get_logger(), "Received depth message");
}

void Listener::ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg) {
    //IMU processing logic here
    RCLCPP_INFO(this->get_logger(), "Received IMU message");
}

void Listener::InfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cameraInfo_msg) {
    //IMU processing logic here
    RCLCPP_INFO(this->get_logger(), "Received Info message");
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Listener)
