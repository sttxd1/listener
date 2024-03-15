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
                                                    "/camera/color/image_raw/compressed",
                                                    std::bind(&Listener::ImageCallback, this, std::placeholders::_1),
                                                    image_transport);

    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    "/camera/color/image_raw/compressed", rclcpp::SensorDataQoS(),
    std::bind(&Listener::ImageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: /camera/color/image_raw/compressed");

    // Subscribe to the IMU topic
    // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    //     "/camera/imu", 10, std::bind(&Listener::ImuCallback, this, std::placeholders::_1),rmw_qos_profile_sensor_data);

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/camera/imu", rclcpp::SensorDataQoS(), std::bind(&Listener::ImuCallback, this, std::placeholders::_1));


    RCLCPP_INFO(this->get_logger(), "Subscribed to IMU topic: /camera/imu");
}

void Listener::ImageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & image_msg) {
    //image processing logic here
    cv::Mat cv_image = cv::imdecode(cv::Mat(image_msg->data), cv::IMREAD_COLOR);

        if (!cv_image.empty()) {
            // Convert cv::Mat to sensor_msgs/msg/Image
            sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image).toImageMsg();

    RCLCPP_INFO(this->get_logger(), "Received image message");}
}

void Listener::ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg) {
    //IMU processing logic here
    RCLCPP_INFO(this->get_logger(), "Received IMU message");
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(Listener)
