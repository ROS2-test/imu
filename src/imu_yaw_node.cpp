#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>

namespace
{

float yaw_from_quat(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

}  // namespace

class ImuYawNode : public rclcpp::Node
{
public:
  ImuYawNode()
  : Node("imu_yaw_node")
  {
    declare_parameter<std::string>("imu_topic", "imu/data_raw");
    declare_parameter<std::string>("yaw_topic", "imu/yaw");

    const auto imu_topic = get_parameter("imu_topic").as_string();
    const auto yaw_topic = get_parameter("yaw_topic").as_string();

    yaw_pub_ = create_publisher<std_msgs::msg::Float32>(
      yaw_topic, rclcpp::SensorDataQoS().keep_last(1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ImuYawNode::on_imu, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "imu_yaw_node: %s → %s", imu_topic.c_str(), yaw_topic.c_str());
  }

private:
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto & q = msg->orientation;
    const double n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    // Skip invalid / unset orientation (same gate as localization).
    if (msg->orientation_covariance[0] < 0.0 || n2 < 0.25) {
      return;
    }

    std_msgs::msg::Float32 out;
    out.data = yaw_from_quat(q.x, q.y, q.z, q.w);
    yaw_pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuYawNode>());
  rclcpp::shutdown();
  return 0;
}
