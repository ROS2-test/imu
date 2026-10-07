#include <cmath>
#include <mutex>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"

namespace {

inline double wrap_pi(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

inline double yaw_from_quat(double x, double y, double z, double w)
{
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

/**
 * Continuous yaw for SOVA / localization.
 *
 * Waveshare IMU zeros gyro / freezes orientation below ~0.1 rad/s.
 * Absolute IMU yaw after a freeze is stale and would snap the robot.
 *
 * Strategy:
 *  - Maintain continuous_yaw from IMU *deltas* only (never absolute).
 *  - While frozen (|ωz| < threshold): integrate wheel_odom ω.
 *  - On unfreeze: re-anchor last IMU sample without jumping continuous_yaw.
 *  - Always publish /imu/yaw.
 */
class ImuYawNode : public rclcpp::Node
{
public:
  ImuYawNode()
  : Node("imu_yaw_node")
  {
    declare_parameter<std::string>("imu_topic", "/imu/data");
    declare_parameter<std::string>("odom_topic", "/wheel_odom");
    declare_parameter<std::string>("yaw_topic", "/imu/yaw");
    declare_parameter<double>("freeze_wz_eps", 0.10);
    declare_parameter<int>("freeze_consec", 3);

    const auto imu_topic = get_parameter("imu_topic").as_string();
    const auto odom_topic = get_parameter("odom_topic").as_string();
    const auto yaw_topic = get_parameter("yaw_topic").as_string();
    freeze_wz_eps_ = get_parameter("freeze_wz_eps").as_double();
    freeze_consec_ = get_parameter("freeze_consec").as_int();

    yaw_pub_ = create_publisher<std_msgs::msg::Float32>(yaw_topic, 10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&ImuYawNode::on_imu, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&ImuYawNode::on_odom, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "imu_yaw_node: continuous yaw on %s, freeze |wz|<%.3f -> wheel integrate",
      yaw_topic.c_str(), freeze_wz_eps_);
  }

private:
  void publish_yaw()
  {
    std_msgs::msg::Float32 yaw_msg;
    yaw_msg.data = static_cast<float>(continuous_yaw_);
    yaw_pub_->publish(yaw_msg);
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double t =
      static_cast<double>(msg->header.stamp.sec) +
      1e-9 * static_cast<double>(msg->header.stamp.nanosec);
    const double wz = msg->twist.twist.angular.z;

    if (have_odom_stamp_ && frozen_ && initialized_) {
      const double dt = t - last_odom_t_;
      if (dt > 0.0 && dt < 0.5) {
        continuous_yaw_ = wrap_pi(continuous_yaw_ + wz * dt);
        publish_yaw();
      }
    }

    last_odom_t_ = t;
    have_odom_stamp_ = true;
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const double imu_yaw = yaw_from_quat(
      msg->orientation.x, msg->orientation.y,
      msg->orientation.z, msg->orientation.w);
    const double wz = msg->angular_velocity.z;
    const bool sample_frozen = std::fabs(wz) < freeze_wz_eps_;

    if (sample_frozen) {
      ++freeze_count_;
    } else {
      freeze_count_ = 0;
    }
    const bool now_frozen = freeze_count_ >= freeze_consec_;

    if (!initialized_) {
      continuous_yaw_ = imu_yaw;
      last_imu_yaw_ = imu_yaw;
      initialized_ = true;
      frozen_ = now_frozen;
      publish_yaw();
      return;
    }

    if (now_frozen) {
      frozen_ = true;
      last_imu_yaw_ = imu_yaw;
      publish_yaw();
      return;
    }

    if (frozen_) {
      last_imu_yaw_ = imu_yaw;
      frozen_ = false;
      publish_yaw();
      return;
    }

    const double dy = wrap_pi(imu_yaw - last_imu_yaw_);
    continuous_yaw_ = wrap_pi(continuous_yaw_ + dy);
    last_imu_yaw_ = imu_yaw;
    publish_yaw();
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;

  std::mutex mutex_;
  double freeze_wz_eps_{0.10};
  int freeze_consec_{3};
  int freeze_count_{0};

  bool initialized_{false};
  bool frozen_{false};
  bool have_odom_stamp_{false};
  double continuous_yaw_{0.0};
  double last_imu_yaw_{0.0};
  double last_odom_t_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuYawNode>());
  rclcpp::shutdown();
  return 0;
}
