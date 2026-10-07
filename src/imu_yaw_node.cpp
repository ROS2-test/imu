#include <cmath>
#include <mutex>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
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
 * Gyro health latch for Waveshare AHRS.
 *
 * While gyro is dead: yaw_valid=false, SOVA/MCU own orientation. We only
 * mirror MCU yaw (no second integration).
 * While gyro is alive: continuous yaw from IMU orientation deltas.
 *
 * Dead  = robot turning (wheels) but IMU gyro silent or AHRS yaw stuck.
 * Alive = while turning, ω_imu agrees with ω_wheel for unfreeze_consec
 *         samples (not a bare |ω_imu| threshold — works at any speed,
 *         rejects flicker that is not real recovery).
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
    declare_parameter<std::string>("valid_topic", "/imu/yaw_valid");
    declare_parameter<double>("imu_wz_dead_eps", 0.10);
    declare_parameter<double>("wheel_wz_eps", 0.05);
    declare_parameter<double>("freeze_dyaw_eps", 1.0e-4);
    // |w_imu| must be within [agree_lo, agree_hi] * |w_wheel|, same sign.
    declare_parameter<double>("agree_lo", 0.4);
    declare_parameter<double>("agree_hi", 2.5);
    declare_parameter<int>("freeze_consec", 5);
    declare_parameter<int>("unfreeze_consec", 40);

    const auto imu_topic = get_parameter("imu_topic").as_string();
    const auto odom_topic = get_parameter("odom_topic").as_string();
    const auto yaw_topic = get_parameter("yaw_topic").as_string();
    const auto valid_topic = get_parameter("valid_topic").as_string();
    imu_wz_dead_eps_ = get_parameter("imu_wz_dead_eps").as_double();
    wheel_wz_eps_ = get_parameter("wheel_wz_eps").as_double();
    freeze_dyaw_eps_ = get_parameter("freeze_dyaw_eps").as_double();
    agree_lo_ = get_parameter("agree_lo").as_double();
    agree_hi_ = get_parameter("agree_hi").as_double();
    freeze_consec_ = get_parameter("freeze_consec").as_int();
    unfreeze_consec_ = get_parameter("unfreeze_consec").as_int();

    yaw_pub_ = create_publisher<std_msgs::msg::Float32>(yaw_topic, 10);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(valid_topic, 10);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&ImuYawNode::on_imu, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      std::bind(&ImuYawNode::on_odom, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "imu_yaw_node: health latch — dead if turning&gyro silent; "
      "alive if w_imu agrees with w_wheel for %d samples",
      unfreeze_consec_);
  }

private:
  bool wheels_turning() const
  {
    return std::fabs(last_wheel_wz_) >= wheel_wz_eps_;
  }

  bool gyro_silent(double imu_wz) const
  {
    return std::fabs(imu_wz) < imu_wz_dead_eps_;
  }

  // Real recovery: turning, gyro not silent, same sign, magnitude matches wheels.
  bool gyro_agrees(double imu_wz) const
  {
    if (!wheels_turning() || gyro_silent(imu_wz)) {
      return false;
    }
    if (imu_wz * last_wheel_wz_ <= 0.0) {
      return false;
    }
    const double aw = std::fabs(last_wheel_wz_);
    const double ai = std::fabs(imu_wz);
    return ai >= agree_lo_ * aw && ai <= agree_hi_ * aw;
  }

  void publish_yaw(bool valid)
  {
    std_msgs::msg::Float32 yaw_msg;
    yaw_msg.data = static_cast<float>(continuous_yaw_);
    yaw_pub_->publish(yaw_msg);

    std_msgs::msg::Bool valid_msg;
    valid_msg.data = valid;
    valid_pub_->publish(valid_msg);
  }

  void enter_dead(double imu_wz)
  {
    if (gyro_dead_) {
      return;
    }
    gyro_dead_ = true;
    live_count_ = 0;
    RCLCPP_WARN(
      get_logger(),
      "gyro DEAD (w_imu=%.3f w_wheel=%.3f): IMU out, MCU owns orientation",
      imu_wz, last_wheel_wz_);
  }

  void enter_alive(double imu_yaw, double imu_wz)
  {
    if (!gyro_dead_) {
      return;
    }
    gyro_dead_ = false;
    dead_count_ = 0;
    live_count_ = 0;
    last_imu_yaw_ = imu_yaw;
    RCLCPP_WARN(
      get_logger(),
      "gyro ALIVE (w_imu=%.3f w_wheel=%.3f): resume IMU deltas, cont=%.3f",
      imu_wz, last_wheel_wz_, continuous_yaw_);
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_wheel_wz_ = msg->twist.twist.angular.z;

    if (gyro_dead_ && initialized_) {
      continuous_yaw_ = yaw_from_quat(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      publish_yaw(false);
    }
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const double imu_yaw = yaw_from_quat(
      msg->orientation.x, msg->orientation.y,
      msg->orientation.z, msg->orientation.w);
    const double imu_wz = msg->angular_velocity.z;
    const double dy = wrap_pi(imu_yaw - last_imu_yaw_);
    const bool yaw_stuck = std::fabs(dy) < freeze_dyaw_eps_;

    if (!initialized_) {
      continuous_yaw_ = imu_yaw;
      last_imu_yaw_ = imu_yaw;
      initialized_ = true;
      gyro_dead_ = false;
      dead_count_ = 0;
      live_count_ = 0;
      publish_yaw(true);
      return;
    }

    // Unhealthy while turning: silent gyro, or AHRS yaw frozen (any speed).
    const bool unhealthy_now =
      wheels_turning() && (gyro_silent(imu_wz) || yaw_stuck);

    if (!gyro_dead_) {
      if (unhealthy_now) {
        ++dead_count_;
      } else {
        dead_count_ = 0;
      }
      if (dead_count_ >= freeze_consec_) {
        enter_dead(imu_wz);
      }
    } else {
      // Only revive on sustained agreement with wheels — no bare |w_imu| gate.
      if (gyro_agrees(imu_wz) && !yaw_stuck) {
        ++live_count_;
      } else {
        live_count_ = 0;
      }
      if (live_count_ >= unfreeze_consec_) {
        enter_alive(imu_yaw, imu_wz);
      }
    }

    if (gyro_dead_) {
      last_imu_yaw_ = imu_yaw;
      publish_yaw(false);
      return;
    }

    continuous_yaw_ = wrap_pi(continuous_yaw_ + dy);
    last_imu_yaw_ = imu_yaw;
    publish_yaw(true);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;

  std::mutex mutex_;
  double imu_wz_dead_eps_{0.10};
  double wheel_wz_eps_{0.05};
  double freeze_dyaw_eps_{1.0e-4};
  double agree_lo_{0.4};
  double agree_hi_{2.5};
  int freeze_consec_{5};
  int unfreeze_consec_{40};
  int dead_count_{0};
  int live_count_{0};

  bool initialized_{false};
  bool gyro_dead_{false};
  double continuous_yaw_{0.0};
  double last_imu_yaw_{0.0};
  double last_wheel_wz_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuYawNode>());
  rclcpp::shutdown();
  return 0;
}
