#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>

namespace
{

struct Quat
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

Quat quat_from_rpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  Quat q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

Quat quat_conj(const Quat & q)
{
  return Quat{-q.x, -q.y, -q.z, q.w};
}

Quat quat_mul(const Quat & a, const Quat & b)
{
  Quat q;
  q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return q;
}

float yaw_from_quat(const Quat & q)
{
  return static_cast<float>(
    std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
}

float pitch_from_quat(const Quat & q)
{
  const double s = std::max(-1.0, std::min(1.0, 2.0 * (q.w * q.y - q.z * q.x)));
  return static_cast<float>(std::asin(s));
}

float roll_from_quat(const Quat & q)
{
  return static_cast<float>(
    std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y)));
}

float angle_from_quat(const std::string & axis, const Quat & q)
{
  if (axis == "pitch") {
    return pitch_from_quat(q);
  }
  if (axis == "roll") {
    return roll_from_quat(q);
  }
  return yaw_from_quat(q);
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
    // After remount: rotate IMU quat into base, then take yaw (full ±π).
    // Matches static TF base_link -> base_imu_link RPY.
    declare_parameter<double>("mount_roll", 3.141592653589793);
    declare_parameter<double>("mount_pitch", -1.5707963267948966);
    declare_parameter<double>("mount_yaw", 0.0);
    // Fallback: extract raw IMU euler without mount (yaw|pitch|roll)
    declare_parameter<std::string>("euler_axis", "yaw");
    declare_parameter<bool>("use_mount", true);
    declare_parameter<double>("sign", 1.0);

    const auto imu_topic = get_parameter("imu_topic").as_string();
    const auto yaw_topic = get_parameter("yaw_topic").as_string();
    euler_axis_ = get_parameter("euler_axis").as_string();
    use_mount_ = get_parameter("use_mount").as_bool();
    sign_ = get_parameter("sign").as_double() >= 0.0 ? 1.0 : -1.0;

    q_mount_ = quat_from_rpy(
      get_parameter("mount_roll").as_double(),
      get_parameter("mount_pitch").as_double(),
      get_parameter("mount_yaw").as_double());
    q_mount_inv_ = quat_conj(q_mount_);

    yaw_pub_ = create_publisher<std_msgs::msg::Float32>(
      yaw_topic, rclcpp::SensorDataQoS().keep_last(1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ImuYawNode::on_imu, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "imu_yaw_node: %s → %s (use_mount=%s axis=%s sign=%.0f)",
      imu_topic.c_str(), yaw_topic.c_str(),
      use_mount_ ? "true" : "false", euler_axis_.c_str(), sign_);
  }

private:
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto & o = msg->orientation;
    const double n2 = o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w;
    if (msg->orientation_covariance[0] < 0.0 || n2 < 0.25) {
      return;
    }

    Quat q_imu{o.x, o.y, o.z, o.w};
    float heading = 0.0f;
    if (use_mount_) {
      // Orientation of base in world ≈ q_imu ⊗ q_mount^{-1}
      // (q_mount = rotation base→imu, same as static TF RPY)
      const Quat q_base = quat_mul(q_imu, q_mount_inv_);
      heading = yaw_from_quat(q_base);
    } else {
      heading = angle_from_quat(euler_axis_, q_imu);
    }

    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(sign_ * heading);
    yaw_pub_->publish(out);
  }

  bool use_mount_{true};
  std::string euler_axis_;
  double sign_{1.0};
  Quat q_mount_;
  Quat q_mount_inv_;
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
