// ChcnavFsdBridge：华测 CGI 组合导航 -> FSD 标准话题桥接节点
//
// 本节点为项目自定义开发，不属于厂商 demo 示例（src/demo/ 下的 ChcnavFixDemo 等仅供示范）。
//
// 输入：
//   /chcnav/devpvt   msg_interfaces/Hcinspvatzcb
//                    由厂商 HcCgiProtocolProcessNode 从 INSPVATZCB(0x1201) 协议解析得到，
//                    本节点直接复用其已解析字段，不重复做协议层解析。
// 输出：
//   /chcnav/odometry  nav_msgs/Odometry            局部 ENU 平面位姿 + 车体速度 + 偏航角速度
//   /chcnav/velocity  geometry_msgs/TwistStamped   车体坐标系速度
//
// 设计约束（对齐 WUTA-FSD 的 localization_manager/config/ekf.yaml）：
//   1. frame_id = odom，child_frame_id = base_link；本节点不发布任何 TF，
//      odom->base_link 由 FSD 的 robot_localization EKF 独占发布。
//   2. 两个话题时间戳同源，均沿用 devpvt 的 GPS 时间。
//   3. 姿态与角速度统一为 REP-103（右手系、逆时针为正、弧度）。
//   4. pose 协方差取自设备自带标准差字段，保证 EKF 马氏距离门限可用（不可为 0）。

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <msg_interfaces/msg/hcinspvatzcb.hpp>

namespace chcnav_fsd_bridge
{

namespace
{

constexpr double kDeg2Rad = M_PI / 180.0;

// WGS84 椭球参数，用于经纬度 -> 局部 ENU 切平面投影
constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84F = 1.0 / 298.257223563;

// 标准差下限：EKF 使用马氏距离门限，协方差为 0 会使门限失效
constexpr double kMinPositionStd = 1e-3;    // m
constexpr double kMinAngleStd = 1e-4;       // rad
constexpr double kMinLinearVelStd = 0.05;   // m/s
constexpr double kMinAngularVelStd = 0.02;  // rad/s

double variance(double std_dev, double floor_std)
{
  const double s = std::max(std_dev, floor_std);
  return s * s;
}

}  // namespace

class ChcnavFsdBridge : public rclcpp::Node
{
public:
  ChcnavFsdBridge()
  : Node("chcnav_fsd_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/chcnav/devpvt");
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/chcnav/odometry");
    velocity_topic_ = declare_parameter<std::string>("velocity_topic", "/chcnav/velocity");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    // 局部 ENU 原点；任一为 NaN 时改用首个有效定位作为原点
    origin_lat_param_ = declare_parameter<double>("origin_lat", std::nan(""));
    origin_lon_param_ = declare_parameter<double>("origin_lon", std::nan(""));
    origin_alt_param_ = declare_parameter<double>("origin_alt", std::nan(""));

    // 低于该 GNSS 状态不发布（4/8 为 RTK 固定解）；置 0 可在无天线时台架调试
    min_gnss_status_ = declare_parameter<int>("min_gnss_status", 1);

    // 安装角/符号标定余量，默认 0，按实车实测微调
    roll_offset_deg_ = declare_parameter<double>("roll_offset_deg", 0.0);
    pitch_offset_deg_ = declare_parameter<double>("pitch_offset_deg", 0.0);
    yaw_offset_deg_ = declare_parameter<double>("yaw_offset_deg", 0.0);

    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(velocity_topic_, 10);

    devpvt_sub_ = create_subscription<msg_interfaces::msg::Hcinspvatzcb>(
      input_topic_, 10,
      [this](const msg_interfaces::msg::Hcinspvatzcb::ConstSharedPtr msg) { onDevpvt(msg); });

    RCLCPP_INFO(
      get_logger(),
      "Chcnav FSD bridge: %s -> %s / %s (frame %s -> %s, min_gnss_status=%d)",
      input_topic_.c_str(), odometry_topic_.c_str(), velocity_topic_.c_str(),
      odom_frame_.c_str(), base_frame_.c_str(), min_gnss_status_);
  }

private:
  void onDevpvt(const msg_interfaces::msg::Hcinspvatzcb::ConstSharedPtr & msg)
  {
    if (static_cast<int>(msg->stat[1]) < min_gnss_status_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "GNSS 状态 %u 低于 min_gnss_status=%d，暂停发布 FSD 话题",
        static_cast<unsigned int>(msg->stat[1]), min_gnss_status_);
      return;
    }

    if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) ||
      !std::isfinite(msg->altitude))
    {
      return;
    }

    if (!origin_ready_) {
      if (std::isfinite(origin_lat_param_) && std::isfinite(origin_lon_param_) &&
        std::isfinite(origin_alt_param_))
      {
        origin_lat_ = origin_lat_param_;
        origin_lon_ = origin_lon_param_;
        origin_alt_ = origin_alt_param_;
      } else {
        origin_lat_ = msg->latitude;
        origin_lon_ = msg->longitude;
        origin_alt_ = msg->altitude;
      }
      origin_ready_ = true;
      RCLCPP_INFO(
        get_logger(), "局部 ENU 原点: lat=%.9f lon=%.9f alt=%.3f",
        origin_lat_, origin_lon_, origin_alt_);
    }

    // --- 经纬高 -> 局部 ENU 切平面 ---
    const double lat0_rad = origin_lat_ * kDeg2Rad;
    const double sin_lat0 = std::sin(lat0_rad);
    const double e2 = kWgs84F * (2.0 - kWgs84F);
    const double w = 1.0 - e2 * sin_lat0 * sin_lat0;
    const double rn = kWgs84A / std::sqrt(w);                     // 卯酉圈曲率半径
    const double rm = kWgs84A * (1.0 - e2) / std::pow(w, 1.5);     // 子午圈曲率半径

    const double east = (msg->longitude - origin_lon_) * kDeg2Rad * rn * std::cos(lat0_rad);
    const double north = (msg->latitude - origin_lat_) * kDeg2Rad * rm;
    const double up = static_cast<double>(msg->altitude) - origin_alt_;

    // --- 姿态：devpvt.yaw 为车体坐标系双天线航向，[-180,180] 逆时针为正，与 REP-103 一致 ---
    tf2::Quaternion q;
    q.setRPY(
      (static_cast<double>(msg->roll) + roll_offset_deg_) * kDeg2Rad,
      (static_cast<double>(msg->pitch) + pitch_offset_deg_) * kDeg2Rad,
      (static_cast<double>(msg->yaw) + yaw_offset_deg_) * kDeg2Rad);
    q.normalize();

    // --- 车体坐标系速度 (m/s) 与角速度 (deg/s -> rad/s) ---
    const double vx = msg->vehicle_linear_velocity.x;
    const double vy = msg->vehicle_linear_velocity.y;
    const double vz = msg->vehicle_linear_velocity.z;
    const double wx = static_cast<double>(msg->vehicle_angular_velocity.x) * kDeg2Rad;
    const double wy = static_cast<double>(msg->vehicle_angular_velocity.y) * kDeg2Rad;
    const double wz = static_cast<double>(msg->vehicle_angular_velocity.z) * kDeg2Rad;

    // --- /chcnav/odometry ---
    nav_msgs::msg::Odometry odom;
    odom.header = msg->header;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = east;
    odom.pose.pose.position.y = north;
    odom.pose.pose.position.z = up;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    fillPoseCovariance(odom.pose.covariance, msg);
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = vz;
    odom.twist.twist.angular.x = wx;
    odom.twist.twist.angular.y = wy;
    odom.twist.twist.angular.z = wz;
    fillTwistCovariance(odom.twist.covariance);

    // --- /chcnav/velocity ---
    geometry_msgs::msg::TwistStamped vel;
    vel.header = msg->header;
    vel.header.frame_id = base_frame_;
    vel.twist = odom.twist.twist;

    odometry_pub_->publish(odom);
    velocity_pub_->publish(vel);
  }

  static void fillPoseCovariance(
    std::array<double, 36> & cov,
    const msg_interfaces::msg::Hcinspvatzcb::ConstSharedPtr & msg)
  {
    std::fill(cov.begin(), cov.end(), 0.0);

    // position_stdev = [lat, lon, alt]；局部 ENU 下 x=东<-lon, y=北<-lat, z=天<-alt
    const double var_east = variance(msg->position_stdev[1], kMinPositionStd);
    const double var_north = variance(msg->position_stdev[0], kMinPositionStd);
    const double var_up = variance(msg->position_stdev[2], kMinPositionStd);

    // euler_stdev = [roll, pitch, yaw]，单位 deg
    const double var_roll =
      variance(static_cast<double>(msg->euler_stdev[0]) * kDeg2Rad, kMinAngleStd);
    const double var_pitch =
      variance(static_cast<double>(msg->euler_stdev[1]) * kDeg2Rad, kMinAngleStd);
    const double var_yaw =
      variance(static_cast<double>(msg->euler_stdev[2]) * kDeg2Rad, kMinAngleStd);

    cov[0] = var_east;
    cov[7] = var_north;
    cov[14] = var_up;
    cov[21] = var_roll;
    cov[28] = var_pitch;
    cov[35] = var_yaw;
  }

  static void fillTwistCovariance(std::array<double, 36> & cov)
  {
    std::fill(cov.begin(), cov.end(), 0.0);
    // 设备未提供车体坐标系速度标准差（只有 ENU 方向），此处取保守常数
    const double var_linear = kMinLinearVelStd * kMinLinearVelStd;
    const double var_angular = kMinAngularVelStd * kMinAngularVelStd;
    cov[0] = var_linear;
    cov[7] = var_linear;
    cov[14] = var_linear;
    cov[21] = var_angular;
    cov[28] = var_angular;
    cov[35] = var_angular;
  }

  std::string input_topic_;
  std::string odometry_topic_;
  std::string velocity_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  double origin_lat_param_;
  double origin_lon_param_;
  double origin_alt_param_;
  double origin_lat_ = 0.0;
  double origin_lon_ = 0.0;
  double origin_alt_ = 0.0;
  bool origin_ready_ = false;

  int min_gnss_status_ = 1;

  double roll_offset_deg_ = 0.0;
  double pitch_offset_deg_ = 0.0;
  double yaw_offset_deg_ = 0.0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Subscription<msg_interfaces::msg::Hcinspvatzcb>::SharedPtr devpvt_sub_;
};

}  // namespace chcnav_fsd_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<chcnav_fsd_bridge::ChcnavFsdBridge>());
  rclcpp::shutdown();
  return 0;
}
