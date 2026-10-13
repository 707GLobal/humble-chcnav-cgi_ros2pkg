#include "temporary_ins_diagnostics.hpp"
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
#include <stdexcept>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

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
  const double s = std::isfinite(std_dev) && std_dev >= 0.0
    ? std::max(std_dev, floor_std) : floor_std;
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

    // 默认0不因GNSS质量停止发布；任务机另外要求状态4才推进阶段
    min_gnss_status_ = declare_parameter<int>("min_gnss_status", 0);

    // 安装角/符号标定余量，默认 0，按实车实测微调
    roll_offset_deg_ = declare_parameter<double>("roll_offset_deg", 0.0);
    pitch_offset_deg_ = declare_parameter<double>("pitch_offset_deg", 0.0);
    yaw_offset_deg_ = declare_parameter<double>("yaw_offset_deg", 0.0);
    if (!std::isfinite(roll_offset_deg_) || !std::isfinite(pitch_offset_deg_) ||
      !std::isfinite(yaw_offset_deg_)) {
      throw std::invalid_argument("INS installation angle offsets must be finite");
    }

    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 10);
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(velocity_topic_, 10);

    devpvt_sub_ = create_subscription<msg_interfaces::msg::Hcinspvatzcb>(
      input_topic_, 10,
      [this](const msg_interfaces::msg::Hcinspvatzcb::ConstSharedPtr msg) { onDevpvt(msg); });
    if (diagnostics_->enabled()) {
      diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1),
        [recorder = diagnostics_]() { recorder->tick(); });
    }

    RCLCPP_INFO(
      get_logger(),
      "Chcnav FSD bridge: %s -> %s / %s (frame %s -> %s, min_gnss_status=%d)",
      input_topic_.c_str(), odometry_topic_.c_str(), velocity_topic_.c_str(),
      odom_frame_.c_str(), base_frame_.c_str(), min_gnss_status_);
    RCLCPP_INFO(get_logger(),
      "FSD linear velocity source: ENU rotated into base_link; "
      "ENU yaw = device yaw + 90 deg + yaw_offset_deg (east-zero, CCW).");
  }

private:
  void onDevpvt(const msg_interfaces::msg::Hcinspvatzcb::ConstSharedPtr & msg)
  {
    diagnostics_->add(ins_diag::PVT_RECEIVED);
    diagnostics_->input();
    diagnostics_->gnss(msg->stat[1]);
    if (static_cast<int>(msg->stat[1]) < min_gnss_status_) {
      diagnostics_->add(ins_diag::GNSS_REJECTED);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "GNSS 状态 %u 低于 min_gnss_status=%d，暂停发布 FSD 话题",
        static_cast<unsigned int>(msg->stat[1]), min_gnss_status_);
      return;
    }

    if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude) ||
      !std::isfinite(msg->altitude) || !std::isfinite(msg->roll) ||
      !std::isfinite(msg->pitch) || !std::isfinite(msg->yaw) ||
      !std::isfinite(msg->enu_velocity.x) || !std::isfinite(msg->enu_velocity.y) ||
      !std::isfinite(msg->enu_velocity.z) ||
      !std::isfinite(msg->vehicle_angular_velocity.x) ||
      !std::isfinite(msg->vehicle_angular_velocity.y) ||
      !std::isfinite(msg->vehicle_angular_velocity.z))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Rejecting non-finite INS pose/ENU velocity/angular velocity; no FSD output.");
      diagnostics_->add(ins_diag::NONFINITE_REJECTED);
      return;
    }

    for (int axis = 0; axis < 3; ++axis) {
      if (!std::isfinite(msg->position_stdev[axis]) || msg->position_stdev[axis] < 0.0 ||
        !std::isfinite(msg->euler_stdev[axis]) || msg->euler_stdev[axis] < 0.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Rejecting invalid INS pose standard deviations; no FSD output.");
        diagnostics_->add(ins_diag::STDEV_REJECTED);
        return;
      }
    }

    if (!origin_ready_) {
      const bool configured_origin = std::isfinite(origin_lat_param_) &&
        std::isfinite(origin_lon_param_) && std::isfinite(origin_alt_param_);
      if (configured_origin)
      {
        origin_lat_ = origin_lat_param_;
        origin_lon_ = origin_lon_param_;
        origin_alt_ = origin_alt_param_;
      } else {
        origin_lat_ = msg->latitude;
        origin_lon_ = msg->longitude;
        origin_alt_ = msg->altitude;
      }
      // 状态0也发布，但不能把无效经纬度永久锁为车辆定位原点。
      // 未达到4时只使用本帧临时参考，状态4到达后固定原点。
      origin_ready_ = configured_origin || msg->stat[1] == 4;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000, "局部 ENU 参考: lat=%.9f lon=%.9f alt=%.3f",
        origin_lat_, origin_lon_, origin_alt_);

      // 原点固定后投影常量只需算一次，不必每帧重算
      const double lat0_rad = origin_lat_ * kDeg2Rad;
      const double sin_lat0 = std::sin(lat0_rad);
      const double e2 = kWgs84F * (2.0 - kWgs84F);
      const double w = 1.0 - e2 * sin_lat0 * sin_lat0;
      rn_ = kWgs84A / std::sqrt(w);                    // 卯酉圈曲率半径
      rm_ = kWgs84A * (1.0 - e2) / std::pow(w, 1.5);   // 子午圈曲率半径
      cos_lat0_ = std::cos(lat0_rad);
    }

    // --- 经纬高 -> 局部 ENU 切平面（投影常量已在定原点时算好） ---
    const double east = (msg->longitude - origin_lon_) * kDeg2Rad * rn_ * cos_lat0_;
    const double north = (msg->latitude - origin_lat_) * kDeg2Rad * rm_;
    const double up = static_cast<double>(msg->altitude) - origin_alt_;

    // 设备 yaw 以北向为零、逆时针为正；ENU 的 ROS yaw 以东向为零。
    // 安装角偏置在参考轴转换后叠加。roll/pitch 沿用现有安装标定。
    tf2::Quaternion q;
    q.setRPY(
      (static_cast<double>(msg->roll) + roll_offset_deg_) * kDeg2Rad,
      (static_cast<double>(msg->pitch) + pitch_offset_deg_) * kDeg2Rad,
      (90.0 + static_cast<double>(msg->yaw) + yaw_offset_deg_) * kDeg2Rad);
    q.normalize();

    // ENU 速度有效而 vehicle_linear_velocity 在实车中可能全零。统一从 ENU
    // 矢量转换，保留前进/倒车、侧向和垂向分量；不能把 speed 标量直接作 vx。
    // 与发布的 pose 共用同一姿态：v_body = R_ENU_from_body^T * v_ENU。
    const tf2::Matrix3x3 enu_from_body(q);
    const tf2::Matrix3x3 body_from_enu = enu_from_body.transpose();
    const tf2::Vector3 v_body = body_from_enu * tf2::Vector3(
      msg->enu_velocity.x, msg->enu_velocity.y, msg->enu_velocity.z);
    const double vx = v_body.x();
    const double vy = v_body.y();
    const double vz = v_body.z();
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
    fillTwistCovariance(odom.twist.covariance, msg, body_from_enu);

    // --- /chcnav/velocity ---
    geometry_msgs::msg::TwistStamped vel;
    vel.header = msg->header;
    vel.header.frame_id = base_frame_;
    vel.twist = odom.twist.twist;

    odometry_pub_->publish(odom);
    diagnostics_->add(ins_diag::ODOMETRY_PUBLISHED);
    velocity_pub_->publish(vel);
    diagnostics_->add(ins_diag::VELOCITY_PUBLISHED);
    diagnostics_->output();
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

  static void fillTwistCovariance(
    std::array<double, 36> & cov,
    const msg_interfaces::msg::Hcinspvatzcb::ConstSharedPtr & msg,
    const tf2::Matrix3x3 & body_from_enu)
  {
    std::fill(cov.begin(), cov.end(), 0.0);
    // ENU 对角协方差同步旋转至车体坐标系，保留旋转产生的非对角项。
    // 设备未提供 ENU 方向间相关项；无效标准差用较保守的 1 m/s 兜底。
    std::array<double, 3> enu_variance;
    for (int axis = 0; axis < 3; ++axis) {
      const double s = msg->enu_velocity_stdev[axis];
      enu_variance[axis] = std::isfinite(s) && s >= 0.0
        ? variance(s, kMinLinearVelStd) : 1.0;
    }
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        for (int axis = 0; axis < 3; ++axis) {
          cov[row * 6 + col] += body_from_enu[row][axis] *
            enu_variance[axis] * body_from_enu[col][axis];
        }
      }
    }
    const double var_angular = kMinAngularVelStd * kMinAngularVelStd;
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

  // 投影常量，定原点时算一次（见 onDevpvt）
  double cos_lat0_ = 1.0;
  double rn_ = 0.0;
  double rm_ = 0.0;

  int min_gnss_status_ = 0;

  double roll_offset_deg_ = 0.0;
  double pitch_offset_deg_ = 0.0;
  double yaw_offset_deg_ = 0.0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  std::shared_ptr<ins_diag::Recorder> diagnostics_ = std::make_shared<ins_diag::Recorder>("bridge");
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
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
