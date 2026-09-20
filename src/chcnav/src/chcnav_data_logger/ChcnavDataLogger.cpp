// ChcnavDataLogger：运行期数据文本日志节点（人看优先）
//
// 本节点为项目自定义开发（不属于厂商 demo 示例）。目标是把运行期间的话题数据
// 落成「人可直接阅读」的文本日志，运行结束后 less / tail -f 即可回看：
//
//   <output_dir>/<YYYYmmdd_HHMMSS>/devpvt.log     话题数据（一帧一个字段分组块）
//   <output_dir>/<YYYYmmdd_HHMMSS>/odometry.log
//   <output_dir>/<YYYYmmdd_HHMMSS>/velocity.log
//   <output_dir>/<YYYYmmdd_HHMMSS>/events.log     事件流（启动/退出/异常/各节点 WARN 及以上）
//   <output_dir>/latest  ->  最新会话目录（软链接，方便直接看当次）
//
// 分类原则：目录分会话，文件名分话题，events.log 单独收事件。
//
// 设计约束（不影响原有功能）：
//   1. 只做订阅，不发布任何话题、不占用 TF、不改变任何现有节点的行为。
//   2. 写盘失败只在 events.log / stderr 提示，节点继续存活，绝不拖垮整条链路。
//   3. 默认订阅三个 FSD 相关话题，可用参数 topics 增减。

#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include <msg_interfaces/msg/bestpos.hpp>
#include <msg_interfaces/msg/hcinspvatzcb.hpp>
#include <msg_interfaces/msg/hcrawimub.hpp>
#include <msg_interfaces/msg/hc_sentence.hpp>
#include <msg_interfaces/msg/heading.hpp>
#include <msg_interfaces/msg/string.hpp>

namespace chcnav_data_logger
{

namespace
{

constexpr int kRosoutWarn = 30;  // rcl_interfaces/msg/Log: DEBUG=10 INFO=20 WARN=30 ERROR=40 FATAL=50

// printf 到 std::string
template <typename... Args>
std::string fmt(const char * format, Args... args)
{
  char buf[2048];
  std::snprintf(buf, sizeof(buf), format, args...);
  return std::string(buf);
}

// 本地时间 "YYYY-mm-dd HH:MM:SS.ffffff"
std::string nowStamp()
{
  const auto now = std::chrono::system_clock::now();
  const auto secs = std::chrono::system_clock::to_time_t(now);
  const long us = static_cast<long>(
    std::chrono::duration_cast<std::chrono::microseconds>(
      now.time_since_epoch()).count() % 1000000);
  std::tm tm{};
  localtime_r(&secs, &tm);
  char date[32];
  std::strftime(date, sizeof(date), "%Y-%m-%d %H:%M:%S", &tm);
  return fmt("%s.%06ld", date, us);
}

// 会话目录名 "YYYYmmdd_HHMMSS"
std::string sessionStamp()
{
  const auto now = std::chrono::system_clock::now();
  const auto secs = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&secs, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
  return std::string(buf);
}

// 递归建目录（已存在不算错）
bool makeDirs(const std::string & path)
{
  if (path.empty()) {
    return false;
  }
  std::string cur;
  for (size_t i = 0; i < path.size(); ++i) {
    cur += path[i];
    if (path[i] == '/' || i + 1 == path.size()) {
      if (cur == "/" || cur.empty()) {
        continue;
      }
      if (::mkdir(cur.c_str(), 0755) != 0 && errno != EEXIST) {
        return false;
      }
    }
  }
  return true;
}

double stampSec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

// 追加一行 "  标签       = 内容"
template <typename... Args>
void put(std::string & out, const char * label, const char * format, Args... args)
{
  out += fmt("  %-10s = ", label);
  out += fmt(format, args...);
  out += '\n';
}

// 二进制转十六进制字符串，最多打印 max_bytes 字节
std::string bytesToHex(const int8_t * data, size_t size, size_t max_bytes)
{
  static const char * kHex = "0123456789ABCDEF";
  const size_t n = std::min(size, max_bytes);
  std::string hex;
  hex.reserve(n * 3);
  for (size_t i = 0; i < n; ++i) {
    const unsigned char b = static_cast<unsigned char>(data[i]);
    if (i) {
      hex += ' ';
    }
    hex += kHex[b >> 4];
    hex += kHex[b & 0x0F];
  }
  if (size > n) {
    hex += fmt(" ...(+%zu B)", size - n);
  }
  return hex;
}

// 逗号/空格分隔的清单（launch、命令行、yaml 三种来源都写成一个字符串最省事）
std::vector<std::string> splitList(const std::string & s)
{
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == ',' || c == ' ' || c == '\t' || c == '\n') {
      if (!cur.empty()) {
        out.push_back(cur);
        cur.clear();
      }
    } else {
      cur += c;
    }
  }
  if (!cur.empty()) {
    out.push_back(cur);
  }
  return out;
}

std::string toString(const std::vector<std::string> & items)
{
  std::string s = "[";
  for (size_t i = 0; i < items.size(); ++i) {
    s += (i ? ", " : "");
    s += items[i];
  }
  return s + "]";
}

}  // namespace

// ---------------------------------------------------------------------------
// 单文件写入器：按大小轮转（devpvt.log -> devpvt_part002.log），提供头注释与 flush
// ---------------------------------------------------------------------------
class TextSink
{
public:
  TextSink(std::string dir, std::string base, size_t rotate_bytes)
  : dir_(std::move(dir)), base_(std::move(base)), rotate_bytes_(rotate_bytes) {}

  void setHeader(const std::string & header) {header_ = header;}

  bool valid() const {return !failed_;}

  void write(const std::string & text)
  {
    if (failed_ || text.empty()) {
      return;
    }
    if (!ofs_.is_open() && !openCurrent()) {
      return;
    }
    ofs_ << text;
    written_ += text.size();
    if (!ofs_.good()) {
      fail(fmt("写入失败: %s", path_.c_str()));
      return;
    }
    if (rotate_bytes_ > 0 && written_ >= rotate_bytes_) {
      ofs_.flush();
      ofs_.close();
      ++part_;
      written_ = 0;
    }
  }

  void flush()
  {
    if (ofs_.is_open()) {
      ofs_.flush();
    }
  }

  void close()
  {
    if (ofs_.is_open()) {
      ofs_.flush();
      ofs_.close();
    }
  }

  const std::string & lastError() const {return error_;}

private:
  bool openCurrent()
  {
    path_ = (part_ <= 1)
      ? fmt("%s/%s.log", dir_.c_str(), base_.c_str())
      : fmt("%s/%s_part%03d.log", dir_.c_str(), base_.c_str(), part_);
    ofs_.open(path_, std::ios::out | std::ios::app);
    if (!ofs_.is_open()) {
      fail(fmt("无法打开日志文件: %s", path_.c_str()));
      return false;
    }
    if (!header_.empty()) {
      ofs_ << header_;
    }
    return true;
  }

  void fail(const std::string & why)
  {
    failed_ = true;
    error_ = why;
    std::fprintf(stderr, "[chcnav_data_logger] %s\n", why.c_str());
  }

  std::string dir_;
  std::string base_;
  std::string header_;
  std::string path_;
  std::string error_;
  std::ofstream ofs_;
  size_t rotate_bytes_ = 0;
  size_t written_ = 0;
  int part_ = 1;
  bool failed_ = false;
};

// ---------------------------------------------------------------------------
// 每个话题一个写入器 + 帧计数（帧计数用于事件汇总与 dt 计算）
// ---------------------------------------------------------------------------
struct TopicLogger
{
  std::string name;
  std::shared_ptr<TextSink> sink;
  uint64_t count = 0;
  rclcpp::Time last_recv;
  bool has_last = false;

  // 头行：[本地时间] 话题名  #帧号  dt=与上一帧的接收间隔
  std::string headLine(rclcpp::Node & node)
  {
    ++count;
    const rclcpp::Time now = node.now();
    double dt_ms = 0.0;
    const bool first = !has_last;
    if (!first) {
      dt_ms = (now - last_recv).seconds() * 1000.0;
    }
    last_recv = now;
    has_last = true;
    return first
      ? fmt("[%s] %-14s #%llu\n", nowStamp().c_str(), name.c_str(),
          static_cast<unsigned long long>(count))
      : fmt("[%s] %-14s #%llu  dt=%.1fms\n", nowStamp().c_str(), name.c_str(),
          static_cast<unsigned long long>(count), dt_ms);
  }
};

// ---------------------------------------------------------------------------
// 话题清单：短名 <-> 实际话题名
// ---------------------------------------------------------------------------
enum class Kind
{
  Devpvt, Devimu, HcSentence, Nmea, Bestpos, Heading, Odometry, Velocity
};

struct TopicSpec
{
  const char * name;
  const char * topic;
  Kind kind;
};

const TopicSpec kTopicTable[] = {
  {"devpvt", "/chcnav/devpvt", Kind::Devpvt},
  {"devimu", "/chcnav/devimu", Kind::Devimu},
  {"hc_sentence", "/chcnav/hc_sentence", Kind::HcSentence},
  {"nmea_sentence", "/chcnav/nmea_sentence", Kind::Nmea},
  {"bestpos", "/chcnav/bestpos", Kind::Bestpos},
  {"heading", "/chcnav/heading", Kind::Heading},
  {"odometry", "/chcnav/odometry", Kind::Odometry},
  {"velocity", "/chcnav/velocity", Kind::Velocity},
};

std::string trimTopic(const std::string & s)
{
  const size_t b = s.find_first_not_of(" \t");
  if (b == std::string::npos) {
    return "";
  }
  const size_t e = s.find_last_not_of(" \t");
  return s.substr(b, e - b + 1);
}

// ---------------------------------------------------------------------------
class ChcnavDataLogger : public rclcpp::Node
{
public:
  ChcnavDataLogger()
  : Node("chcnav_data_logger")
  {
    output_dir_ = declare_parameter<std::string>("output_dir", "log");
    style_ = declare_parameter<std::string>("style", "brief");
    // topics 用逗号分隔字符串，launch / 命令行 / yaml 三种写法都稳（不接受列表型参数）
    topics_ = splitList(declare_parameter<std::string>("topics", "devpvt,odometry,velocity"));
    flush_interval_s_ = declare_parameter<double>("flush_interval_s", 1.0);
    rotate_max_mb_ = declare_parameter<int>("rotate_max_mb", 200);
    enable_rosout_ = declare_parameter<bool>("rosout", true);

    full_ = (style_ == "full");
    rotate_bytes_ = static_cast<size_t>(std::max(rotate_max_mb_, 1)) * 1024 * 1024;

    session_dir_ = output_dir_ + "/" + sessionStamp();
    if (!makeDirs(session_dir_)) {
      RCLCPP_ERROR(
        get_logger(), "无法创建日志目录 %s，本次不做任何记录（不影响其它节点）",
        session_dir_.c_str());
      session_dir_.clear();
      return;
    }
    updateLatestLink();

    const std::string boot = fmt(
      "# chcnav 数据日志 | 会话 %s | 样式 %s | 话题 %s | 轮转 %d MB | 每 %.1fs flush\n",
      sessionStamp().c_str(), style_.c_str(), toString(topics_).c_str(),
      rotate_max_mb_, flush_interval_s_);

    events_ = std::make_shared<TextSink>(session_dir_, "events", rotate_bytes_);
    events_->setHeader(boot);
    event("INFO", "logger", fmt(
      "启动: 目录=%s 样式=%s 话题=%s rosout=%s",
      session_dir_.c_str(), style_.c_str(), toString(topics_).c_str(),
      enable_rosout_ ? "on" : "off"));

    for (const auto & raw : topics_) {
      addTopic(trimTopic(raw), boot);
    }

    if (enable_rosout_) {
      rosout_sub_ = create_subscription<rcl_interfaces::msg::Log>(
        "/rosout", rclcpp::QoS(200),
        [this](rcl_interfaces::msg::Log::ConstSharedPtr msg) {onRosout(msg);});
    }

    const auto period = std::chrono::milliseconds(
      static_cast<int64_t>(std::max(flush_interval_s_, 0.1) * 1000.0));
    flush_timer_ = create_wall_timer(period, [this]() {flushAll();});

    RCLCPP_INFO(
      get_logger(), "数据日志已开启: %s (样式 %s, 话题 %s)",
      session_dir_.c_str(), style_.c_str(), toString(topics_).c_str());
  }

  ~ChcnavDataLogger() override
  {
    if (session_dir_.empty()) {
      return;
    }
    std::string summary;
    for (const auto & tl : topic_loggers_) {
      summary += fmt(" %s=%llu", tl->name.c_str(),
        static_cast<unsigned long long>(tl->count));
    }
    event("INFO", "logger", fmt("退出: 各话题帧数%s", summary.c_str()));
    flushAll();
    if (events_) {
      events_->close();
    }
    for (auto & tl : topic_loggers_) {
      if (tl->sink) {
        tl->sink->close();
      }
    }
  }

private:
  void updateLatestLink()
  {
    const std::string link = output_dir_ + "/latest";
    ::unlink(link.c_str());
    ::symlink(session_dir_.c_str(), link.c_str());
  }

  void event(const char * level, const std::string & source, const std::string & text)
  {
    if (events_) {
      events_->write(
        fmt("[%s] %-5s %s | %s\n", nowStamp().c_str(), level, source.c_str(), text.c_str()));
    }
  }

  void flushAll()
  {
    if (events_) {
      events_->flush();
    }
    for (auto & tl : topic_loggers_) {
      if (tl->sink) {
        tl->sink->flush();
      }
    }
  }

  bool findSpec(const std::string & raw, TopicSpec & out) const
  {
    std::string key = trimTopic(raw);
    if (!key.empty() && key[0] == '/') {
      key = key.substr(1);
    }
    for (const auto & spec : kTopicTable) {
      if (key == spec.name || key == spec.topic || ("/" + key) == spec.topic) {
        out = spec;
        return true;
      }
    }
    return false;
  }

  void addTopic(const std::string & raw, const std::string & boot)
  {
    TopicSpec spec{};
    if (!findSpec(raw, spec)) {
      RCLCPP_WARN(
        get_logger(),
        "未知话题 '%s'，已忽略。可选: devpvt devimu hc_sentence nmea_sentence "
        "bestpos heading odometry velocity", raw.c_str());
      event("WARN", "logger", fmt("忽略未知话题 '%s'", raw.c_str()));
      return;
    }

    auto tl = std::make_shared<TopicLogger>();
    tl->name = spec.name;
    tl->sink = std::make_shared<TextSink>(session_dir_, spec.name, rotate_bytes_);
    tl->sink->setHeader(boot);
    topic_loggers_.push_back(tl);

    const std::string topic = spec.topic;
    switch (spec.kind) {
      case Kind::Devpvt:
        subs_.push_back(create_subscription<msg_interfaces::msg::Hcinspvatzcb>(
          topic, rclcpp::QoS(1000),
          [this, tl](msg_interfaces::msg::Hcinspvatzcb::ConstSharedPtr m) {onDevpvt(tl, *m);}));
        break;
      case Kind::Devimu:
        subs_.push_back(create_subscription<msg_interfaces::msg::Hcrawimub>(
          topic, rclcpp::QoS(1000),
          [this, tl](msg_interfaces::msg::Hcrawimub::ConstSharedPtr m) {onDevimu(tl, *m);}));
        break;
      case Kind::HcSentence:
        subs_.push_back(create_subscription<msg_interfaces::msg::HcSentence>(
          topic, rclcpp::QoS(1000),
          [this, tl](msg_interfaces::msg::HcSentence::ConstSharedPtr m) {onHcSentence(tl, *m);}));
        break;
      case Kind::Nmea:
        subs_.push_back(create_subscription<msg_interfaces::msg::String>(
          topic, rclcpp::QoS(1000),
          [this, tl](msg_interfaces::msg::String::ConstSharedPtr m) {onNmea(tl, *m);}));
        break;
      case Kind::Bestpos:
        subs_.push_back(create_subscription<msg_interfaces::msg::BESTPOS>(
          topic, rclcpp::QoS(1000),
          [this, tl](msg_interfaces::msg::BESTPOS::ConstSharedPtr m) {onBestpos(tl, *m);}));
        break;
      case Kind::Heading:
        subs_.push_back(create_subscription<msg_interfaces::msg::HEADING>(
          topic, rclcpp::QoS(1000),
          [this, tl](msg_interfaces::msg::HEADING::ConstSharedPtr m) {onHeading(tl, *m);}));
        break;
      case Kind::Odometry:
        subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
          topic, rclcpp::QoS(1000),
          [this, tl](nav_msgs::msg::Odometry::ConstSharedPtr m) {onOdometry(tl, *m);}));
        break;
      case Kind::Velocity:
        subs_.push_back(create_subscription<geometry_msgs::msg::TwistStamped>(
          topic, rclcpp::QoS(1000),
          [this, tl](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {onVelocity(tl, *m);}));
        break;
    }
  }

  // ---------------- /rosout：把各节点 WARN 及以上收进 events.log ----------------
  void onRosout(const rcl_interfaces::msg::Log::ConstSharedPtr & msg)
  {
    if (msg->level < kRosoutWarn) {
      return;
    }
    const char * level = "WARN";
    if (msg->level >= 50) {
      level = "FATAL";
    } else if (msg->level >= 40) {
      level = "ERROR";
    }
    event(level, msg->name, msg->msg);
  }

  // ---------------- 各话题的人读格式 ----------------
  void onDevpvt(const std::shared_ptr<TopicLogger> & tl, const msg_interfaces::msg::Hcinspvatzcb & m)
  {
    std::string out = tl->headLine(*this);
    put(out, "stamp", "week %u  second %.9f", static_cast<unsigned>(m.week), m.second);
    put(out, "lla", "lon %.9f  lat %.9f  alt %.3f", m.longitude, m.latitude,
      static_cast<double>(m.altitude));
    put(out, "rpy", "roll %.3f  pitch %.3f  yaw %.3f  (deg)",
      static_cast<double>(m.roll), static_cast<double>(m.pitch), static_cast<double>(m.yaw));
    put(out, "course", "speed %.3f  heading %.3f/%.3f  (deg)",
      static_cast<double>(m.speed), static_cast<double>(m.heading),
      static_cast<double>(m.heading2));
    put(out, "enu_vel", "e %.3f  n %.3f  u %.3f", m.enu_velocity.x, m.enu_velocity.y,
      m.enu_velocity.z);
    put(out, "veh_vel", "x %.3f  y %.3f  z %.3f", m.vehicle_linear_velocity.x,
      m.vehicle_linear_velocity.y, m.vehicle_linear_velocity.z);
    put(out, "veh_acc", "x %.3f  y %.3f  z %.3f", m.vehicle_linear_acceleration.x,
      m.vehicle_linear_acceleration.y, m.vehicle_linear_acceleration.z);
    put(out, "veh_gyro", "x %.3f  y %.3f  z %.3f  (deg/s)",
      m.vehicle_angular_velocity.x, m.vehicle_angular_velocity.y, m.vehicle_angular_velocity.z);
    put(out, "status", "stat [%u, %u]  age %.1f  ns %u/%u  leaps %u",
      static_cast<unsigned>(m.stat[0]), static_cast<unsigned>(m.stat[1]),
      static_cast<double>(m.age), static_cast<unsigned>(m.ns),
      static_cast<unsigned>(m.ns2), static_cast<unsigned>(m.leaps));
    put(out, "dop", "h %.2f  p %.2f  v %.2f  t %.2f  g %.2f",
      static_cast<double>(m.hdop), static_cast<double>(m.pdop), static_cast<double>(m.vdop),
      static_cast<double>(m.tdop), static_cast<double>(m.gdop));
    put(out, "stdev", "pos %.3f/%.3f/%.3f  euler %.3f/%.3f/%.3f",
      static_cast<double>(m.position_stdev[0]), static_cast<double>(m.position_stdev[1]),
      static_cast<double>(m.position_stdev[2]), static_cast<double>(m.euler_stdev[0]),
      static_cast<double>(m.euler_stdev[1]), static_cast<double>(m.euler_stdev[2]));
    put(out, "info", "warning %u  sensor_used %u",
      static_cast<unsigned>(m.warning), static_cast<unsigned>(m.sensor_used));

    if (full_) {
      put(out, "undulation", "%.3f", static_cast<double>(m.undulation));
      put(out, "enu_vel_stdev", "%.3f/%.3f/%.3f",
        static_cast<double>(m.enu_velocity_stdev[0]),
        static_cast<double>(m.enu_velocity_stdev[1]),
        static_cast<double>(m.enu_velocity_stdev[2]));
      put(out, "acc_no_g", "x %.3f  y %.3f  z %.3f",
        m.vehicle_linear_acceleration_without_g.x,
        m.vehicle_linear_acceleration_without_g.y,
        m.vehicle_linear_acceleration_without_g.z);
      put(out, "raw_gyro", "x %.3f  y %.3f  z %.3f",
        m.raw_angular_velocity.x, m.raw_angular_velocity.y, m.raw_angular_velocity.z);
      put(out, "raw_acc", "x %.3f  y %.3f  z %.3f",
        m.raw_acceleration.x, m.raw_acceleration.y, m.raw_acceleration.z);
      put(out, "ins2gnss", "x %.4f  y %.4f  z %.4f",
        m.ins2gnss_vector.x, m.ins2gnss_vector.y, m.ins2gnss_vector.z);
      put(out, "ins2body", "x %.4f  y %.4f  z %.4f  (deg)",
        m.ins2body_angle.x, m.ins2body_angle.y, m.ins2body_angle.z);
      put(out, "gnss2body", "x %.4f  y %.4f  z %.4f  yaw_z %.4f",
        m.gnss2body_vector.x, m.gnss2body_vector.y, m.gnss2body_vector.z,
        static_cast<double>(m.gnss2body_angle_z));
      std::vector<int8_t> recv(m.receiver.begin(), m.receiver.end());
      put(out, "receiver", "%s", bytesToHex(recv.data(), recv.size(), recv.size()).c_str());
    }
    out += '\n';
    tl->sink->write(out);
  }

  void onDevimu(const std::shared_ptr<TopicLogger> & tl, const msg_interfaces::msg::Hcrawimub & m)
  {
    std::string out = tl->headLine(*this);
    put(out, "stamp", "week %u  second %.9f", static_cast<unsigned>(m.week), m.second);
    put(out, "gyro", "x %.3f  y %.3f  z %.3f  (deg/s)",
      m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z);
    put(out, "acc", "x %.3f  y %.3f  z %.3f",
      m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z);
    put(out, "temp", "%.1f C", static_cast<double>(m.temp));
    put(out, "status", "err %d  yaw %.2f",
      static_cast<int>(m.err_status), static_cast<double>(m.yaw) * 0.01);
    out += '\n';
    tl->sink->write(out);
  }

  void onHcSentence(
    const std::shared_ptr<TopicLogger> & tl, const msg_interfaces::msg::HcSentence & m)
  {
    std::string out = tl->headLine(*this);
    put(out, "msg_id", "0x%04X (%d)", static_cast<unsigned>(m.msg_id) & 0xFFFF,
      static_cast<int>(m.msg_id));
    put(out, "length", "%zu B", m.data.size());
    put(out, "data", "%s", bytesToHex(m.data.data(), m.data.size(), 64).c_str());
    out += '\n';
    tl->sink->write(out);
  }

  void onNmea(const std::shared_ptr<TopicLogger> & tl, const msg_interfaces::msg::String & m)
  {
    std::string out = tl->headLine(*this);
    put(out, "sentence", "%s", m.sentence.c_str());
    out += '\n';
    tl->sink->write(out);
  }

  void onBestpos(const std::shared_ptr<TopicLogger> & tl, const msg_interfaces::msg::BESTPOS & m)
  {
    std::string out = tl->headLine(*this);
    put(out, "sol", "status %u  pos_type %u  ext 0x%02X",
      static_cast<unsigned>(m.sol_status.value), static_cast<unsigned>(m.pos_type.value),
      static_cast<unsigned>(m.ext_sol_stat.value));
    put(out, "pos", "lat %.9f  lon %.9f  hgt %.3f", m.lat, m.lon,
      static_cast<double>(m.hgt));
    put(out, "stdev", "lat %.3f  lon %.3f  hgt %.3f  undulation %.3f",
      static_cast<double>(m.lat_stdev), static_cast<double>(m.lon_stdev),
      static_cast<double>(m.hgt_stdev), static_cast<double>(m.undulation));
    put(out, "age", "diff %.1f  sol %.1f  datum %u",
      static_cast<double>(m.diff_age), static_cast<double>(m.sol_age),
      static_cast<unsigned>(m.datum_id));
    put(out, "svs", "tracked %u  used %u  l1 %u  multi %u",
      static_cast<unsigned>(m.num_svs), static_cast<unsigned>(m.num_sol_svs),
      static_cast<unsigned>(m.num_sol_l1_svs), static_cast<unsigned>(m.num_sol_multi_svs));
    if (full_) {
      put(out, "arc_header", "msg_id %u  seq %u  gps_week %u  gps_ms %u  sw %u",
        static_cast<unsigned>(m.arc_header.msg_id), static_cast<unsigned>(m.arc_header.sequence),
        static_cast<unsigned>(m.arc_header.gps_week), static_cast<unsigned>(m.arc_header.gps_ms),
        static_cast<unsigned>(m.arc_header.sw_version));
      std::vector<int8_t> stn(m.stn_id.begin(), m.stn_id.end());
      put(out, "stn_id", "%s", bytesToHex(stn.data(), stn.size(), stn.size()).c_str());
      put(out, "misc", "reserved %u  reserved1 %u  sig_mask 0x%02X",
        static_cast<unsigned>(m.reserved), static_cast<unsigned>(m.reserved1),
        static_cast<unsigned>(m.sig_mask));
    }
    out += '\n';
    tl->sink->write(out);
  }

  void onHeading(const std::shared_ptr<TopicLogger> & tl, const msg_interfaces::msg::HEADING & m)
  {
    std::string out = tl->headLine(*this);
    put(out, "sol", "status %u  pos_type %u  ext 0x%02X",
      static_cast<unsigned>(m.sol_status.value), static_cast<unsigned>(m.pos_type.value),
      static_cast<unsigned>(m.ext_sol_status.value));
    put(out, "angle", "heading %.3f  pitch %.3f  (deg)",
      static_cast<double>(m.heading), static_cast<double>(m.pitch));
    put(out, "baseline", "length %.3f m  heading_stdev %.3f  pitch_stdev %.3f",
      static_cast<double>(m.length), static_cast<double>(m.heading_stdev),
      static_cast<double>(m.pitch_stdev));
    put(out, "svs", "tracked %u  used %u  obs %u  multi %u",
      static_cast<unsigned>(m.num_sv_tracked), static_cast<unsigned>(m.num_sv_in_sol),
      static_cast<unsigned>(m.num_sv_obs), static_cast<unsigned>(m.num_sv_multi));
    if (full_) {
      put(out, "rover_stn_id", "%s", m.rover_stn_id.c_str());
      put(out, "arc_header", "msg_id %u  seq %u  gps_week %u  gps_ms %u  sw %u",
        static_cast<unsigned>(m.arc_header.msg_id), static_cast<unsigned>(m.arc_header.sequence),
        static_cast<unsigned>(m.arc_header.gps_week), static_cast<unsigned>(m.arc_header.gps_ms),
        static_cast<unsigned>(m.arc_header.sw_version));
      put(out, "misc", "reserved %.3f  reserved1 %u  reserved2 %u  sig_mask 0x%02X",
        static_cast<double>(m.reserved), static_cast<unsigned>(m.reserved1),
        static_cast<unsigned>(m.reserved2), static_cast<unsigned>(m.sig_mask));
    }
    out += '\n';
    tl->sink->write(out);
  }

  void onOdometry(const std::shared_ptr<TopicLogger> & tl, const nav_msgs::msg::Odometry & m)
  {
    std::string out = tl->headLine(*this);
    put(out, "stamp", "%.9f", stampSec(m.header.stamp));
    put(out, "frame", "%s -> %s", m.header.frame_id.c_str(), m.child_frame_id.c_str());
    put(out, "pos", "x %.3f  y %.3f  z %.3f",
      m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z);
    put(out, "quat", "x %.4f  y %.4f  z %.4f  w %.4f",
      m.pose.pose.orientation.x, m.pose.pose.orientation.y,
      m.pose.pose.orientation.z, m.pose.pose.orientation.w);
    put(out, "twist", "vx %.3f  vy %.3f  vz %.3f  |  wx %.4f  wy %.4f  wz %.4f",
      m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z,
      m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z);
    put(out, "cov_diag", "x %.5g  y %.5g  z %.5g  roll %.5g  pitch %.5g  yaw %.5g",
      m.pose.covariance[0], m.pose.covariance[7], m.pose.covariance[14],
      m.pose.covariance[21], m.pose.covariance[28], m.pose.covariance[35]);
    if (full_) {
      put(out, "cov_pose", "%s", nonzeroCov(m.pose.covariance).c_str());
      put(out, "cov_twist", "%s", nonzeroCov(m.twist.covariance).c_str());
    }
    out += '\n';
    tl->sink->write(out);
  }

  void onVelocity(
    const std::shared_ptr<TopicLogger> & tl, const geometry_msgs::msg::TwistStamped & m)
  {
    std::string out = tl->headLine(*this);
    put(out, "stamp", "%.9f", stampSec(m.header.stamp));
    put(out, "frame", "%s", m.header.frame_id.c_str());
    put(out, "linear", "x %.3f  y %.3f  z %.3f",
      m.twist.linear.x, m.twist.linear.y, m.twist.linear.z);
    put(out, "angular", "x %.4f  y %.4f  z %.4f",
      m.twist.angular.x, m.twist.angular.y, m.twist.angular.z);
    out += '\n';
    tl->sink->write(out);
  }

  // 协方差里的非零项，形如 "0:0.0025 7:0.0025 14:0.01"
  static std::string nonzeroCov(const std::array<double, 36> & cov)
  {
    std::string s;
    for (size_t i = 0; i < cov.size(); ++i) {
      if (cov[i] != 0.0) {
        s += fmt("%s%zu:%.5g", s.empty() ? "" : " ", i, cov[i]);
      }
    }
    return s.empty() ? "全 0" : s;
  }

  std::string output_dir_;
  std::string style_;
  std::vector<std::string> topics_;
  double flush_interval_s_ = 1.0;
  int rotate_max_mb_ = 200;
  bool enable_rosout_ = true;
  bool full_ = false;
  size_t rotate_bytes_ = 0;
  std::string session_dir_;

  std::shared_ptr<TextSink> events_;
  std::vector<std::shared_ptr<TopicLogger>> topic_loggers_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
};

}  // namespace chcnav_data_logger

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<chcnav_data_logger::ChcnavDataLogger>();
  rclcpp::spin(node);
  node.reset();          // 先析构（写盘收尾），再关 ROS
  rclcpp::shutdown();
  return 0;
}
