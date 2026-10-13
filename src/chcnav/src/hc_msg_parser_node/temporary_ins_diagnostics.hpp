#pragma once
// Temporary observation only: no second serial reader, no protocol/control changes.
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

namespace ins_diag {
enum Metric { READ_CALLS, BYTES, EMPTY_READS, READ_ERRORS, PARSE_ERRORS,
  CGI_FRAMES, NMEA_FRAMES, NOVATEL_FRAMES, NMEA_XOR_ERRORS, NMEA_TAIL_ERRORS,
  CGI_RECEIVED, CRC_OK, CRC_ERRORS, PVT_LENGTH_ERRORS, PVT_PUBLISHED,
  PVT_RECEIVED, GNSS_REJECTED, NONFINITE_REJECTED, STDEV_REJECTED,
  ODOMETRY_PUBLISHED, VELOCITY_PUBLISHED, COUNT };

class Recorder {
  std::array<std::atomic<unsigned long long>, COUNT> counters_;
  std::array<unsigned long long, COUNT> previous_{};
  std::atomic<long long> last_input_{0}, last_output_{0};
  std::atomic<int> gnss_{-1};
  std::atomic<int> message_id_{-1};
  std::ofstream file_;
  std::string stage_;
  long long previous_time_ = 0;
  static long long steady_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
public:
  explicit Recorder(const std::string &stage) : stage_(stage), previous_time_(steady_ms()) {
    for (auto &counter : counters_) counter.store(0);
    const char *directory = std::getenv("CHCNAV_DIAGNOSTIC_DIR");
    if (directory && *directory) {
      const std::string path = std::string(directory) + "/" + stage + ".jsonl";
      file_.open(path, std::ios::app);
      if (!file_) std::cerr << "[INS_DIAG] Cannot open " << path << "; diagnostics disabled\n";
    }
  }
  bool enabled() const { return file_.is_open(); }
  void add(Metric key, unsigned long long amount = 1) {
    if (enabled()) counters_[key].fetch_add(amount, std::memory_order_relaxed);
  }
  void input() { if (enabled()) last_input_.store(steady_ms()); }
  void output() { if (enabled()) last_output_.store(steady_ms()); }
  void gnss(int value) { if (enabled()) gnss_.store(value); }
  void message_id(int value) { if (enabled()) message_id_.store(value); }
  void tick() {
    if (!enabled()) return;
    static const char *names[COUNT] = {"read_calls", "bytes", "empty_reads", "read_errors", "parse_errors",
      "cgi_frames", "nmea_frames", "novatel_frames", "nmea_xor_errors", "nmea_tail_errors",
      "cgi_received", "crc_ok", "crc_errors", "pvt_length_errors", "pvt_published",
      "pvt_received", "gnss_rejected", "nonfinite_rejected", "stdev_rejected",
      "odometry_published", "velocity_published"};
    const auto now = steady_ms();
    const auto epoch = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const auto input = last_input_.load(), output = last_output_.load();
    file_ << "{\"epoch_ms\":" << epoch << ",\"stage\":\"" << stage_
      << "\",\"interval_ms\":" << now - previous_time_
      << ",\"input_age_ms\":" << (input ? now - input : -1)
      << ",\"output_age_ms\":" << (output ? now - output : -1)
      << ",\"gnss_status\":" << gnss_.load() << ",\"message_id\":" << message_id_.load() << ",\"delta\":{";
    std::array<unsigned long long, COUNT> current;
    for (int index = 0; index < COUNT; ++index) {
      current[index] = counters_[index].load(std::memory_order_relaxed);
      file_ << (index ? "," : "") << '"' << names[index] << "\":" << current[index] - previous_[index];
    }
    file_ << "},\"total\":{";
    for (int index = 0; index < COUNT; ++index)
      file_ << (index ? "," : "") << '"' << names[index] << "\":" << current[index];
    file_ << "}}\n";
    file_.flush();
    previous_ = current;
    previous_time_ = now;
  }
};
} // namespace ins_diag
