#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "fairino_ik/solver.hpp"
#include "protocol.hpp"
namespace lpf {
using Q7 = std::array<double, 7>;
using Q14 = std::array<double, 14>;
using Pose = std::array<double, 16>;
struct HardwareConfig {
  int sdk_robot_type;
  Q14 lower_deg, upper_deg, signs, offsets_deg, max_velocity_rad_s;
  std::array<int, 14> permutation;
  std::string model_compatibility_evidence, joint_mapping_evidence;
  std::array<double, 28> left_dh_compensation{}, right_dh_compensation{};
  int rpc_timeout_ms = 2000, worker_watchdog_ms = 500;
  std::optional<std::array<double, 5>> fixed_torso_rad;
  std::optional<double> fixed_head_rad;
  std::string cartesian_frame_evidence;
  void validate() const;
};
struct SdkReply {
  int code = 0;
  std::string error;
  std::vector<double> values;
  double angle = 0;
  std::uint8_t state = 0;
  bool ok() const { return code == 0; }
};
struct Snapshot {
  Q14 q_rad{};
  std::array<SdkReply, 2> arms;
  std::array<std::uint64_t, 2> host_read_begin_ns{}, host_read_end_ns{};
  bool valid = false;
  bool hardware_freshness_known = false;
};
struct Arm;
class Session : public std::enable_shared_from_this<Session> {
 public:
  explicit Session(HardwareConfig config, std::string worker_executable = "",
                   std::string backend_override = "");
  ~Session();
  Session(const Session&) = delete;
  void connect(bool deny_network = false);
  void close();
  bool connected() const;
  std::string diagnostic() const;
  std::string log_directory() const;
  int last_worker_pid() const;
  Arm arm(int side);
  SdkReply call(int arm, Op op, const std::vector<double>& values = {}, int arg0 = 0,
                double speed = 0, double acceleration = 0, double angle = 0);
  std::array<SdkReply, 2> servo_joints(const Q14& model_q_rad, int period_ms);
  Snapshot read_state();
  const HardwareConfig& config() const { return config_; }
  Q14 sdk_degrees(const Q14&) const;
  void validate_target(const Q14&) const;

 private:
  Frame exchange_locked(Frame request);
  void reap_locked() noexcept;
  HardwareConfig config_;
  Q14 active_lower_, active_upper_;
  std::string worker_, backend_, diagnostic_, log_directory_;
  mutable std::mutex mutex_;
  int fd_ = -1, pid_ = -1, pidfd_ = -1, last_worker_pid_ = -1;
  std::uint64_t sequence_ = 0;
};
struct Arm {
  std::shared_ptr<Session> session;
  int side;
};
struct ControllerStatus {
  bool running = false;
  std::uint64_t target_sequence = 0, sent_sequence = 0, host_send_end_ns = 0;
  Q14 commanded_q_rad{};
  std::string error;
};
class Controller {
 public:
  Controller(std::shared_ptr<Session>, Q14 initial_q_rad, int period_ms = 5,
             int command_timeout_ms = 500);
  ~Controller();
  void start();
  std::uint64_t submit_joints(const Q14&);
  fairino_ik::Result submit_cartesian(fairino_ik::Model&, const Pose&, const Pose&,
                                      const std::vector<double>&, const fairino_ik::Options&);
  void stop();
  ControllerStatus status() const;

 private:
  void loop() noexcept;
  std::shared_ptr<Session> session_;
  int period_ms_, watchdog_ms_;
  Q14 initial_, target_;
  mutable std::mutex mutex_;
  std::mutex lifecycle_;
  std::chrono::steady_clock::time_point last_submit_;
  ControllerStatus status_;
  std::atomic<bool> stop_requested_{true};
  std::thread thread_;
};
std::uint64_t now_ns();
}  // namespace lpf
