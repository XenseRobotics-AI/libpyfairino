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
using Clock = std::chrono::steady_clock;
using Deadline = Clock::time_point;
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
enum class DispatchState { NotDispatched, SdkReplied, Unknown, WorkerRejected };
struct CallResult {
  int arm = 0;
  Op operation = Op::ServoJ;
  DispatchState state = DispatchState::NotDispatched;
  std::optional<SdkReply> reply;
  std::string reason;
  bool ok() const { return state == DispatchState::SdkReplied && reply && reply->ok(); }
};
struct FaultRecord {
  std::uint64_t id = 0;
  std::optional<int> arm;
  Op operation = Op::StopMotion;
  FaultOrigin origin = FaultOrigin::Transport;
  std::optional<int> sdk_code;
  std::string reason;
};
struct PairResult {
  PairResult() {
    for (int a = 0; a < 2; ++a) {
      arms[a].arm = a;
      arms[a].operation = Op::ServoJ;
    }
  }
  std::array<CallResult, 2> arms;
  std::uint64_t target_sequence = 0;
  std::string reason;
  bool complete() const { return arms[0].ok() && arms[1].ok(); }
};
struct StopReport {
  StopReport() {
    for (int i = 0; i < 4; ++i) {
      operations[i].arm = i / 2;
      operations[i].operation = i % 2 ? Op::ServoMoveEnd : Op::StopMotion;
    }
  }
  std::array<CallResult, 4> operations;
  std::vector<FaultRecord> faults;
  std::string reason;
  bool ok() const;
};
struct ReapReport {
  bool reaped = false, forced = false;
  std::optional<int> exit_code, term_signal;
  std::string error;
};
struct CleanupReport {
  bool had_worker = false, sdk_destructors_completed = false;
  StopReport stop;
  ReapReport process;
  std::string reason;
  bool ok() const;
};
struct Snapshot {
  Q14 q_rad{};
  std::array<CallResult, 2> arms;
  std::array<std::uint64_t, 2> host_read_begin_ns{}, host_read_end_ns{};
  std::vector<FaultRecord> faults;
  bool valid = false, hardware_freshness_known = false;
};
struct Arm;
class Session : public std::enable_shared_from_this<Session> {
 public:
  explicit Session(HardwareConfig config, std::string worker_executable = "",
                   std::string backend_override = "");
  ~Session();
  Session(const Session&) = delete;
  void connect(bool deny_network = false);
  CleanupReport close();
  StopReport stop_all();
  bool connected() const;
  std::string diagnostic() const;
  std::string log_directory() const;
  int last_worker_pid() const;
  std::vector<FaultRecord> faults() const;
  std::vector<FaultRecord> poll_status();
  CleanupReport last_cleanup() const;
  PairResult last_pair() const;
  Arm arm(int side);
  CallResult call(int arm, Op op, const std::vector<double>& values = {}, int arg0 = 0,
                  double speed = 0, double acceleration = 0, double angle = 0);
  PairResult servo_joints(const Q14&, int period_ms);
  PairResult dispatch_pair(const Q14&, int period_ms, std::uint64_t target_sequence, Deadline,
                           const std::atomic<bool>* stop);
  Snapshot read_state();
  const HardwareConfig& config() const { return config_; }
  Q14 sdk_degrees(const Q14&) const;
  void validate_target(const Q14&) const;

 private:
  Frame prepare_locked(int arm, Op, const std::vector<double>&, int, double, double, double) const;
  Frame exchange_locked(Frame, Deadline);
  void send_locked(Frame&, Deadline, bool& submitted);
  Frame receive_locked(const Frame&, Deadline);
  CallResult dispatch_locked(Frame, Deadline);
  StopReport stop_locked(bool shutdown, Deadline, bool& destructed);
  ReapReport reap_locked(Deadline, bool force);
  void harvest(const Frame&);
  void transport_failure(const std::string&, int, Op);
  void wait_locked(short, Deadline);
  HardwareConfig config_;
  Q14 active_lower_, active_upper_;
  std::string worker_, backend_, diagnostic_, log_directory_;
  mutable std::mutex mutex_, fault_mutex_;
  std::vector<FaultRecord> faults_;
  CleanupReport last_cleanup_;
  PairResult last_pair_;
  int fd_ = -1, pid_ = -1, pidfd_ = -1, last_worker_pid_ = -1;
  std::uint64_t sequence_ = 0;
};
struct Arm {
  std::shared_ptr<Session> session;
  int side;
};
struct ControllerStatus {
  bool running = false, cartesian_in_progress = false, stop_requested = true;
  std::uint64_t target_sequence = 0, sent_sequence = 0, host_send_end_ns = 0;
  Q14 commanded_q_rad{};
  std::string error;
  PairResult last_pair;
  std::array<double, 5> last_cartesian_torso{};
  double last_cartesian_head = 0;
  StopReport stop_result;
  std::vector<FaultRecord> faults;
};
class Controller {
 public:
  Controller(std::shared_ptr<Session>, Q14 initial_q_rad, int period_ms = 5,
             int command_timeout_ms = 500);
  ~Controller();
  void start();
  std::uint64_t submit_joints(const Q14&);
  fairino_ik::Result submit_cartesian(fairino_ik::Model&, const Pose&, const Pose&,
                                      const std::vector<double>&, fairino_ik::Options);
  StopReport stop();
  ControllerStatus status() const;

 private:
  struct Target {
    Q14 q;
    std::uint64_t sequence = 0;
    Deadline submitted, deadline;
  };
  void loop() noexcept;
  std::shared_ptr<Session> session_;
  int period_ms_, watchdog_ms_;
  Q14 initial_;
  Target target_{};
  mutable std::mutex mutex_;
  std::mutex lifecycle_;
  ControllerStatus status_;
  std::atomic<bool> stop_requested_{true}, cartesian_in_progress_{false};
  std::thread thread_;
};
std::uint64_t now_ns();
}  // namespace lpf
