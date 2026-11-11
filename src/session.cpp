#define _GNU_SOURCE
#include "session.hpp"

#include <dlfcn.h>
#include <poll.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <stdexcept>
extern char** environ;
namespace lpf {
namespace {
constexpr double pi = 3.14159265358979323846;
void require(bool ok, const char* message) {
  if (!ok) throw std::invalid_argument(message);
}
template <class A>
void finite(const A& a) {
  for (double x : a) require(std::isfinite(x), "all numbers must be finite");
}
std::string package_directory() {
  Dl_info info{};
  if (!dladdr(reinterpret_cast<void*>(&now_ns), &info))
    throw std::runtime_error("cannot locate native module");
  return std::filesystem::absolute(info.dli_fname).parent_path();
}
}  // namespace
std::uint64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
void HardwareConfig::validate() const {
  require(sdk_robot_type >= 0 && sdk_robot_type <= 3,
          "choose an actual official SDK enum explicitly; no ART7 enum exists");
  require(!model_compatibility_evidence.empty() && !joint_mapping_evidence.empty(),
          "explicit model compatibility and joint mapping evidence are required");
  finite(lower_deg);
  finite(upper_deg);
  finite(signs);
  finite(offsets_deg);
  finite(max_velocity_rad_s);
  finite(left_dh_compensation);
  finite(right_dh_compensation);
  for (int side = 0; side < 2; ++side) {
    std::array<bool, 7> seen{};
    for (int j = 0; j < 7; ++j) {
      int k = side * 7 + j;
      require(permutation[k] >= 0 && permutation[k] < 7, "each arm permutation uses indices 0..6");
      require(!seen[permutation[k]], "each arm permutation must be bijective");
      seen[permutation[k]] = true;
      require(signs[k] == 1 || signs[k] == -1, "joint signs must be +1 or -1");
      require(lower_deg[k] < upper_deg[k], "lower SDK degree limits must be below upper limits");
      require(max_velocity_rad_s[k] > 0, "model joint velocity caps must be positive");
    }
  }
  if (fixed_torso_rad || fixed_head_rad || !cartesian_frame_evidence.empty()) {
    require(fixed_torso_rad.has_value() && fixed_head_rad.has_value() &&
                !cartesian_frame_evidence.empty(),
            "Cartesian control requires explicit fixed torso, head and frame evidence together");
    finite(*fixed_torso_rad);
    require(std::isfinite(*fixed_head_rad), "fixed head angle must be finite");
  }
  require(rpc_timeout_ms >= 1 && rpc_timeout_ms <= 60000, "RPC timeout must be 1..60000 ms");
  require(worker_watchdog_ms >= 10 && worker_watchdog_ms <= 60000,
          "worker watchdog must be 10..60000 ms");
}
Session::Session(HardwareConfig config, std::string worker, std::string backend)
    : config_(std::move(config)), worker_(std::move(worker)), backend_(std::move(backend)) {
  config_.validate();
  active_lower_ = config_.lower_deg;
  active_upper_ = config_.upper_deg;
  if (worker_.empty()) worker_ = package_directory() + "/_sdk_worker";
  if (backend_.empty()) backend_ = package_directory() + "/_sdk_backend.so";
}
Session::~Session() {
  try {
    close();
  } catch (...) {
  }
}
bool Session::connected() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return fd_ >= 0;
}
std::string Session::diagnostic() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return diagnostic_;
}
std::string Session::log_directory() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return log_directory_;
}
int Session::last_worker_pid() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_worker_pid_;
}
Arm Session::arm(int side) {
  require(side == 0 || side == 1, "side must be 0 or 1");
  return {shared_from_this(), side};
}
void Session::reap_locked() noexcept {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  if (pid_ > 0) {
    if (pidfd_ >= 0) syscall(SYS_pidfd_send_signal, pidfd_, SIGKILL, nullptr, 0);
    int status;
    while (waitpid(pid_, &status, 0) < 0 && errno == EINTR) {
    }
    pid_ = -1;
  }
  if (pidfd_ >= 0) {
    ::close(pidfd_);
    pidfd_ = -1;
  }
}
Frame Session::exchange_locked(Frame request) {
  if (fd_ < 0) throw std::runtime_error("SDK session is not connected");
  request.sequence = ++sequence_;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(config_.rpc_timeout_ms);
  auto wait = [&](short events) {
    for (;;) {
      auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
                      deadline - std::chrono::steady_clock::now())
                      .count();
      if (left <= 0)
        throw std::runtime_error("SDK worker request timed out; physical stop is unconfirmed");
      pollfd p{fd_, events, 0};
      int rc = poll(&p, 1, static_cast<int>(left));
      if (rc < 0 && errno == EINTR) continue;
      if (rc == 0)
        throw std::runtime_error("SDK worker request timed out; physical stop is unconfirmed");
      if (rc < 0 || (p.revents & (POLLERR | POLLNVAL)))
        throw std::runtime_error("SDK worker IPC failed");
      if (p.revents & (events | POLLHUP)) return;
    }
  };
  try {
    for (;;) {
      wait(POLLOUT);
      auto n = send(fd_, &request, sizeof(request), MSG_NOSIGNAL | MSG_DONTWAIT);
      if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
      if (n != sizeof(request)) throw std::runtime_error("SDK worker send failed");
      break;
    }
    Frame response;
    for (;;) {
      wait(POLLIN);
      auto n = recv(fd_, &response, sizeof(response), MSG_DONTWAIT | MSG_TRUNC);
      if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
      if (n == 0)
        throw std::runtime_error(
            "SDK worker exited before a valid reply (exit code is not a health handshake)");
      if (n != sizeof(response)) throw std::runtime_error("invalid SDK worker frame size");
      break;
    }
    if (!valid_frame(response) || response.sequence != request.sequence ||
        response.op != request.op || response.arm != request.arm ||
        (response.tag != Tag::Reply && response.tag != Tag::Ready))
      throw std::runtime_error("invalid SDK worker reply identity");
    return response;
  } catch (const std::exception& e) {
    diagnostic_ = e.what();
    reap_locked();
    throw;
  }
}
void Session::connect(bool deny_network) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) throw std::runtime_error("SDK session already connected");
  int sv[2];
  if (socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0, sv))
    throw std::runtime_error("socketpair failed");
  char path[] = "/tmp/libpyfairino-session-XXXXXX";
  if (!mkdtemp(path)) {
    ::close(sv[0]);
    ::close(sv[1]);
    throw std::runtime_error("cannot create isolated SDK log directory");
  }
  log_directory_ = path;
  // Duplicate to a descriptor outside both socketpair descriptors; dup2 clears CLOEXEC.
  const int child_fd = std::max(sv[0], sv[1]) + 1;
  const std::string fd_string = std::to_string(child_fd);
  std::vector<std::string> args{worker_, fd_string, backend_,
                                deny_network ? "deny-network" : "allow-network"};
  std::vector<char*> argv;
  for (auto& a : args) argv.push_back(a.data());
  argv.push_back(nullptr);
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_adddup2(&actions, sv[1], child_fd);
  posix_spawn_file_actions_addclose(&actions, sv[0]);
  posix_spawn_file_actions_addclose(&actions, sv[1]);
  posix_spawn_file_actions_addchdir_np(&actions, log_directory_.c_str());
  pid_t child = -1;
  int rc = posix_spawn(&child, worker_.c_str(), &actions, nullptr, argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  ::close(sv[1]);
  if (rc) {
    ::close(sv[0]);
    diagnostic_ = "cannot spawn SDK worker: " + std::string(strerror(rc));
    throw std::runtime_error(diagnostic_);
  }
  pid_ = child;
  last_worker_pid_ = child;
  fd_ = sv[0];
  pidfd_ = syscall(SYS_pidfd_open, pid_, 0);
  if (pidfd_ < 0) {
    ::close(fd_);
    fd_ = -1;
    int status;
    while (waitpid(pid_, &status, 0) < 0 && errno == EINTR) {
    }
    pid_ = -1;
    throw std::runtime_error(
        "Linux pidfd support is required before permitting SDK initialization");
  }
  char gate = 1;
  if (send(fd_, &gate, 1, MSG_NOSIGNAL) != 1) {
    reap_locked();
    throw std::runtime_error("SDK worker launch gate failed");
  }
  Frame init;
  init.op = Op::Initialize;
  init.arg0 = config_.sdk_robot_type;
  init.arg1 = config_.worker_watchdog_ms;
  init.count = 84;
  std::copy(config_.lower_deg.begin(), config_.lower_deg.end(), init.values.begin());
  std::copy(config_.upper_deg.begin(), config_.upper_deg.end(), init.values.begin() + 14);
  std::copy(config_.left_dh_compensation.begin(), config_.left_dh_compensation.end(),
            init.values.begin() + 28);
  std::copy(config_.right_dh_compensation.begin(), config_.right_dh_compensation.end(),
            init.values.begin() + 56);
  auto ready = exchange_locked(init);
  if (ready.tag != Tag::Ready || ready.code != 0) {
    diagnostic_ = "SDK worker did not complete arm initialization: " +
                  std::string(ready.error.data(), strnlen(ready.error.data(), ready.error.size()));
    reap_locked();
    throw std::runtime_error(diagnostic_);
  }
  active_lower_ = config_.lower_deg;
  active_upper_ = config_.upper_deg;
  diagnostic_.clear();
}
void Session::close() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    reap_locked();
    return;
  }
  Frame f;
  f.op = Op::Shutdown;
  try {
    auto r = exchange_locked(f);
    if (r.code)
      diagnostic_ = "SDK stop/end returned error code " + std::to_string(r.code) + " during close";
  } catch (...) {
    reap_locked();
    throw;
  }
  reap_locked();
}
Q14 Session::sdk_degrees(const Q14& q) const {
  finite(q);
  Q14 result;
  for (int a = 0; a < 2; ++a)
    for (int j = 0; j < 7; ++j) {
      int k = 7 * a + j;
      result[k] =
          config_.signs[k] * q[7 * a + config_.permutation[k]] * 180 / pi + config_.offsets_deg[k];
    }
  return result;
}
void Session::validate_target(const Q14& q) const {
  auto d = sdk_degrees(q);
  for (int k = 0; k < 14; ++k)
    require(d[k] >= config_.lower_deg[k] && d[k] <= config_.upper_deg[k],
            "joint target exceeds confirmed SDK degree limits");
}
SdkReply Session::call(int arm, Op op, const std::vector<double>& values, int arg0, double speed,
                       double acceleration, double angle) {
  require(arm == 0 || arm == 1, "arm must be 0 or 1");
  require(values.size() <= 1024, "SDK payload too large");
  finite(values);
  require(std::isfinite(speed) && std::isfinite(acceleration) && std::isfinite(angle),
          "parameters must be finite");
  if (op == Op::MoveJ || op == Op::MoveP || op == Op::MoveL || op == Op::MoveC) {
    require(speed >= 0 && speed <= 100, "velocity percentage must be 0..100");
    require(acceleration >= 0 && acceleration <= 100, "acceleration percentage must be 0..100");
  }
  if (op == Op::SetSpeed || op == Op::SetAccScale)
    require(speed >= 0 && speed <= 100, "percentage must be 0..100");
  if (op == Op::ServoJ)
    require(arg0 >= 1 && arg0 <= 1000, "adapter ServoJ period must be 1..1000 ms");
  if (op == Op::AxisEnable || op == Op::AxisDisable || op == Op::AxisZeroing)
    require(arg0 >= 0 && arg0 <= 7, "joint ID must be 0..7");
  if (op == Op::SetToolCorrd) require(arg0 >= 1 && arg0 <= 19, "tool ID must be 1..19");
  if (op == Op::ServoCart)
    require(values.size() > 0 && values.size() % 7 == 0 && values.size() <= 7 * 128,
            "ServoCart supports 1..128 seven-value points");
  std::lock_guard<std::mutex> lock(mutex_);
  if (op == Op::ServoJ || op == Op::MoveJ)
    for (int j = 0; j < 7; ++j)
      require(
          values.at(j) >= active_lower_[arm * 7 + j] && values.at(j) <= active_upper_[arm * 7 + j],
          "joint degrees exceed active confirmed limits");
  if (op == Op::SetLimitPositive || op == Op::SetLimitNegative)
    for (int j = 0; j < 7; ++j)
      require(values.at(j) >= config_.lower_deg[arm * 7 + j] &&
                  values.at(j) <= config_.upper_deg[arm * 7 + j],
              "runtime soft limits cannot expand confirmed bounds");
  if (op == Op::SetLimitPositive)
    for (int j = 0; j < 7; ++j)
      require(values.at(j) > active_lower_[arm * 7 + j],
              "positive soft limit must exceed active negative limit");
  if (op == Op::SetLimitNegative)
    for (int j = 0; j < 7; ++j)
      require(values.at(j) < active_upper_[arm * 7 + j],
              "negative soft limit must remain below active positive limit");
  Frame f;
  f.arm = arm;
  f.op = op;
  f.arg0 = arg0;
  f.speed = speed;
  f.acceleration = acceleration;
  f.angle = angle;
  f.count = values.size();
  std::copy(values.begin(), values.end(), f.values.begin());
  auto r = exchange_locked(f);
  if (r.code == 0 && op == Op::SetLimitPositive)
    std::copy(values.begin(), values.end(), active_upper_.begin() + arm * 7);
  if (r.code == 0 && op == Op::SetLimitNegative)
    std::copy(values.begin(), values.end(), active_lower_.begin() + arm * 7);
  return {r.code, std::string(r.error.data(), strnlen(r.error.data(), r.error.size())),
          std::vector<double>(r.values.begin(), r.values.begin() + r.count), r.angle, r.state};
}
std::array<SdkReply, 2> Session::servo_joints(const Q14& q, int period) {
  validate_target(q);
  auto degrees = sdk_degrees(q);
  std::array<SdkReply, 2> out;
  for (auto& r : out) {
    r.code = -20;
    r.error = "not sent because an earlier arm command failed";
  }
  for (int a = 0; a < 2; ++a) {
    out[a] = call(a, Op::ServoJ, {degrees.begin() + a * 7, degrees.begin() + a * 7 + 7}, period);
    if (!out[a].ok()) break;
  }
  return out;
}
Snapshot Session::read_state() {
  Snapshot s;
  s.q_rad.fill(std::nan(""));
  s.valid = true;
  for (int a = 0; a < 2; ++a) {
    s.host_read_begin_ns[a] = now_ns();
    s.arms[a] = call(a, Op::GetActualJointPosDegree);
    s.host_read_end_ns[a] = now_ns();
    if (!s.arms[a].ok() || s.arms[a].values.size() != 7) {
      s.valid = false;
      continue;
    }
    for (int j = 0; j < 7; ++j) {
      int k = 7 * a + j;
      double value = (s.arms[a].values[j] - config_.offsets_deg[k]) * pi / 180 * config_.signs[k];
      s.q_rad[7 * a + config_.permutation[k]] = value;
      if (!std::isfinite(value)) s.valid = false;
    }
  }
  return s;
}
Controller::Controller(std::shared_ptr<Session> s, Q14 initial, int period, int timeout)
    : session_(std::move(s)),
      period_ms_(period),
      watchdog_ms_(timeout),
      initial_(initial),
      target_(initial) {
  require(period >= 1 && period <= 1000, "period must be 1..1000 ms");
  require(timeout >= period && timeout <= 60000, "command timeout must be period..60000 ms");
  session_->validate_target(initial);
}
Controller::~Controller() {
  try {
    stop();
  } catch (...) {
  }
}
ControllerStatus Controller::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}
void Controller::start() {
  std::lock_guard<std::mutex> life(lifecycle_);
  if (thread_.joinable()) throw std::runtime_error("stop controller before restarting");
  if (!session_->connected()) throw std::runtime_error("explicitly connect SDK session first");
  auto observed = session_->read_state();
  if (!observed.valid) throw std::runtime_error("startup q read failed");
  for (int j = 0; j < 14; ++j)
    require(std::abs(observed.q_rad[j] - initial_[j]) <= 0.01,
            "startup anchor differs from SDK q by more than the adapter 0.01 rad tolerance; source "
            "freshness remains unknown");
  session_->validate_target(observed.q_rad);
  initial_ = observed.q_rad;
  for (int a = 0; a < 2; ++a) {
    auto r = session_->call(a, Op::ServoMoveStart);
    if (!r.ok()) {
      for (int b = 0; b <= a; ++b) {
        session_->call(b, Op::StopMotion);
        session_->call(b, Op::ServoMoveEnd);
      }
      throw std::runtime_error("ServoMoveStart failed: " + r.error);
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = ControllerStatus{};
    status_.running = true;
    status_.commanded_q_rad = initial_;
    target_ = initial_;
    last_submit_ = std::chrono::steady_clock::now();
  }
  stop_requested_ = false;
  thread_ = std::thread(&Controller::loop, this);
}
std::uint64_t Controller::submit_joints(const Q14& q) {
  session_->validate_target(q);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!status_.running) throw std::runtime_error("controller is not running");
  target_ = q;
  last_submit_ = std::chrono::steady_clock::now();
  return ++status_.target_sequence;
}
fairino_ik::Result Controller::submit_cartesian(fairino_ik::Model& m, const Pose& left,
                                                const Pose& right, const std::vector<double>& seed,
                                                const fairino_ik::Options& options) {
  require(m.nq() == 14, "hardware Cartesian control requires the fourteen-variable arm-only model");
  const auto& config = session_->config();
  require(
      config.fixed_torso_rad.has_value() && config.fixed_head_rad.has_value() &&
          !config.cartesian_frame_evidence.empty(),
      "explicit fixed torso/head and frame calibration required for hardware Cartesian targets");
  for (int j = 0; j < 5; ++j)
    require(std::abs(options.torso_position[j] - (*config.fixed_torso_rad)[j]) < 1e-12,
            "IK fixed torso differs from confirmed hardware configuration");
  require(std::abs(options.head_position - *config.fixed_head_rad) < 1e-12,
          "IK fixed head differs from confirmed hardware configuration");
  auto r = m.solveIK(left, right, seed, options);
  if (r.success) {
    Q14 q;
    std::copy(r.q.begin(), r.q.end(), q.begin());
    submit_joints(q);
  }
  return r;
}
void Controller::loop() noexcept {
  auto previous = std::chrono::steady_clock::now();
  try {
    while (!stop_requested_) {
      auto now = std::chrono::steady_clock::now();
      Q14 goal, current;
      std::uint64_t seq;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (now - last_submit_ > std::chrono::milliseconds(watchdog_ms_))
          throw std::runtime_error("native command watchdog expired");
        goal = target_;
        current = status_.commanded_q_rad;
        seq = status_.target_sequence;
      }
      double dt = std::clamp(std::chrono::duration<double>(now - previous).count(), 0.0,
                             2 * period_ms_ / 1000.0);
      previous = now;
      for (int j = 0; j < 14; ++j) {
        double bound = session_->config().max_velocity_rad_s[j] * dt;
        current[j] += std::clamp(goal[j] - current[j], -bound, bound);
      }
      auto sent = session_->servo_joints(current, period_ms_);
      for (const auto& r : sent)
        if (!r.ok()) throw std::runtime_error("SDK ServoJ failed: " + r.error);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.commanded_q_rad = current;
        status_.sent_sequence = seq;
        status_.host_send_end_ns = now_ns();
      }
      std::this_thread::sleep_until(now + std::chrono::milliseconds(period_ms_));
    }
  } catch (const std::exception& e) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.error = e.what();
  }
  for (int a = 0; a < 2; ++a) try {
      auto stop = session_->call(a, Op::StopMotion);
      auto end = session_->call(a, Op::ServoMoveEnd);
      if (!stop.ok() || !end.ok()) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.error += "; SDK stop/end failed";
      }
    } catch (const std::exception& e) {
      std::lock_guard<std::mutex> lock(mutex_);
      status_.error += "; stop unconfirmed: " + std::string(e.what());
    }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.running = false;
  }
  stop_requested_ = true;
}
void Controller::stop() {
  std::lock_guard<std::mutex> life(lifecycle_);
  stop_requested_ = true;
  if (thread_.joinable()) thread_.join();
}
}  // namespace lpf
