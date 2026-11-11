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
#include <climits>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <sstream>
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

namespace {
struct TransportError : std::runtime_error {
  bool submitted;
  TransportError(std::string why, bool sent)
      : std::runtime_error(std::move(why)), submitted(sent) {}
};
StopReport empty_stops(const std::string& why) {
  StopReport r;
  for (int i = 0; i < 4; ++i)
    r.operations[i] = {i / 2, i % 2 ? Op::ServoMoveEnd : Op::StopMotion,
                       DispatchState::NotDispatched, std::nullopt, why};
  return r;
}
SdkReply sdk_reply(const Frame& f) {
  return {f.code,
          std::string(f.error.data(), strnlen(f.error.data(), f.error.size())),
          {f.values.begin(), f.values.begin() + f.count},
          f.angle,
          f.state};
}
Deadline rpc_deadline(int milliseconds) {
  return Clock::now() + std::chrono::milliseconds(milliseconds);
}
}  // namespace
bool StopReport::ok() const {
  return faults.empty() && std::all_of(operations.begin(), operations.end(),
                                       [](const CallResult& r) { return r.ok(); });
}
bool CleanupReport::ok() const {
  if (!had_worker) return stop.faults.empty() && reason.empty();
  return stop.ok() && sdk_destructors_completed && process.reaped && !process.forced &&
         process.exit_code == 0 && reason.empty();
}
Session::Session(HardwareConfig config, std::string worker, std::string backend)
    : config_(std::move(config)), worker_(std::move(worker)), backend_(std::move(backend)) {
  config_.validate();
  active_lower_ = config_.lower_deg;
  active_upper_ = config_.upper_deg;
  if (worker_.empty()) worker_ = package_directory() + "/_sdk_worker";
  if (backend_.empty()) backend_ = package_directory() + "/_sdk_backend.so";
  last_cleanup_.stop = empty_stops("no worker has been started");
}
Session::~Session() {
  try {
    close();
  } catch (...) {
  }
  if (fd_ >= 0) ::close(fd_);
  if (pidfd_ >= 0) ::close(pidfd_);
}
bool Session::connected() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return fd_ >= 0;
}
std::string Session::diagnostic() const {
  std::lock_guard<std::mutex> lock(fault_mutex_);
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
std::vector<FaultRecord> Session::faults() const {
  std::lock_guard<std::mutex> lock(fault_mutex_);
  return faults_;
}
CleanupReport Session::last_cleanup() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_cleanup_;
}
PairResult Session::last_pair() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_pair_;
}
Arm Session::arm(int side) {
  require(side == 0 || side == 1, "side must be 0 or 1");
  return {shared_from_this(), side};
}
void Session::harvest(const Frame& f) {
  std::lock_guard<std::mutex> lock(fault_mutex_);
  for (const auto& w : f.faults)
    if (w.id) {
      if (w.arm < 0 || w.arm > 1) throw std::runtime_error("invalid fault arm");
      if (std::none_of(faults_.begin(), faults_.end(),
                       [&](const FaultRecord& x) { return x.id == w.id; })) {
        FaultRecord record{
            w.id,         w.arm,
            w.operation,  w.origin,
            std::nullopt, "worker watchdog/stop fault latched; physical stop is unconfirmed"};
        if (w.has_sdk_reply) record.sdk_code = w.code;
        faults_.push_back(record);
        diagnostic_ = record.reason;
      }
    }
}
void Session::transport_failure(const std::string& why, int arm, Op op) {
  std::lock_guard<std::mutex> lock(fault_mutex_);
  faults_.push_back({0, arm < 0 ? std::nullopt : std::optional<int>(arm), op,
                     FaultOrigin::Transport, std::nullopt, why});
  diagnostic_ = why;
}
void Session::wait_locked(short events, Deadline deadline) {
  for (;;) {
    auto now = Clock::now();
    if (now >= deadline)
      throw std::runtime_error("SDK worker deadline expired; physical stop is unconfirmed");
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now).count();
    int ms = static_cast<int>(std::min<long long>(INT_MAX, (ns + 999999) / 1000000));
    pollfd p{fd_, events, 0};
    int rc = poll(&p, 1, ms);
    if (rc < 0 && errno == EINTR) continue;
    if (rc == 0)
      throw std::runtime_error("SDK worker deadline expired; physical stop is unconfirmed");
    if (rc < 0 || (p.revents & (POLLERR | POLLNVAL)))
      throw std::runtime_error("SDK worker IPC failed");
    if (p.revents & (events | POLLHUP)) return;
  }
}
void Session::send_locked(Frame& request, Deadline deadline, bool& submitted) {
  request.sequence = ++sequence_;
  submitted = false;
  if (fd_ < 0) throw TransportError("SDK session is not connected", false);
  try {
    for (;;) {
      wait_locked(POLLOUT, deadline);
      auto n = send(fd_, &request, sizeof(request), MSG_NOSIGNAL | MSG_DONTWAIT);
      if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
      if (n >= 0) submitted = true;
      if (n != sizeof(request)) throw std::runtime_error("SDK worker request write failed");
      submitted = true;
      return;
    }
  } catch (const std::exception& e) {
    throw TransportError(e.what(), submitted);
  }
}
Frame Session::receive_locked(const Frame& request, Deadline deadline) {
  Frame response;
  for (;;) {
    wait_locked(POLLIN, deadline);
    auto n = recv(fd_, &response, sizeof(response), MSG_DONTWAIT | MSG_TRUNC);
    if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
    if (n == 0)
      throw std::runtime_error(
          "SDK worker exited before a valid reply (exit code is not a health handshake)");
    if (n != sizeof(response) || !valid_frame(response) || response.sequence != request.sequence ||
        response.op != request.op || response.arm != request.arm)
      throw std::runtime_error("invalid SDK worker reply identity/size");
    if (request.op == Op::Initialize && response.tag == Tag::Ready &&
        !valid_ready_payload(response)) {
      std::ostringstream reason;
      reason << "invalid initialization Ready: code " << response.code << ", sdk_reply "
             << static_cast<int>(response.has_sdk_reply) << ", count " << response.count
             << ", error_payload " << ready_error_payload(response) << ", fault_payload "
             << ready_fault_payload(response);
      // Do not promote contradictory Ready fault data into the trusted worker ledger.
      throw std::runtime_error(reason.str());
    }
    harvest(response);
    return response;
  }
}
Frame Session::exchange_locked(Frame request, Deadline deadline) {
  bool submitted = false;
  try {
    send_locked(request, deadline, submitted);
    auto response = receive_locked(request, deadline);
    if (response.tag != Tag::Reply && !(request.op == Op::Initialize && response.tag == Tag::Ready))
      throw std::runtime_error("unexpected SDK reply phase");
    if (request.op != Op::Initialize && request.op != Op::PollStatus && !response.has_sdk_reply &&
        response.error[0] == 0)
      throw std::runtime_error("missing original SDK reply or explicit native rejection");
    return response;
  } catch (const TransportError&) {
    throw;
  } catch (const std::exception& e) {
    throw TransportError(e.what(), submitted);
  }
}
ReapReport Session::reap_locked(Deadline deadline, bool force) {
  ReapReport report;
  if (fd_ >= 0) {
    if (::close(fd_) != 0) report.error = "IPC close failed: " + std::string(strerror(errno));
    fd_ = -1;
  }
  if (pid_ <= 0) {
    report.reaped = true;
    return report;
  }
  auto collect = [&]() {
    int status = 0;
    pid_t result = waitpid(pid_, &status, WNOHANG);
    if (result == pid_) {
      report.reaped = true;
      if (WIFEXITED(status)) report.exit_code = WEXITSTATUS(status);
      if (WIFSIGNALED(status)) report.term_signal = WTERMSIG(status);
      pid_ = -1;
      return true;
    }
    if (result < 0 && errno != EINTR) {
      report.error += "; waitpid failed: " + std::string(strerror(errno));
      return true;
    }
    return false;
  };
  auto wait_exit = [&](Deadline until) {
    while (!collect()) {
      if (Clock::now() >= until) return false;
      pollfd p{pidfd_, POLLIN, 0};
      int ms = static_cast<int>(std::min<long long>(
          10, std::max<long long>(
                  1, std::chrono::duration_cast<std::chrono::milliseconds>(until - Clock::now())
                         .count())));
      int rc = poll(&p, 1, ms);
      if (rc < 0 && errno != EINTR) {
        report.error += "; pidfd poll failed: " + std::string(strerror(errno));
        return false;
      }
    }
    return report.reaped;
  };
  if (!force) {
    if (!wait_exit(deadline) && !report.reaped && Clock::now() >= deadline)
      report.error += "; normal worker exit deadline expired";
  } else {
    collect();
  }
  if (!report.reaped && pid_ > 0) {
    if (pidfd_ < 0)
      report.error += "; no pidfd: worker termination unconfirmed";
    else if (syscall(SYS_pidfd_send_signal, pidfd_, SIGKILL, nullptr, 0) == 0) {
      report.forced = true;
      wait_exit(rpc_deadline(100));
    } else if (errno == ESRCH)
      collect();
    else
      report.error += "; pidfd signal failed: " + std::string(strerror(errno));
  }
  if (report.reaped && pidfd_ >= 0) {
    if (::close(pidfd_) != 0) report.error += "; pidfd close failed";
    pidfd_ = -1;
  }
  if (!report.reaped) report.error += "; worker reap unconfirmed";
  return report;
}
void Session::connect(bool deny_network) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0 || pid_ > 0)
    throw std::runtime_error("close/reap the previous worker before connecting");
  {
    std::lock_guard<std::mutex> faults(fault_mutex_);
    faults_.clear();
    diagnostic_.clear();
  }
  int sv[2];
  if (socketpair(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0, sv))
    throw std::runtime_error("socketpair failed");
  char path[] = "/tmp/libpyfairino-session-XXXXXX";
  if (!mkdtemp(path)) {
    ::close(sv[0]);
    ::close(sv[1]);
    throw std::runtime_error("cannot create SDK log directory");
  }
  log_directory_ = path;
  int child_fd = std::max(sv[0], sv[1]) + 1;
  std::vector<std::string> args{worker_, std::to_string(child_fd), backend_,
                                deny_network ? "deny-network" : "allow-network"};
  std::vector<char*> argv;
  for (auto& a : args) argv.push_back(a.data());
  argv.push_back(nullptr);
  posix_spawn_file_actions_t actions;
  int rc = posix_spawn_file_actions_init(&actions);
  const bool actions_initialized = rc == 0;
  if (!rc) rc = posix_spawn_file_actions_adddup2(&actions, sv[1], child_fd);
  if (!rc) rc = posix_spawn_file_actions_addclose(&actions, sv[0]);
  if (!rc) rc = posix_spawn_file_actions_addclose(&actions, sv[1]);
  if (!rc) rc = posix_spawn_file_actions_addchdir_np(&actions, log_directory_.c_str());
  pid_t child = -1;
  if (!rc) rc = posix_spawn(&child, worker_.c_str(), &actions, nullptr, argv.data(), environ);
  if (actions_initialized) posix_spawn_file_actions_destroy(&actions);
  ::close(sv[1]);
  if (rc) {
    ::close(sv[0]);
    throw std::runtime_error("cannot spawn SDK worker: " + std::string(strerror(rc)));
  }
  pid_ = child;
  last_worker_pid_ = child;
  fd_ = sv[0];
  pidfd_ = syscall(SYS_pidfd_open, pid_, 0);
  const auto deadline = rpc_deadline(config_.rpc_timeout_ms);
  try {
    if (pidfd_ < 0)
      throw std::runtime_error("Linux pidfd support is required before SDK initialization");
    char gate = 1;
    if (send(fd_, &gate, 1, MSG_NOSIGNAL | MSG_DONTWAIT) != 1)
      throw std::runtime_error("SDK worker launch gate failed");
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
    auto ready = exchange_locked(init, deadline);
    if (ready.tag != Tag::Ready)
      throw std::runtime_error("SDK arm initialization failed with code " +
                               std::to_string(ready.code));
    active_lower_ = config_.lower_deg;
    active_upper_ = config_.upper_deg;
  } catch (const std::exception& e) {
    transport_failure(e.what(), -1, Op::Initialize);
    last_cleanup_ = {};
    last_cleanup_.had_worker = true;
    last_cleanup_.stop = empty_stops("initialization failed; no stop reply");
    last_cleanup_.process = reap_locked(deadline, true);
    last_cleanup_.stop.faults = faults();
    last_cleanup_.reason = e.what();
    throw;
  }
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

Frame Session::prepare_locked(int arm, Op op, const std::vector<double>& values, int arg0,
                              double speed, double acceleration, double angle) const {
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
  return f;
}
CallResult Session::dispatch_locked(Frame frame, Deadline deadline) {
  CallResult result{frame.arm, frame.op, DispatchState::NotDispatched, std::nullopt, ""};
  if (fd_ < 0) {
    result.reason = "SDK session is not connected";
    return result;
  }
  try {
    auto reply = exchange_locked(frame, std::min(deadline, rpc_deadline(config_.rpc_timeout_ms)));
    if (reply.has_sdk_reply) {
      result.state = DispatchState::SdkReplied;
      result.reply = sdk_reply(reply);
    } else {
      result.state = DispatchState::WorkerRejected;
      result.reason =
          std::string(reply.error.data(), strnlen(reply.error.data(), reply.error.size()));
    }
  } catch (const TransportError& e) {
    result.state = e.submitted ? DispatchState::Unknown : DispatchState::NotDispatched;
    result.reason = e.what();
    transport_failure(result.reason, frame.arm, frame.op);
    last_cleanup_ = {};
    last_cleanup_.had_worker = true;
    last_cleanup_.stop = empty_stops("worker lost before stop request");
    last_cleanup_.process = reap_locked(deadline, true);
    last_cleanup_.stop.faults = faults();
    last_cleanup_.reason = result.reason;
  }
  return result;
}
CallResult Session::call(int arm, Op op, const std::vector<double>& values, int arg0, double speed,
                         double acceleration, double angle) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto frame = prepare_locked(arm, op, values, arg0, speed, acceleration, angle);
  auto result = dispatch_locked(frame, Deadline::max());
  if (result.ok() && op == Op::SetLimitPositive)
    std::copy(values.begin(), values.end(), active_upper_.begin() + arm * 7);
  if (result.ok() && op == Op::SetLimitNegative)
    std::copy(values.begin(), values.end(), active_lower_.begin() + arm * 7);
  return result;
}
PairResult Session::servo_joints(const Q14& q, int period) {
  return dispatch_pair(q, period, 0, Deadline::max(), nullptr);
}
PairResult Session::dispatch_pair(const Q14& q, int period, std::uint64_t target_sequence,
                                  Deadline deadline, const std::atomic<bool>* stop) {
  std::lock_guard<std::mutex> lock(mutex_);
  // All locally knowable failures are checked while holding the same lock as limit updates.
  validate_target(q);
  auto degrees = sdk_degrees(q);
  std::array<Frame, 2> frames;
  for (int a = 0; a < 2; ++a)
    frames[a] = prepare_locked(
        a, Op::ServoJ, {degrees.begin() + a * 7, degrees.begin() + a * 7 + 7}, period, 0, 0, 0);
  PairResult pair;
  pair.target_sequence = target_sequence;
  for (int a = 0; a < 2; ++a)
    pair.arms[a] = {a, Op::ServoJ, DispatchState::NotDispatched, std::nullopt, "not dispatched"};
  for (int a = 0; a < 2; ++a) {
    // Linearization point: checks immediately precede this arm's IPC dispatch. In-flight SDK work
    // is not retractable.
    if (stop && stop->load(std::memory_order_acquire)) {
      pair.reason = "stop requested before arm dispatch";
      break;
    }
    if (Clock::now() >= deadline) {
      pair.reason = "captured target deadline expired before arm dispatch";
      break;
    }
    pair.arms[a] = dispatch_locked(frames[a], deadline);
    if (!pair.arms[a].ok()) {
      pair.reason =
          pair.arms[a].reason.empty() ? "SDK returned nonzero for an arm" : pair.arms[a].reason;
      break;
    }
  }
  for (auto& a : pair.arms)
    if (a.state == DispatchState::NotDispatched && !pair.reason.empty()) a.reason = pair.reason;
  last_pair_ = pair;
  return pair;
}
std::vector<FaultRecord> Session::poll_status() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ >= 0) {
    Frame f;
    f.op = Op::PollStatus;
    try {
      exchange_locked(f, rpc_deadline(config_.rpc_timeout_ms));
    } catch (const std::exception& e) {
      transport_failure(e.what(), -1, Op::PollStatus);
      last_cleanup_ = {};
      last_cleanup_.had_worker = true;
      last_cleanup_.stop = empty_stops("status transport failed before stop");
      last_cleanup_.process = reap_locked(rpc_deadline(100), true);
      last_cleanup_.stop.faults = faults();
      last_cleanup_.reason = e.what();
    }
  }
  return faults();
}
StopReport Session::stop_locked(bool shutdown, Deadline deadline, bool& destructed) {
  StopReport stop = empty_stops("stop request not dispatched");
  destructed = false;
  if (fd_ < 0) {
    stop.reason = "worker unavailable; physical stop unconfirmed";
    stop.faults = faults();
    return stop;
  }
  Frame f;
  f.op = shutdown ? Op::Shutdown : Op::StopAll;
  bool submitted = false;
  try {
    send_locked(f, deadline, submitted);
    for (auto& op : stop.operations) {
      op.state = DispatchState::Unknown;
      op.reason = "stop request submitted; SDK reply not yet known";
    }
    int next = 0;
    bool stopped = false;
    for (;;) {
      auto response = receive_locked(f, deadline);
      if (response.tag == Tag::StopProgress) {
        if (response.arg0 != next || next >= 4 || !response.has_sdk_reply)
          throw std::runtime_error("invalid stop progress");
        auto& op = stop.operations[next++];
        op.state = DispatchState::SdkReplied;
        op.reply = sdk_reply(response);
        op.reason.clear();
      } else if (response.tag == Tag::Stopped) {
        if (next != 4 || stopped) throw std::runtime_error("incomplete stop phase");
        stopped = true;
        if (!shutdown) break;
      } else if (response.tag == Tag::Destructed && shutdown && stopped) {
        destructed = true;
        break;
      } else
        throw std::runtime_error("invalid shutdown phase");
    }
  } catch (const std::exception& e) {
    if (submitted)
      for (auto& op : stop.operations)
        if (op.state == DispatchState::NotDispatched) {
          op.state = DispatchState::Unknown;
          op.reason = "stop request submitted; operation result unknown";
        }
    stop.reason = e.what();
    transport_failure(stop.reason, -1, f.op);
  }
  stop.faults = faults();
  return stop;
}
StopReport Session::stop_all() {
  std::lock_guard<std::mutex> lock(mutex_);
  bool unused = false;
  auto deadline = rpc_deadline(config_.rpc_timeout_ms);
  auto stop = stop_locked(false, deadline, unused);
  if (!stop.reason.empty() && fd_ >= 0) {
    last_cleanup_ = {};
    last_cleanup_.had_worker = true;
    last_cleanup_.stop = stop;
    last_cleanup_.process = reap_locked(deadline, true);
    last_cleanup_.reason = stop.reason;
  }
  return stop;
}
CleanupReport Session::close() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fd_ < 0) {
    if (pid_ > 0) {
      auto previous = last_cleanup_.process;
      last_cleanup_.process = reap_locked(rpc_deadline(config_.rpc_timeout_ms), true);
      last_cleanup_.process.forced = last_cleanup_.process.forced || previous.forced;
      if (!previous.error.empty())
        last_cleanup_.process.error = previous.error + "; " + last_cleanup_.process.error;
    }
    return last_cleanup_;
  }
  CleanupReport cleanup;
  cleanup.had_worker = true;
  const auto deadline = rpc_deadline(config_.rpc_timeout_ms);
  cleanup.stop = stop_locked(true, deadline, cleanup.sdk_destructors_completed);
  cleanup.process = reap_locked(deadline, !cleanup.sdk_destructors_completed);
  cleanup.reason = cleanup.stop.reason;
  if (!cleanup.sdk_destructors_completed && cleanup.reason.empty())
    cleanup.reason = "SDK destructors not acknowledged";
  if (!cleanup.process.error.empty()) cleanup.reason += "; " + cleanup.process.error;
  last_cleanup_ = cleanup;
  return cleanup;
}
Snapshot Session::read_state() {
  Snapshot s;
  s.q_rad.fill(std::nan(""));
  s.valid = true;
  for (int a = 0; a < 2; ++a) {
    s.host_read_begin_ns[a] = now_ns();
    s.arms[a] = call(a, Op::GetActualJointPosDegree);
    s.host_read_end_ns[a] = now_ns();
    if (!s.arms[a].ok() || s.arms[a].reply->values.size() != 7) {
      s.valid = false;
      continue;
    }
    for (int j = 0; j < 7; ++j) {
      int k = 7 * a + j;
      double value =
          (s.arms[a].reply->values[j] - config_.offsets_deg[k]) * pi / 180 * config_.signs[k];
      s.q_rad[7 * a + config_.permutation[k]] = value;
      if (!std::isfinite(value)) s.valid = false;
    }
  }
  s.faults = faults();
  return s;
}
Controller::Controller(std::shared_ptr<Session> s, Q14 initial, int period, int timeout)
    : session_(std::move(s)), period_ms_(period), watchdog_ms_(timeout), initial_(initial) {
  require(period >= 1 && period <= 1000, "period must be 1..1000 ms");
  require(timeout >= period && timeout <= 60000, "command timeout must be period..60000 ms");
  session_->validate_target(initial);
  status_.stop_result = empty_stops("controller has not started");
}
Controller::~Controller() {
  try {
    stop();
  } catch (...) {
  }
}
ControllerStatus Controller::status() const {
  ControllerStatus result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result = status_;
  }
  result.cartesian_in_progress = cartesian_in_progress_.load(std::memory_order_acquire);
  result.stop_requested = stop_requested_.load(std::memory_order_acquire);
  result.faults = session_->faults();
  return result;
}
void Controller::start() {
  std::lock_guard<std::mutex> life(lifecycle_);
  if (thread_.joinable()) throw std::runtime_error("stop controller before restarting");
  if (!session_->connected()) throw std::runtime_error("explicitly connect SDK session first");
  if (!session_->poll_status().empty())
    throw std::runtime_error("latched session fault requires close and reconstruction");
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
      auto cleaned = session_->stop_all();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.stop_result = cleaned;
      }
      throw std::runtime_error("ServoMoveStart did not return SDK success");
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = ControllerStatus{};
    status_.running = true;
    status_.commanded_q_rad = initial_;
    const auto now = Clock::now();
    target_ = {initial_, 0, now, now + std::chrono::milliseconds(watchdog_ms_)};
  }
  stop_requested_ = false;
  thread_ = std::thread(&Controller::loop, this);
}
std::uint64_t Controller::submit_joints(const Q14& q) {
  session_->validate_target(q);
  if (!session_->faults().empty())
    throw std::runtime_error("latched session fault requires reconstruction");
  std::lock_guard<std::mutex> lock(mutex_);
  if (!status_.running || stop_requested_) throw std::runtime_error("controller is not running");
  const auto now = Clock::now();
  const auto seq = ++status_.target_sequence;
  target_ = {q, seq, now, now + std::chrono::milliseconds(watchdog_ms_)};
  return seq;
}
fairino_ik::Result Controller::submit_cartesian(fairino_ik::Model& m, const Pose& left,
                                                const Pose& right, const std::vector<double>& seed,
                                                fairino_ik::Options options) {
  bool expected = false;
  if (!cartesian_in_progress_.compare_exchange_strong(expected, true))
    throw std::runtime_error("serialize Cartesian submissions on each Controller");
  struct Reset {
    std::atomic<bool>& flag;
    ~Reset() { flag.store(false, std::memory_order_release); }
  } reset{cartesian_in_progress_};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.last_cartesian_torso = options.torso_position;
    status_.last_cartesian_head = options.head_position;
  }
  require(m.nq() == 14, "hardware Cartesian control requires the fourteen-variable arm-only model");
  const auto& config = session_->config();
  require(
      config.fixed_torso_rad && config.fixed_head_rad && !config.cartesian_frame_evidence.empty(),
      "explicit fixed torso/head and frame calibration required for hardware Cartesian targets");
  const auto limits = m.torsoLimits();
  const auto head = m.headLimits();
  require(limits.size() == 5, "Model must provide five fixed torso limits");
  auto in_range = [](double x, const std::array<double, 2>& bounds) {
    return std::isfinite(x) && std::isfinite(bounds[0]) && std::isfinite(bounds[1]) &&
           bounds[0] <= bounds[1] && x >= bounds[0] && x <= bounds[1];
  };
  for (int i = 0; i < 5; ++i) {
    require(in_range((*config.fixed_torso_rad)[i], limits[i]) &&
                in_range(options.torso_position[i], limits[i]),
            "fixed torso is outside the actual Model closed interval");
    require(options.torso_position[i] == (*config.fixed_torso_rad)[i],
            "IK fixed torso differs from confirmed hardware configuration");
  }
  require(in_range(*config.fixed_head_rad, head) && in_range(options.head_position, head),
          "fixed head is outside the actual Model closed interval");
  require(options.head_position == *config.fixed_head_rad,
          "IK fixed head differs from confirmed hardware configuration");
  auto verify_fixed = [&](const std::vector<double>& q) {
    auto full = m.fullConfiguration(q, options);
    auto coordinates = m.jointCoordinates();
    const std::array<std::string, 5> names{{"j20", "j19", "j18", "j16", "j15"}};
    for (int i = 0; i < 5; ++i)
      require(full.at(coordinates.at(names[i])) == options.torso_position[i],
              "Model changed the fixed torso configuration; unsupported coupling");
    require(full.at(coordinates.at("j17")) == options.head_position,
            "Model changed the fixed head configuration; unsupported coupling");
  };
  verify_fixed(seed);
  auto result = m.solveIK(left, right, seed, options);
  if (result.success) {
    verify_fixed(result.q);
    Q14 q;
    std::copy(result.q.begin(), result.q.end(), q.begin());
    submit_joints(q);
  }
  return result;
}
void Controller::loop() noexcept {
  auto previous = Clock::now();
  try {
    while (!stop_requested_.load(std::memory_order_acquire)) {
      const auto now = Clock::now();
      Target captured;
      Q14 current;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        captured = target_;
        current = status_.commanded_q_rad;
      }
      if (now >= captured.deadline) throw std::runtime_error("captured target deadline expired");
      double dt = std::clamp(std::chrono::duration<double>(now - previous).count(), 0.0,
                             2 * period_ms_ / 1000.0);
      previous = now;
      for (int j = 0; j < 14; ++j) {
        double bound = session_->config().max_velocity_rad_s[j] * dt;
        current[j] += std::clamp(captured.q[j] - current[j], -bound, bound);
      }
      // Policy: a captured old pair may finish inside its original deadline; newer submissions
      // never renew it.
      auto sent = session_->dispatch_pair(current, period_ms_, captured.sequence, captured.deadline,
                                          &stop_requested_);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.last_pair = sent;
        if (sent.complete()) {
          status_.commanded_q_rad = current;
          status_.sent_sequence = captured.sequence;
          status_.host_send_end_ns = now_ns();
        }
      }
      if (!sent.complete())
        throw std::runtime_error(sent.reason.empty() ? "pair did not complete" : sent.reason);
      std::this_thread::sleep_until(now + std::chrono::milliseconds(period_ms_));
    }
  } catch (const std::exception& e) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.error = e.what();
  }
  auto cleanup = session_->stop_all();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.stop_result = cleanup;
    status_.running = false;
    if (!cleanup.ok() && status_.error.empty())
      status_.error = "stop/end failed or remains unconfirmed";
  }
  stop_requested_.store(true, std::memory_order_release);
}
StopReport Controller::stop() {
  // Request is observable before waiting for lifecycle/join or the Session mutex.
  stop_requested_.store(true, std::memory_order_release);
  std::lock_guard<std::mutex> life(lifecycle_);
  stop_requested_.store(true, std::memory_order_release);
  if (thread_.joinable()) thread_.join();
  std::lock_guard<std::mutex> lock(mutex_);
  return status_.stop_result;
}
}  // namespace lpf
