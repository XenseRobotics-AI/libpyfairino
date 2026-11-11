// Explicit test double. Never installed and never evidence for the vendor implementation.
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <thread>

#include "humanorid_sdk.h"
struct State {
  std::array<double, 7> q{}, lo{}, hi{};
  int arm = 0, servo_count = 0, pos_count = 0, stop_count = 0, end_count = 0;
};
static std::map<SingleRobot*, State> states;
static bool mode(const char* s) {
  const char* m = std::getenv("LIBPYFAIRINO_TEST_MODE");
  return m && std::strcmp(m, s) == 0;
}

static std::string setting(const char* key) {
  const char* s = std::getenv(key);
  return s ? s : "";
}
static void event(const char* name, int arm, double value = 0) {
  auto dir = setting("LPF_TEST_DIR");
  if (dir.empty()) return;
  std::ofstream out(dir + "/events.jsonl", std::ios::app);
  out << "{\"event\":\"" << name << "\",\"arm\":" << arm << ",\"value\":" << value
      << ",\"pid\":" << getpid() << "}" << std::endl;
}
static void barrier(const std::string& key, bool once = true) {
  auto dir = setting("LPF_TEST_DIR");
  if (dir.empty() || setting("LPF_TEST_BLOCK") != key || !once) return;
  while (!std::filesystem::exists(dir + "/release_" + key))
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
static int injected(const char* key, int index) {
  std::istringstream in(setting(key));
  std::string part;
  for (int i = 0; i <= index; ++i)
    if (!std::getline(in, part, ',')) return 0;
  return std::atoi(part.c_str());
}
struct ExitProbe {
  ~ExitProbe() {
    if (setting("LPF_TEST_BLOCK") == "process_exit") {
      event("process_exit_enter", -1);
      barrier("process_exit");
      event("process_exit_complete", -1);
    }
  }
};
static ExitProbe exit_probe;
RobotBase::RobotBase() {}
RobotBase::~RobotBase() {}
SingleRobot::SingleRobot(int, RobotName name, double*) {
  if (mode("probe_filter")) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd >= 0) {
      close(fd);
      std::exit(73);
    }
    if (errno != EPERM) std::exit(74);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    if (connect(-1, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != -1 || errno != EPERM)
      std::exit(75);
  }
  if (mode("exit0")) std::exit(0);
  if (mode("exit1")) std::exit(1);
  if (mode("hang_ctor")) std::this_thread::sleep_for(std::chrono::seconds(60));
  states[this] = {};
  states[this].arm = name == SingleRobot::RightArm ? 1 : 0;
  event("construct", states[this].arm);
}
SingleRobot::~SingleRobot() {
  int a = states[this].arm;
  event("destructor_enter", a);
  if (setting("LPF_TEST_EXIT_AT") == "destructor" + std::to_string(a)) std::_Exit(0);
  barrier("destructor" + std::to_string(a));
  int delay = std::atoi(setting("LPF_TEST_DTOR_SLEEP_MS").c_str());
  if (delay > 0) std::this_thread::sleep_for(std::chrono::milliseconds(delay));
  event("destructor_complete", a);
  states.erase(this);
}
ARMErrorCode SingleRobot::MoveJ(double* q, double, double, char*) {
  std::copy(q, q + 7, states[this].q.begin());
  return ARMErrorCode::Success;
}
ARMErrorCode SingleRobot::MoveP(double*, double, double, double, char*) {
  return ARMErrorCode::Success;
}
ARMErrorCode SingleRobot::MoveL(double*, double, double, double, char*) {
  return ARMErrorCode::Success;
}
ARMErrorCode SingleRobot::MoveC(double*, double, double*, double, double, double, char*) {
  return ARMErrorCode::Success;
}
ARMErrorCode SingleRobot::ServoCart(double*, uint32_t, char*) { return ARMErrorCode::Success; }
ARMErrorCode SingleRobot::ServoMoveStart() { return ARMErrorCode::Success; }
ARMErrorCode SingleRobot::ServoMoveEnd() {
  auto& state = states[this];
  event("end_enter", state.arm);
  barrier("end" + std::to_string(state.arm));
  int code = injected("LPF_TEST_STOP_CODES", 2 * state.arm + 1);
  ++state.end_count;
  if (setting("LPF_TEST_FAIL_ONCE") == "1" && state.end_count > 1) code = 0;
  event("end_return", state.arm, code);
  if (setting("LPF_TEST_DROP_REPLY") == "end" + std::to_string(state.arm))
    syscall(SYS_close_range, 3, ~0U, 0);
  return static_cast<ARMErrorCode>(code);
}
ARMErrorCode SingleRobot::ServoJ(double* q, int period, char* error) {
  auto& state = states[this];
  event("servo_enter", state.arm, q[0]);
  ++state.servo_count;
  barrier("servo" + std::to_string(state.arm), state.servo_count == 1);
  if (setting("LPF_TEST_EXIT_AT") == "servo" + std::to_string(state.arm)) std::exit(0);

  if (mode("hang_servo")) std::this_thread::sleep_for(std::chrono::seconds(60));
  if (mode("error_buffer")) {
    std::memset(error, 'X', 1024);
    return ARMErrorCode::InvalidInput_OutOfRange;
  }
  int injected_code = injected("LPF_TEST_SERVO_CODES", state.arm);
  if (injected_code) {
    event("servo_return", state.arm, injected_code);
    return static_cast<ARMErrorCode>(injected_code);
  }
  std::copy(q, q + 7, states[this].q.begin());
  for (int j = 0; j < 7; ++j) q[j] *= 3.14159265358979323846 / 180;
  std::this_thread::sleep_for(std::chrono::milliseconds(period));
  event("servo_return", state.arm, 0);
  if (setting("LPF_TEST_DROP_REPLY") == "servo" + std::to_string(state.arm))
    syscall(SYS_close_range, 3, ~0U, 0);
  return ARMErrorCode::Success;
}
ARMErrorCode SingleRobot::StopMotion() {
  auto& state = states[this];
  event("stop_enter", state.arm);
  barrier("stop" + std::to_string(state.arm));

  if (mode("hang_stop")) std::this_thread::sleep_for(std::chrono::seconds(60));
  int code = injected("LPF_TEST_STOP_CODES", 2 * state.arm);
  ++state.stop_count;
  if (setting("LPF_TEST_FAIL_ONCE") == "1" && state.stop_count > 1) code = 0;
  event("stop_return", state.arm, code);
  if (setting("LPF_TEST_DROP_REPLY") == "stop" + std::to_string(state.arm))
    syscall(SYS_close_range, 3, ~0U, 0);
  return static_cast<ARMErrorCode>(code);
}
ARMErrorCode SingleRobot::SetToolCorrd(int, double*) { return ARMErrorCode::Success; }
ARMErrorCode SingleRobot::SetAccScale(double) { return ARMErrorCode::Success; }
ARMErrorCode SingleRobot::SetSpeed(double) { return ARMErrorCode::Success; }
ARMErrorCode SingleRobot::GetForwardKin(double* q, double* out, double& angle) {
  std::copy(q, q + 6, out);
  angle = q[6];
  return ARMErrorCode::Success;
}
ARMErrorCode SingleRobot::GetInverseKin(double* d, double a, double* out) {
  std::copy(d, d + 6, out);
  out[6] = a;
  return ARMErrorCode::Success;
}
ARMErrorCode SingleRobot::GetInverseKinRef(double* d, double a, double*, double* out) {
  return GetInverseKin(d, a, out);
}
ARMErrorCode SingleRobot::GetActualJointPosDegree(double* out) {
  if (mode("exit_on_read")) std::exit(0);
  std::copy(states[this].q.begin(), states[this].q.end(), out);
  return ARMErrorCode::Success;
}
ARMErrorCode SingleRobot::GetRobotMotionDone(uint8_t& state) {
  state = 1;
  return ARMErrorCode::Success;
}
ARMErrorCode SingleRobot::GetEEVel(double* out) {
  std::fill(out, out + 6, 0);
  return ARMErrorCode::Success;
}
ARMErrorCode SingleRobot::AxisEnable(int) { return ARMErrorCode::Success; }
ARMErrorCode SingleRobot::AxisDisable(int) { return ARMErrorCode::Success; }
ARMErrorCode SingleRobot::AxisZeroing(int) { return ARMErrorCode::Success; }
ARMErrorCode SingleRobot::AxisResetError() { return ARMErrorCode::Success; }
ARMErrorCode SingleRobot::GetActualToolFlangePose(double* q, double* out) {
  std::copy(q, q + 7, out);
  return ARMErrorCode::Success;
}
ARMErrorCode SingleRobot::SetLimitPositive(double* q) {
  auto& state = states[this];
  ++state.pos_count;
  if (state.pos_count > 1) {
    event("limit_enter", state.arm, q[0]);
    barrier("limit" + std::to_string(state.arm));
  }

  std::copy(q, q + 7, states[this].hi.begin());
  if (state.pos_count > 1) event("limit_return", state.arm, q[0]);
  return ARMErrorCode::Success;
}
ARMErrorCode SingleRobot::SetLimitNegative(double* q) {
  std::copy(q, q + 7, states[this].lo.begin());
  return ARMErrorCode::Success;
}
ARMErrorCode SingleRobot::GetJointSoftLimitDeg(double* lo, double* hi) {
  std::copy(states[this].lo.begin(), states[this].lo.end(), lo);
  std::copy(states[this].hi.begin(), states[this].hi.end(), hi);
  return ARMErrorCode::Success;
}
