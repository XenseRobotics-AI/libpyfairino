// Explicit test double. Never installed and never evidence for the vendor implementation.
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <map>
#include <thread>

#include "humanorid_sdk.h"
struct State {
  std::array<double, 7> q{}, lo{}, hi{};
};
static std::map<SingleRobot*, State> states;
static bool mode(const char* s) {
  const char* m = std::getenv("LIBPYFAIRINO_TEST_MODE");
  return m && std::strcmp(m, s) == 0;
}
RobotBase::RobotBase() {}
RobotBase::~RobotBase() {}
SingleRobot::SingleRobot(int, RobotName, double*) {
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
}
SingleRobot::~SingleRobot() { states.erase(this); }
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
ARMErrorCode SingleRobot::ServoMoveEnd() { return ARMErrorCode::Success; }
ARMErrorCode SingleRobot::ServoJ(double* q, int period, char* error) {
  if (mode("hang_servo")) std::this_thread::sleep_for(std::chrono::seconds(60));
  if (mode("error_buffer")) {
    std::memset(error, 'X', 1024);
    return ARMErrorCode::InvalidInput_OutOfRange;
  }
  std::copy(q, q + 7, states[this].q.begin());
  for (int j = 0; j < 7; ++j) q[j] *= 3.14159265358979323846 / 180;
  std::this_thread::sleep_for(std::chrono::milliseconds(period));
  return ARMErrorCode::Success;
}
ARMErrorCode SingleRobot::StopMotion() {
  if (mode("hang_stop")) std::this_thread::sleep_for(std::chrono::seconds(60));
  return ARMErrorCode::Success;
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
  std::copy(q, q + 7, states[this].hi.begin());
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
