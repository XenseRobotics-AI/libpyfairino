#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>

#include "humanorid_sdk.h"
#include "protocol.hpp"
namespace {
using namespace lpf;
void send_frame(int fd, const Frame& f) {
  if (write(fd, &f, sizeof(f)) != sizeof(f))
    throw std::runtime_error("worker response write failed");
}
bool receive(int fd, Frame& f) {
  std::array<char, sizeof(Frame) + 1> bytes;
  auto n = read(fd, bytes.data(), bytes.size());
  if (n == sizeof(Frame)) std::memcpy(&f, bytes.data(), sizeof(Frame));
  if (n == 0) return false;
  if (n != sizeof(f) || !valid_frame(f) || f.tag != Tag::Request)
    throw std::runtime_error("invalid worker request");
  return true;
}
}  // namespace
extern "C" int libpyfairino_backend_main(int fd) {
  using namespace lpf;
  std::array<std::unique_ptr<SingleRobot>, 2> arms;
  std::array<bool, 2> servo{};
  std::array<std::chrono::steady_clock::time_point, 2> last;
  int watchdog = 500;
  auto stop = [&]() {
    int code = 0;
    for (int a = 0; a < 2; ++a)
      if (arms[a]) {
        auto x = arms[a]->StopMotion();
        auto y = arms[a]->ServoMoveEnd();
        if (x != ARMErrorCode::Success) code = static_cast<int>(x);
        if (y != ARMErrorCode::Success) code = static_cast<int>(y);
        servo[a] = false;
      }
    return code;
  };
  try {
    Frame f;
    if (!receive(fd, f) || f.op != Op::Initialize || f.count != 84 || f.arg0 < 0 || f.arg0 > 3 ||
        f.arg1 < 10)
      return 65;
    watchdog = f.arg1;
    for (int a = 0; a < 2; ++a) {
      arms[a] = std::make_unique<SingleRobot>(f.arg0,
                                              a == 0 ? SingleRobot::LeftArm : SingleRobot::RightArm,
                                              f.values.data() + 28 + a * 28);
      auto lo = arms[a]->SetLimitNegative(f.values.data() + a * 7);
      auto hi = arms[a]->SetLimitPositive(f.values.data() + 14 + a * 7);
      if (lo != ARMErrorCode::Success || hi != ARMErrorCode::Success) {
        f.tag = Tag::Reply;
        f.code = static_cast<int>(lo != ARMErrorCode::Success ? lo : hi);
        error_text(f, "failed to apply confirmed SDK joint limits");
        send_frame(fd, f);
        return 66;
      }
    }
    f.tag = Tag::Ready;
    f.count = 0;
    send_frame(fd, f);
    for (;;) {
      pollfd p{fd, POLLIN, 0};
      int pr = poll(&p, 1, 10);
      if (pr < 0) {
        if (errno == EINTR) continue;
        throw std::runtime_error("worker poll failed");
      }
      auto now = std::chrono::steady_clock::now();
      for (int a = 0; a < 2; ++a)
        if (servo[a] && now - last[a] > std::chrono::milliseconds(watchdog)) {
          arms[a]->StopMotion();
          arms[a]->ServoMoveEnd();
          servo[a] = false;
        }
      if (!pr) continue;
      if (!receive(fd, f)) break;
      if (f.op == Op::Initialize) throw std::runtime_error("duplicate initialization");
      if (f.op == Op::Shutdown) {
        f.code = stop();
        f.count = 0;
        f.tag = Tag::Reply;
        send_frame(fd, f);
        return 0;
      }
      int expected = 0;
      switch (f.op) {
        case Op::MoveJ:
        case Op::ServoJ:
        case Op::GetForwardKin:
        case Op::GetActualToolFlangePose:
        case Op::SetLimitPositive:
        case Op::SetLimitNegative:
          expected = 7;
          break;
        case Op::MoveP:
        case Op::MoveL:
        case Op::SetToolCorrd:
        case Op::GetInverseKin:
          expected = 6;
          break;
        case Op::MoveC:
        case Op::GetInverseKinRef:
          expected = 13;
          break;
        case Op::ServoCart:
          expected = f.arg0 * 7;
          if (f.arg0 < 1 || f.arg0 > 128) throw std::runtime_error("invalid ServoCart point count");
          break;
        default:
          break;
      }
      if (f.count != expected) throw std::runtime_error("invalid SDK operation buffer length");
      auto& r = *arms[f.arm];
      auto input = f.values;
      std::fill(f.values.begin(), f.values.end(), std::numeric_limits<double>::quiet_NaN());
      char error[1024]{};
      ARMErrorCode code = ARMErrorCode::DefaultErr;
      f.count = 0;
      switch (f.op) {
        case Op::MoveJ:
          code = r.MoveJ(input.data(), f.speed, f.acceleration, error);
          break;
        case Op::MoveP:
          code = r.MoveP(input.data(), f.angle, f.speed, f.acceleration, error);
          break;
        case Op::MoveL:
          code = r.MoveL(input.data(), f.angle, f.speed, f.acceleration, error);
          break;
        case Op::MoveC:
          code = r.MoveC(input.data(), input[6], input.data() + 7, f.angle, f.speed, f.acceleration,
                         error);
          break;
        case Op::ServoCart:
          code = r.ServoCart(input.data(), f.arg0, error);
          break;
        case Op::ServoMoveStart:
          code = r.ServoMoveStart();
          if (code == ARMErrorCode::Success) {
            servo[f.arm] = true;
            last[f.arm] = now;
          }
          break;
        case Op::ServoMoveEnd:
          code = r.ServoMoveEnd();
          servo[f.arm] = false;
          break;
        case Op::ServoJ:
          if (!servo[f.arm]) {
            error_text(f, "native servo is not active or worker command watchdog expired");
            code = ARMErrorCode::CMD_RunErr;
          } else {
            code = r.ServoJ(input.data(), f.arg0, error);
            last[f.arm] = std::chrono::steady_clock::now();
          }
          break;
        case Op::StopMotion:
          code = r.StopMotion();
          servo[f.arm] = false;
          break;
        case Op::SetToolCorrd:
          code = r.SetToolCorrd(f.arg0, input.data());
          break;
        case Op::SetAccScale:
          code = r.SetAccScale(f.speed);
          break;
        case Op::SetSpeed:
          code = r.SetSpeed(f.speed);
          break;
        case Op::GetForwardKin:
          code = r.GetForwardKin(input.data(), f.values.data(), f.angle);
          f.count = 6;
          break;
        case Op::GetInverseKin:
          code = r.GetInverseKin(input.data(), f.angle, f.values.data());
          f.count = 7;
          break;
        case Op::GetInverseKinRef:
          code = r.GetInverseKinRef(input.data(), f.angle, input.data() + 6, f.values.data());
          f.count = 7;
          break;
        case Op::GetActualJointPosDegree:
          code = r.GetActualJointPosDegree(f.values.data());
          f.count = 7;
          break;
        case Op::GetRobotMotionDone:
          code = r.GetRobotMotionDone(f.state);
          break;
        case Op::GetEEVel:
          code = r.GetEEVel(f.values.data());
          f.count = 6;
          break;
        case Op::AxisEnable:
          code = r.AxisEnable(f.arg0);
          break;
        case Op::AxisDisable:
          code = r.AxisDisable(f.arg0);
          break;
        case Op::AxisZeroing:
          code = r.AxisZeroing(f.arg0);
          break;
        case Op::AxisResetError:
          code = r.AxisResetError();
          break;
        case Op::GetActualToolFlangePose:
          code = r.GetActualToolFlangePose(input.data(), f.values.data());
          f.count = 7;
          break;
        case Op::SetLimitPositive:
          code = r.SetLimitPositive(input.data());
          break;
        case Op::SetLimitNegative:
          code = r.SetLimitNegative(input.data());
          break;
        case Op::GetJointSoftLimitDeg:
          code = r.GetJointSoftLimitDeg(f.values.data(), f.values.data() + 7);
          f.count = 14;
          break;
        default:
          throw std::runtime_error("unknown SDK arm operation");
      }
      f.code = static_cast<int>(code);
      if (error[0]) std::copy(std::begin(error), std::end(error), f.error.begin());
      f.tag = Tag::Reply;
      send_frame(fd, f);
    }
    stop();
    return 0;
  } catch (const std::exception&) {
    try {
      stop();
    } catch (...) {
    }
    return 70;
  }
}
