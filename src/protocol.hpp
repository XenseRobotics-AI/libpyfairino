#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <type_traits>
namespace lpf {
enum class Op : std::uint32_t {
  Initialize,
  Shutdown,
  MoveJ,
  MoveP,
  MoveL,
  MoveC,
  ServoCart,
  ServoMoveStart,
  ServoMoveEnd,
  ServoJ,
  StopMotion,
  SetToolCorrd,
  SetAccScale,
  SetSpeed,
  GetForwardKin,
  GetInverseKin,
  GetInverseKinRef,
  GetActualJointPosDegree,
  GetRobotMotionDone,
  GetEEVel,
  AxisEnable,
  AxisDisable,
  AxisZeroing,
  AxisResetError,
  GetActualToolFlangePose,
  SetLimitPositive,
  SetLimitNegative,
  GetJointSoftLimitDeg,
  StopAll,
  PollStatus
};
enum class Tag : std::uint32_t {
  Request = 1,
  Reply = 2,
  Ready = 3,
  StopProgress = 4,
  Stopped = 5,
  Destructed = 6
};
enum class FaultOrigin : std::uint32_t {
  Watchdog = 1,
  ExplicitStop = 2,
  Cleanup = 3,
  Transport = 4
};
struct WireFault {
  std::uint64_t id = 0;
  std::int32_t arm = 0;
  Op operation = Op::StopMotion;
  FaultOrigin origin = FaultOrigin::Watchdog;
  std::int32_t code = 0;
  std::uint8_t has_sdk_reply = 0;
};
struct Frame {
  std::uint32_t magic = 0x4c504631, version = 2;
  Tag tag = Tag::Request;
  Op op = Op::Initialize;
  std::uint64_t sequence = 0;
  std::int32_t arm = 0, arg0 = 0, arg1 = 0, code = 0;
  std::uint32_t count = 0;
  std::uint8_t state = 0, has_sdk_reply = 0;
  double speed = 0, acceleration = 0, angle = 0;
  std::array<double, 1024> values{};
  std::array<char, 1024> error{};
  std::array<WireFault, 6> faults{};
};
static_assert(std::is_trivially_copyable_v<Frame>);
inline bool valid_frame(const Frame& f) {
  return f.magic == 0x4c504631 && f.version == 2 && f.count <= f.values.size() &&
         f.has_sdk_reply <= 1 && (f.arm == 0 || f.arm == 1);
}
inline void error_text(Frame& f, const char* s) {
  f.error.fill(0);
  std::memcpy(f.error.data(), s, std::min(std::strlen(s), f.error.size()));
}
}  // namespace lpf
