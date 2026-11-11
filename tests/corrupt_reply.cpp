// Test-only IPC fault injection. Never installed; preserves the real fake-SDK call first.
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>

#include "protocol.hpp"
extern "C" ssize_t __real_write(int, const void*, size_t);
extern "C" ssize_t __wrap_write(int fd, const void* data, size_t size) {
  const char* mode = std::getenv("LPF_TEST_CORRUPT_REPLY");
  const char* init_mode = std::getenv("LPF_TEST_INIT_FRAME");
  if ((mode || init_mode) && size == sizeof(lpf::Frame)) {
    lpf::Frame frame;
    std::memcpy(&frame, data, sizeof(frame));
    if (init_mode && frame.op == lpf::Op::Initialize) {
      const std::string which = init_mode;
      if (which == "failure_to_ready" && frame.tag == lpf::Tag::Reply)
        frame.tag = lpf::Tag::Ready;
      else if (frame.tag == lpf::Tag::Ready) {
        if (which == "code")
          frame.code = -2;
        else if (which == "sdk_flag")
          frame.has_sdk_reply = 1;
        else if (which == "count")
          frame.count = 1;
        else if (which == "error")
          frame.error[0] = 'E';
        else if (which == "error_tail")
          frame.error.back() = 'E';
        else if (which == "fault_id")
          frame.faults[0].id = 1;
        else if (which == "fault_code")
          frame.faults[0].code = -3;
        else if (which == "fault_flag")
          frame.faults[0].has_sdk_reply = 1;
        else if (which == "sequence")
          ++frame.sequence;
        else if (which == "operation")
          frame.op = lpf::Op::ServoJ;
        else if (which == "arm")
          frame.arm = 1;
        else if (which == "version")
          ++frame.version;
        else if (which == "stage")
          frame.tag = lpf::Tag::Stopped;
        else if (which == "short")
          return __real_write(fd, &frame, sizeof(frame) - 1);
        else if (which == "exit0")
          std::_Exit(0);
        else if (which == "unused") {
          frame.arg0 = 456;
          frame.arg1 = -789;
          frame.values[0] = std::numeric_limits<double>::quiet_NaN();
          frame.speed = 123;
        }
      }
      return __real_write(fd, &frame, sizeof(frame));
    }
    if (mode && frame.op == lpf::Op::ServoJ && frame.arm == 1 && frame.tag == lpf::Tag::Reply) {
      frame.tag = lpf::Tag::Ready;  // valid identity and actual SDK reply, invalid operation phase
      return __real_write(fd, &frame, sizeof(frame));
    }
  }
  return __real_write(fd, data, size);
}
