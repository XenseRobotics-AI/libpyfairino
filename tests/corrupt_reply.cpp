// Test-only IPC fault injection. Never installed; preserves the real fake-SDK call first.
#include <unistd.h>

#include <cstdlib>
#include <cstring>

#include "protocol.hpp"
extern "C" ssize_t __real_write(int, const void*, size_t);
extern "C" ssize_t __wrap_write(int fd, const void* data, size_t size) {
  const char* mode = std::getenv("LPF_TEST_CORRUPT_REPLY");
  if (mode && size == sizeof(lpf::Frame)) {
    lpf::Frame frame;
    std::memcpy(&frame, data, sizeof(frame));
    if (frame.op == lpf::Op::ServoJ && frame.arm == 1 && frame.tag == lpf::Tag::Reply) {
      frame.tag = lpf::Tag::Ready;  // valid identity and actual SDK reply, invalid operation phase
      return __real_write(fd, &frame, sizeof(frame));
    }
  }
  return __real_write(fd, data, size);
}
