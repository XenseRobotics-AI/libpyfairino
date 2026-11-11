#include <dlfcn.h>
#include <linux/audit.h>
#include <linux/filter.h>
#include <linux/seccomp.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#include "protocol.hpp"
namespace {
void deny_network() {
#if defined(__x86_64__)
  std::vector<sock_filter> f{{BPF_LD | BPF_W | BPF_ABS, 0, 0, offsetof(seccomp_data, arch)},
                             {BPF_JMP | BPF_JEQ | BPF_K, 1, 0, AUDIT_ARCH_X86_64},
                             {BPF_RET | BPF_K, 0, 0, SECCOMP_RET_KILL_PROCESS},
                             {BPF_LD | BPF_W | BPF_ABS, 0, 0, offsetof(seccomp_data, nr)}};
  // Deny x32 ABI and every network creation/connection/transmission syscall.
  f.push_back({BPF_JMP | BPF_JSET | BPF_K, 0, 1, 0x40000000});
  f.push_back({BPF_RET | BPF_K, 0, 0, SECCOMP_RET_ERRNO | EPERM});
  for (unsigned syscall : {SYS_io_uring_setup, SYS_socket, SYS_socketpair, SYS_connect, SYS_bind,
                           SYS_listen, SYS_accept, SYS_accept4, SYS_sendto, SYS_sendmsg,
                           SYS_sendmmsg, SYS_recvfrom, SYS_recvmsg, SYS_recvmmsg}) {
    f.push_back({BPF_JMP | BPF_JEQ | BPF_K, 0, 1, syscall});
    f.push_back({BPF_RET | BPF_K, 0, 0, SECCOMP_RET_ERRNO | EPERM});
  }
  f.push_back({BPF_RET | BPF_K, 0, 0, SECCOMP_RET_ALLOW});
  sock_fprog program{static_cast<unsigned short>(f.size()), f.data()};
  if (prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) ||
      prctl(PR_SET_SECCOMP, SECCOMP_MODE_FILTER, &program))
    throw std::runtime_error("cannot install deny-network seccomp filter");
#else
  throw std::runtime_error("deny-network policy is only supported on Linux x86_64");
#endif
}
}  // namespace
int main(int argc, char** argv) {
  if (argc != 4) return 64;
  int fd = std::atoi(argv[1]);
  try {
    if (std::string(argv[3]) == "deny-network")
      deny_network();
    else if (std::string(argv[3]) != "allow-network")
      return 64;
    char gate = 0;
    if (read(fd, &gate, 1) != 1 || gate != 1) return 65;
    void* library = dlopen(argv[2], RTLD_NOW | RTLD_LOCAL);
    if (!library) throw std::runtime_error(dlerror());
    using Entry = int (*)(int);
    auto entry = reinterpret_cast<Entry>(dlsym(library, "libpyfairino_backend_main"));
    if (!entry) throw std::runtime_error("missing backend entry");
    return entry(fd);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "libpyfairino worker: %s\n", e.what());
    return 70;
  }
}
