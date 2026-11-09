// --pty mode of the integration test: the simulated MCU behind a pseudo-terminal, so the real host tool
// (tools/mico_link.py, including m0-accept) can drive it. A plain thread copies bytes between the pty and the
// simulated UART rings and plays the IWDG: 60 ms without a refresh re-executes the program as after a
// watchdog reset, keeping the same pty open.
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "board_sim.h"

namespace {

int master_fd = -1;
char* self_path = nullptr;

uint64_t now_ms() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000 + ts.tv_nsec / 1000000;
}

void* bridge(void*) {
  uint8_t buf[512];
  uint32_t last_refreshes = 0;
  uint64_t last_change = now_ms();
  for (;;) {
    pollfd p = {master_fd, POLLIN, 0};
    if (poll(&p, 1, 1) > 0 && (p.revents & POLLIN)) {
      ssize_t n = read(master_fd, buf, sizeof(buf));
      if (n > 0) sim::host_to_mcu.write(buf, static_cast<size_t>(n));   // dropped if full, like a UART
    }
    size_t m;
    while ((m = sim::mcu_to_host.read(buf, sizeof(buf))) > 0) {
      size_t off = 0;
      while (off < m) {
        ssize_t w = write(master_fd, buf + off, m - off);
        if (w > 0) off += static_cast<size_t>(w);
        else if (errno != EAGAIN && errno != EINTR) break;
      }
    }
    uint32_t r = sim::refreshes.load();
    if (r != last_refreshes) {
      last_refreshes = r;
      last_change = now_ms();
    } else if (sim::watchdog_started && now_ms() - last_change > 60) {
      char fd[16];
      snprintf(fd, sizeof(fd), "%d", master_fd);
      char* args[] = {self_path, const_cast<char*>("--pty-fd"), fd, const_cast<char*>("--watchdog-reset"),
                      nullptr};
      fprintf(stderr, "[sim] IWDG: no refresh for 60 ms, resetting\n");
      execv("/proc/self/exe", args);
      _Exit(4);
    }
  }
  return nullptr;
}

}  // namespace

// Called from main() before the scheduler starts. fd < 0: open a new pty and print its name.
void start_pty_bridge(char* argv0, int fd) {
  self_path = argv0;
  if (fd < 0) {
    fd = posix_openpt(O_RDWR | O_NOCTTY);
    if (fd < 0 || grantpt(fd) || unlockpt(fd)) {
      perror("pty");
      exit(1);
    }
    printf("%s\n", ptsname(fd));
    fflush(stdout);
  }
  master_fd = fd;
  fcntl(fd, F_SETFD, 0);   // keep across the simulated reset (execv)
  sigset_t all;
  sigfillset(&all);
  sigset_t old;
  pthread_sigmask(SIG_BLOCK, &all, &old);   // FreeRTOS's port signals must never land on this thread
  pthread_t t;
  pthread_create(&t, nullptr, bridge, nullptr);
  pthread_sigmask(SIG_SETMASK, &old, nullptr);
}
