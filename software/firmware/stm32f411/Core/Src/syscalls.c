/* Minimal newlib stubs. The firmware never uses malloc: _sbrk refuses and counts the attempt, so an
 * accidental allocation shows up in Diag (allocs_after_start) instead of silently growing the heap. */
#include <errno.h>
#include <stdint.h>
#include <sys/stat.h>

volatile uint32_t mico_sbrk_calls;

void* _sbrk(ptrdiff_t incr) {
  (void)incr;
  ++mico_sbrk_calls;
  errno = ENOMEM;
  return (void*)-1;
}

int _close(int fd) { (void)fd; return -1; }
int _fstat(int fd, struct stat* st) { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd) { (void)fd; return 1; }
int _lseek(int fd, int off, int whence) { (void)fd; (void)off; (void)whence; return 0; }
int _read(int fd, char* p, int n) { (void)fd; (void)p; (void)n; return 0; }
int _write(int fd, const char* p, int n) { (void)fd; (void)p; return n; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; errno = EINVAL; return -1; }
void _exit(int status) { (void)status; for (;;) {} }
