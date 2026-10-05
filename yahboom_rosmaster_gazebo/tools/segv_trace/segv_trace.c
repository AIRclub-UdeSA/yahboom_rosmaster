/*
 * LD_PRELOAD library: when a chosen process takes a fatal signal, attach gdb to
 * it from inside the handler and write every thread's backtrace to a file, then
 * let the process die of the signal as it would have (exit status unchanged).
 *
 * It exists for issue #55, where image_bridge exits -11 at shutdown and the
 * launch sends it SIGINT with os.kill(pid), which rules out running it under a
 * gdb prefix (the signal would reach gdb, not the bridge).
 *
 *   SEGV_TRACE_COMMS  comma-separated /proc/<pid>/comm names to watch
 *                     (e.g. image_bridge,parameter_bridge)
 *   SEGV_TRACE_DIR    directory for <comm>.<pid>.bt files (default /tmp)
 *
 * Processes whose comm is not listed are untouched: no handler is installed.
 *
 * Build: gcc -shared -fPIC -O1 -o libsegv_trace.so segv_trace.c
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

static char g_comm[32];
static char g_dir[256];

static void handler(int sig, siginfo_t *info, void *ctx)
{
  (void)ctx;
  char path[400], pidstr[16], head[300];
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  snprintf(path, sizeof path, "%s/%s.%d.bt", g_dir, g_comm, (int)getpid());
  snprintf(pidstr, sizeof pidstr, "%d", (int)getpid());
  int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd >= 0) {
    int n = snprintf(
      head, sizeof head,
      "signal %d si_code %d si_addr %p faulting_tid %ld wall %ld.%09ld\n",
      sig, info->si_code, info->si_addr, (long)syscall(SYS_gettid),
      (long)ts.tv_sec, ts.tv_nsec);
    if (write(fd, head, n) < 0) {}
    close(fd);
  }
  prctl(PR_SET_PTRACER, PR_SET_PTRACER_ANY, 0, 0, 0);
  pid_t child = fork();
  if (child == 0) {
    char cmd[900];
    snprintf(
      cmd, sizeof cmd,
      "exec gdb -batch -p %s -ex 'info threads' -ex 'thread apply all bt 40' "
      "-ex 'info sharedlibrary' >> '%s' 2>&1", pidstr, path);
    char *envp[] = {(char *)"PATH=/usr/bin:/bin", (char *)"HOME=/tmp", NULL};
    char *argv[] = {(char *)"sh", (char *)"-c", cmd, NULL};
    execve("/bin/sh", argv, envp);
    _exit(127);
  }
  if (child > 0) {
    int status;
    waitpid(child, &status, 0);
  }
  signal(sig, SIG_DFL);
  /* Returning re-executes the faulting instruction, which now kills us. For a
   * signal raised by raise()/abort() re-raise it. */
  if (sig != SIGSEGV && sig != SIGBUS) {
    raise(sig);
  }
}

__attribute__((constructor)) static void init(void)
{
  const char *comms = getenv("SEGV_TRACE_COMMS");
  if (!comms) {return;}
  FILE *f = fopen("/proc/self/comm", "r");
  if (!f) {return;}
  if (!fgets(g_comm, sizeof g_comm, f)) {fclose(f); return;}
  fclose(f);
  g_comm[strcspn(g_comm, "\n")] = 0;
  char list[512];
  snprintf(list, sizeof list, ",%s,", comms);
  char key[48];
  snprintf(key, sizeof key, ",%s,", g_comm);
  if (!strstr(list, key)) {return;}
  const char *dir = getenv("SEGV_TRACE_DIR");
  snprintf(g_dir, sizeof g_dir, "%s", dir ? dir : "/tmp");
  struct sigaction sa;
  memset(&sa, 0, sizeof sa);
  sa.sa_sigaction = handler;
  sa.sa_flags = SA_SIGINFO | SA_RESETHAND;
  sigaction(SIGSEGV, &sa, NULL);
  sigaction(SIGBUS, &sa, NULL);
  sigaction(SIGABRT, &sa, NULL);
}
