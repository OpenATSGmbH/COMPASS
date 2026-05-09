#!/bin/bash
# ---------------------------------------------------------------------------
# gdb_watch_sigabrt.sh - preemptive gdb attach to catch SIGABRT (or SEGV/BUS)
#
# Purpose
#   When a COMPASS process aborts deep inside library code (glibc heap check,
#   Qt teardown, boost::stacktrace), the signal handler's own output is often
#   partial or missing, and any core dump gets filtered by systemd-coredump
#   (ProcessSizeMax=1G). This script attaches gdb *before* the crash point,
#   waits for the signal, and captures symbolic backtraces of all threads.
#
# How it works
#   1. Tails TESTLOG until TRIGGER_RE has appeared TRIGGER_COUNT times.
#      (Counting occurrences lets you catch the second/Nth restart cycle.)
#   2. Resolves the PID of PROCESS_NAME via pgrep.
#   3. Attaches gdb with:
#        - handle SIG* stop print nopass  → gdb breaks on signal and does NOT
#          deliver it to the program, so the process stays inspectable.
#        - "continue" in --batch mode runs until the next signal.
#   4. Dumps info threads and thread apply all bt 60 on stop.
#   5. Detaches cleanly - the test framework continues its normal path.
#
# Why the helper shell does the attach, not gdb directly
#   The gdb attach needs to happen at a very specific moment (right before
#   the suspected abort). Polling a log file for a trigger is easier in bash
#   than from gdb. Also, sudo is avoided deliberately - see below.
#
# Why no sudo
#   ptrace_scope=0 on crunch lets the process owner attach without root.
#   Running gdb as root would fail to read the AppImage FUSE mount (FUSE
#   mounts are owner-only unless allow_other is set), so symbol loading
#   would fail with "?? ()" frames everywhere.
#
# Why symbols are loaded from an extracted AppImage
#   /proc/PID/exe points into the running AppImage's FUSE mount (e.g.
#   /tmp/.mount_COMPAS*/usr/bin/compass_client). That mount path varies per
#   run and is unreadable by root. Extracting the AppImage once to a stable
#   path (APPIMAGE_EXTRACT_DIR) gives gdb a reliable place to find debug
#   info. The binary's build-id matches regardless, so gdb maps symbols
#   onto the running process correctly.
#
# Why the test runner must be unbuffered
#   Python's stdout is block-buffered when redirected to a file. Messages
#   like "quitting running compass instance" sit in the buffer until the
#   process ends, so the watcher would trigger after the event it was
#   supposed to catch. The runner MUST set PYTHONUNBUFFERED=1 (or use
#   `python3 -u`) - otherwise this script can't find its trigger in time.
#
# Environment / arguments
#   TESTLOG            path to the test framework's live log (required)
#   GDBLOG             path to write gdb output to (required)
#   TRIGGER_RE         grep regex that marks the moment to attach
#                        default: 'quitting running compass instance'
#   TRIGGER_COUNT      wait until the regex has matched N times
#                        default: 2 (skip the first restart, catch the second)
#   PROCESS_NAME       pgrep -x target (default: compass_client)
#   APPIMAGE_EXTRACT_DIR  root of an extracted AppImage (required for symbols)
#                          (the dir that *contains* squashfs-root/)
#   GDB_TIMEOUT_SEC    hard kill timeout for the gdb batch (default: 150)
#
# Example
#   APPIMAGE_EXTRACT_DIR=/tmp/compass_extracted \
#   TESTLOG=/tmp/repro_test.log GDBLOG=/tmp/repro_gdb.log \
#   ./gdb_watch_sigabrt.sh &
# ---------------------------------------------------------------------------

set -u

TESTLOG=${TESTLOG:?TESTLOG not set}
GDBLOG=${GDBLOG:?GDBLOG not set}
TRIGGER_RE=${TRIGGER_RE:-quitting running compass instance}
TRIGGER_COUNT=${TRIGGER_COUNT:-2}
PROCESS_NAME=${PROCESS_NAME:-compass_client}
APPIMAGE_EXTRACT_DIR=${APPIMAGE_EXTRACT_DIR:?APPIMAGE_EXTRACT_DIR not set (path containing squashfs-root/)}
GDB_TIMEOUT_SEC=${GDB_TIMEOUT_SEC:-150}

: > "$GDBLOG"
log() { echo "[$(date +%H:%M:%S.%3N)] $*" >> "$GDBLOG"; }
log "watcher start (trigger='$TRIGGER_RE' count=$TRIGGER_COUNT proc=$PROCESS_NAME)"

# wait for the test framework to start writing to the log
while [ ! -f "$TESTLOG" ]; do sleep 0.1; done
log "testlog exists"

# busy-poll the log file for the Nth occurrence of the trigger phrase.
# grep -c is cheap; 50 ms sleep keeps CPU negligible while staying responsive
# to the short window between trigger print and the abort fire.
while :; do
    CNT=$(grep -c "$TRIGGER_RE" "$TESTLOG" 2>/dev/null || echo 0)
    if [ "$CNT" -ge "$TRIGGER_COUNT" ]; then break; fi
    sleep 0.05
done
log "trigger matched ($CNT x) - locating process"

CLIENT_PID=$(pgrep -x "$PROCESS_NAME" | head -1)
log "process pid: ${CLIENT_PID:-<none>}"
if [ -z "$CLIENT_PID" ]; then
    log "no pid found - aborting gdb attach"
    exit 0
fi

CLIENT_EXE="$APPIMAGE_EXTRACT_DIR/squashfs-root/usr/bin/$PROCESS_NAME"
LIB_DIR="$APPIMAGE_EXTRACT_DIR/squashfs-root/usr/lib"
if [ ! -x "$CLIENT_EXE" ]; then
    log "WARN: $CLIENT_EXE not found - symbols will be missing"
fi

log "attaching gdb (file=$CLIENT_EXE libs=$LIB_DIR)"

# --batch: run commands and exit
# -iex runs BEFORE inferior/file commands; -ex runs after
# set auto-load safe-path /  → allow gdb to load gdb-scripts from any path
# handle SIG* stop print nopass:
#   stop   → break into gdb on signal
#   print  → announce it
#   nopass → do NOT deliver the signal to the program (process stays alive)
#            this means we can inspect, but then also must detach before
#            the program can proceed past the trap point.
# "continue" in batch mode runs until a break occurs (handled signal or exit).
timeout "$GDB_TIMEOUT_SEC" gdb --batch \
    -iex "set pagination off" \
    -iex "set height 0" -iex "set width 0" -iex "set confirm off" \
    -iex "set solib-search-path $LIB_DIR" \
    -iex "set auto-load safe-path /" \
    -ex "file $CLIENT_EXE" \
    -ex "attach $CLIENT_PID" \
    -ex "handle SIGABRT stop print nopass" \
    -ex "handle SIGSEGV stop print nopass" \
    -ex "handle SIGBUS  stop print nopass" \
    -ex "echo === gdb: continuing inferior ===\n" \
    -ex "continue" \
    -ex "echo === gdb: stopped (signal or exit) ===\n" \
    -ex "info program" \
    -ex "info threads" \
    -ex "thread apply all bt 60" \
    -ex "echo === gdb: detaching ===\n" \
    -ex "detach" \
    -ex "quit" >> "$GDBLOG" 2>&1

log "gdb batch exited rc=$?"
log "watcher done"
