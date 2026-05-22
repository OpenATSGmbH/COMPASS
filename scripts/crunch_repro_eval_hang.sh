#!/bin/bash
# ---------------------------------------------------------------------------
# crunch_repro_eval_hang.sh - reproduce the eval-shutdown heap-corruption hang
#                             on crunch and capture its backtrace in gdb.
#
# Purpose
#   CI build #50 (and similar) fail at tests like eval_mlat_all / eval_radar
#   with "application terminated (0)" / "application crashed (-9)" after the
#   previous eval's COMPASS is asked to quit. The bug does NOT reproduce by
#   running a single eval test - it needs at least two evals back-to-back so
#   the second one's open_db triggers the (failing) quit of the first one.
#
#   This script runs eval_adsb_dubious followed by eval_mlat_all on crunch,
#   while a companion watcher (gdb_watch_sigabrt.sh) waits for the second
#   restart's quit marker and attaches gdb to compass_client. gdb catches
#   SIGABRT without delivering it, letting us dump all thread backtraces.
#
# Why these two tests
#   eval_mlat_all has restart=True on its open_db, so its first action is
#   "quit the previous COMPASS". If the previous COMPASS is a post-eval
#   instance (i.e. eval_adsb_dubious), shutdown hits the bug. A fresh
#   calculate_references instance (no eval run) does NOT trigger it.
#
# What you need on crunch
#   - An extracted AppImage at $EXTRACT_DIR/squashfs-root/ with debug info
#     (the stock AppImage has .debug_info, it is not stripped)
#   - The scripts/ dir in $SCRIPTS_DIR
#   - Test data at /home/user/ci/test/at_20230422/
#   - ptrace_scope=0 (default on crunch) so gdb can attach without sudo
#
# Why PYTHONUNBUFFERED=1 is mandatory
#   The watcher polls the test log for "quitting running compass instance".
#   Without unbuffered stdout, Python holds that line in a buffer until the
#   process exits - long after the crash has already happened. Line buffering
#   makes the trigger appear in real time.
#
# Why --no_highdpi -r is used
#   Matches Jenkins CI config (COMPASS_EXTRA_ARGS in Jenkinsfile). Some
#   display-server environments on crunch scale the UI oddly without it;
#   -r resets user config for a clean test run.
#
# Usage
#   ssh user@crunch.local
#   /path/to/compass/scripts/crunch_repro_eval_hang.sh \
#       /home/user/ci/<timestamp>-<build>-<branch>
#
#   The build artifact dir must contain:
#       COMPASS_deb10-x86_64.AppImage
#       scripts/test_infra/test_suite.py
#       scripts/tests/
# ---------------------------------------------------------------------------

set -eu

BUILD_DIR=${1:?Usage: $0 <build-artifact-dir>}
APPIMAGE="$BUILD_DIR/COMPASS_deb10-x86_64.AppImage"
SCRIPTS_DIR="$BUILD_DIR/scripts"
MANIFEST="/home/user/ci/test/at_20230422/at_20230422_05h.json"
TEST_OUTPUT="/home/user/ci/test"

EXTRACT_DIR=${EXTRACT_DIR:-/tmp/compass_extracted}
TESTLOG=${TESTLOG:-/tmp/repro_test.log}
GDBLOG=${GDBLOG:-/tmp/repro_gdb.log}

# location of the watcher - same dir as this script
WATCHER_SCRIPT="$(cd "$(dirname "$0")" && pwd)/gdb_watch_sigabrt.sh"
if [ ! -x "$WATCHER_SCRIPT" ]; then
    echo "error: watcher not found or not executable: $WATCHER_SCRIPT" >&2
    exit 1
fi

# kill any lingering COMPASS processes from earlier runs (force-kill is safe -
# they would otherwise confuse pidof / block AppImage mount ports).
for N in COMPASS_deb10-x86_64.AppImage compass_client compass_handler; do
    pkill -9 "$N" 2>/dev/null || true
done
sleep 1

# extract AppImage to a stable path so gdb can find the binary with debug
# symbols (the running FUSE mount path changes each invocation and is
# unreadable by root anyway - see gdb_watch_sigabrt.sh header).
rm -rf "$EXTRACT_DIR"
mkdir -p "$EXTRACT_DIR"
(cd "$EXTRACT_DIR" && "$APPIMAGE" --appimage-extract > /dev/null 2>&1)

# fresh logs
rm -f "$TESTLOG" "$GDBLOG"

# launch the watcher in the background. It waits for the 2nd occurrence of
# "quitting running compass instance" - the first fires at eval_adsb_dubious
# (quits whatever was open from calculate_references), the second fires at
# eval_mlat_all (quits eval_adsb_dubious's COMPASS - the one that hangs).
APPIMAGE_EXTRACT_DIR="$EXTRACT_DIR" \
TESTLOG="$TESTLOG" GDBLOG="$GDBLOG" \
TRIGGER_RE='quitting running compass instance' TRIGGER_COUNT=2 \
PROCESS_NAME=compass_client \
nohup "$WATCHER_SCRIPT" > /tmp/watcher.stdout 2>&1 &
WATCHER=$!
echo "watcher pid $WATCHER (log: $GDBLOG)"

# run test_suite.py with unbuffered stdout so the watcher sees events live.
cd "$SCRIPTS_DIR/test_infra"
DISPLAY=:0 \
PYTHONUNBUFFERED=1 \
COMPASS_EXTRA_ARGS="--no_highdpi -r" \
python3 -u test_suite.py \
    --binary="$APPIMAGE" \
    --path="$SCRIPTS_DIR/tests" \
    --manifest="$MANIFEST" \
    --output="$TEST_OUTPUT" \
    --tests=eval_adsb_dubious,eval_mlat_all \
    --deps=tests \
    --no-prompt \
    --cfg-override=none \
    > "$TESTLOG" 2>&1 || true

# wait for the watcher to finish (its own gdb timeout caps this at ~150s).
for _ in $(seq 1 240); do
    kill -0 "$WATCHER" 2>/dev/null || break
    sleep 1
done

echo "=== test result tail ==="
tail -15 "$TESTLOG"
echo
echo "=== gdb log ($GDBLOG, $(wc -l < "$GDBLOG") lines) ==="
tail -60 "$GDBLOG"
