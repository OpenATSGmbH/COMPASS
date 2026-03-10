#!/bin/bash

# Run Python integration tests using the AppImage
# Usage: run_ci_tests.sh <test_modules> <pipeline_number> <branch>

set -eo pipefail

MODULES="$1"
PIPELINE_NUM="$2"
BRANCH="$3"

if [ -z "$MODULES" ]; then
    echo "No test modules specified, skipping"
    exit 0
fi

# Find the latest run directory for this pipeline
RUN_DIR=$(ls -dt /home/user/ci/*-${PIPELINE_NUM}-${BRANCH} 2>/dev/null | head -1)

if [ -z "$RUN_DIR" ]; then
    echo "ERROR: No artifact directory found for pipeline $PIPELINE_NUM branch $BRANCH"
    exit 1
fi

echo "Run directory: $RUN_DIR"

APPIMAGE="$RUN_DIR/COMPASS_deb10-x86_64.AppImage"
SCRIPTS="$RUN_DIR/scripts"
TEST_PATH="$SCRIPTS/tests"
DATA_PATH="/home/user/ci/test"
LOG_DIR="$RUN_DIR/logs"

mkdir -p "$LOG_DIR"

if [ ! -f "$APPIMAGE" ]; then
    echo "ERROR: AppImage not found at $APPIMAGE"
    exit 1
fi

chmod +x "$APPIMAGE"

echo "AppImage: $APPIMAGE"
echo "Test path: $TEST_PATH"
echo "Data path: $DATA_PATH"
echo "Log dir: $LOG_DIR"
echo "Modules: $MODULES"
echo ""

cd "$SCRIPTS/test"
export PYTHONPATH="$SCRIPTS"

python3 test_suite.py \
    --binary="$APPIMAGE" \
    --path="$TEST_PATH" \
    --data="$DATA_PATH" \
    --modules="$MODULES" \
    --no-prompt \
    --cfg-override=none \
    2>&1 | tee "$LOG_DIR/test_${MODULES}.log"

echo ""
echo "Test logs saved to $LOG_DIR/test_${MODULES}.log"
