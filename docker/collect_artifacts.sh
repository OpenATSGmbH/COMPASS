#!/bin/bash

# Collect build artifacts for CI runs
# Usage: collect_artifacts.sh <workspace> <pipeline_number> <branch>

set -e

WORKSPACE="$1"
PIPELINE_NUM="$2"
BRANCH="$3"

RUN_DIR="/home/user/ci/$(date +%Y%m%d-%H%M%S)-${PIPELINE_NUM}-${BRANCH}"
mkdir -p "$RUN_DIR/scripts"

cp "$WORKSPACE/COMPASS_deb10-x86_64.AppImage" "$RUN_DIR/"
cp -a "$WORKSPACE/experimental_src/py/"* "$RUN_DIR/scripts/"
cp "$WORKSPACE/docker/run_ci_tests.sh" "$RUN_DIR/"

echo "Artifacts collected in $RUN_DIR"
ls -la "$RUN_DIR"
