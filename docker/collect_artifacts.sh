#!/bin/bash

# Collect build artifacts for CI runs
# Usage: collect_artifacts.sh <workspace> <pipeline_number> <branch> [os_name]

set -e

WORKSPACE="$1"
PIPELINE_NUM="$2"
BRANCH="$3"
OS="${4:-deb10}" # OS_NAME of the build image, names the AppImage

RUN_DIR="/home/user/ci/$(date +%Y%m%d-%H%M%S)-${PIPELINE_NUM}-${BRANCH}"
mkdir -p "$RUN_DIR/scripts"

cp "$WORKSPACE/COMPASS_${OS}-x86_64.AppImage" "$RUN_DIR/"
cp -a "$WORKSPACE/experimental_src/py/"* "$RUN_DIR/scripts/"
cp "$WORKSPACE/docker/run_ci_tests.sh" "$RUN_DIR/"

echo "Artifacts collected in $RUN_DIR"
ls -la "$RUN_DIR"
