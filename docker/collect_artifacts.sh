#!/bin/bash

# Collect build artifacts for CI runs
# Usage: collect_artifacts.sh <workspace> <run_dir> <os_name>
#   os_name: OS_NAME of the build image, selects the artifact
#     Linux (e.g. alma8): COMPASS_<os>-x86_64.AppImage
#     win_deb12:          COMPASS_<version>_win64.zip (win_deploy.sh --zip), unpacked into win64/ for
#                         the integration tests, plus compass_client_wine.sh to start it in Wine
# Called once per built OS, all artifacts of a build go into the same run dir.

set -e

WORKSPACE="$1"
RUN_DIR="$2"
OS="$3"

mkdir -p "$RUN_DIR/scripts"

if [[ "$OS" == win_* ]]; then
    # newest ZIP file: the build dir keeps the ones of earlier versions
    ZIP=$(ls -t "$WORKSPACE/build_$OS"/COMPASS_*_win64.zip | head -1)
    cp "$ZIP" "$RUN_DIR/"
    rm -rf "$RUN_DIR/win64"
    unzip -q "$ZIP" -d "$RUN_DIR/win64"
    cp "$WORKSPACE/docker/win_test/compass_client_wine.sh" "$RUN_DIR/"
else
    cp "$WORKSPACE/COMPASS_${OS}-x86_64.AppImage" "$RUN_DIR/"
fi

cp -a "$WORKSPACE/experimental_src/py/"* "$RUN_DIR/scripts/"
cp "$WORKSPACE/docker/run_ci_tests.sh" "$RUN_DIR/"

echo "Artifacts collected in $RUN_DIR"
ls -la "$RUN_DIR"
