#!/bin/bash
#
# Windows build in the compass/build_win_deb12 container, the counterpart of make_appimage.sh:
# jASTERIX and COMPASS (win_build.sh), then the ZIP file (win_deploy.sh --zip).
#
# Usage (inside the container): make_windows_build.sh [--clean]
#   --clean  delete the build folders first
#
# Result: ../build_win_deb12/COMPASS_<version>_win64.zip

# exit when any command fails
set -e

echo "os: '$OS_NAME'"

CLEAN=""
if [[ "$1" == "--clean" ]]; then
    CLEAN="--clean"
fi

./win_build.sh all $CLEAN
./win_deploy.sh --zip
