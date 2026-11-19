#!/bin/bash
#
# Non-interactive Windows build (make_windows_build.sh) in the compass/build_win_deb12 container,
# the counterpart of run_make_appimage.sh. Works over SSH without a TTY.
#
# Usage (in the docker folder): run_make_windows_build.sh [--clean]

# exit when any command fails
set -e

CLEAN=${1:+"$1"}

docker run --rm --user $(id -u):$(id -g) -v $PWD/../../..:/app compass/build_win_deb12 ./make_windows_build.sh $CLEAN
