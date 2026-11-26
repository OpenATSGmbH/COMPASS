#!/bin/bash
#
# Cross build of jASTERIX and COMPASS for Windows in the compass/build_win_deb12 container.
#
# Usage (inside the container): win_build.sh [jasterix|compass|all] [--clean]
#   --clean  delete the build folders of the selected step first
#
# Paths (as build_compass.sh, WORKSPACE_BASE defaults to /app/workspace, see run_docker.sh):
#   $WORKSPACE_BASE/compass                   COMPASS source (with experimental_src)
#   $WORKSPACE_BASE/jasterix                  jASTERIX source
#   $WORKSPACE_BASE/compass/build_$OS_NAME    build output: jasterix_build, jasterix_install, compass_build

set -e

: "${OS_NAME:?OS_NAME is not set, rebuild the image with ./build_docker.sh win_deb12}"

BASE=${WORKSPACE_BASE:-/app/workspace}
COMPASS_SRC=$BASE/compass
JASTERIX_SRC=$BASE/jasterix
OUT=$COMPASS_SRC/build_$OS_NAME

STEP=all
CLEAN=0
for arg in "$@"; do
    case "$arg" in
        jasterix|compass|all) STEP=$arg ;;
        --clean)              CLEAN=1 ;;
        *)                    echo "unknown argument $arg"; exit 1 ;;
    esac
done

JOBS=$(nproc)
JASTERIX_INSTALL=$OUT/jasterix_install

build_jasterix()
{
    if [ $CLEAN == 1 ]; then rm -rf "$OUT/jasterix_build" "$JASTERIX_INSTALL"; fi

    # shared: linked by the shared compass_core
    cmake -G Ninja -S "$JASTERIX_SRC" -B "$OUT/jasterix_build" -DCMAKE_TOOLCHAIN_FILE=/opt/mingw_toolchain.cmake \
        -DCMAKE_INSTALL_PREFIX="$JASTERIX_INSTALL" -DBUILD_SHARED_LIBS=ON
    cmake --build "$OUT/jasterix_build" -j "$JOBS"
    cmake --install "$OUT/jasterix_build"
}

build_compass()
{
    if [ $CLEAN == 1 ]; then rm -rf "$OUT/compass_build"; fi

    # jASTERIX is outside the find root paths of the toolchain: set its paths directly
    cmake -G Ninja -S "$COMPASS_SRC" -B "$OUT/compass_build" -DCMAKE_TOOLCHAIN_FILE=/opt/mingw_toolchain.cmake \
        -DjASTERIX_INCLUDE_DIR="$JASTERIX_INSTALL/include" -DjASTERIX_LIBRARIES="$JASTERIX_INSTALL/lib/libjasterix.dll.a" \
        -DCMAKE_INSTALL_PREFIX="$OUT/compass_install"
    cmake --build "$OUT/compass_build" -j "$JOBS" -- -k 0
}

case "$STEP" in
    jasterix) build_jasterix ;;
    compass)  build_compass ;;
    all)      build_jasterix; build_compass ;;
esac
