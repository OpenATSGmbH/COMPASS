#!/bin/bash
#
# Cross build of jASTERIX and COMPASS for Windows in the compass/build_win_deb12 container.
#
# Usage (inside the container): win_build.sh [jasterix|compass|all]
#
# Mounts:
#   /src/compass   COMPASS source (with experimental_src)
#   /src/jasterix  jASTERIX source
#   /out           build output: jasterix_build, jasterix_install, compass_build
#
# Example (on the host):
#   docker run --rm --user $(id -u):$(id -g) \
#       -v ~/workspace/compass:/src/compass:ro -v ~/workspace/jasterix:/src/jasterix:ro \
#       -v ~/workspace/compass/build_win:/out compass/build_win_deb12 /src/compass/docker/win_build.sh all

set -e

STEP=${1:-all}
JOBS=$(nproc)
JASTERIX_INSTALL=/out/jasterix_install

build_jasterix()
{
    # shared: linked by the shared compass_core
    cmake -G Ninja -S /src/jasterix -B /out/jasterix_build -DCMAKE_TOOLCHAIN_FILE=/opt/mingw_toolchain.cmake \
        -DCMAKE_INSTALL_PREFIX="$JASTERIX_INSTALL" -DBUILD_SHARED_LIBS=ON
    cmake --build /out/jasterix_build -j "$JOBS"
    cmake --install /out/jasterix_build
}

build_compass()
{
    # jASTERIX is outside the find root paths of the toolchain: set its paths directly
    cmake -G Ninja -S /src/compass -B /out/compass_build -DCMAKE_TOOLCHAIN_FILE=/opt/mingw_toolchain.cmake \
        -DjASTERIX_INCLUDE_DIR="$JASTERIX_INSTALL/include" -DjASTERIX_LIBRARIES="$JASTERIX_INSTALL/lib/libjasterix.dll.a" \
        -DCMAKE_INSTALL_PREFIX=/out/compass_install
    cmake --build /out/compass_build -j "$JOBS" -- -k 0
}

case "$STEP" in
    jasterix) build_jasterix ;;
    compass)  build_compass ;;
    all)      build_jasterix; build_compass ;;
    *)        echo "unknown step $STEP"; exit 1 ;;
esac
