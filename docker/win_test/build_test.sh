#!/bin/bash
#
# Builds one test program in the compass/build_win_deb12 container and deploys it with
# all runtime DLLs, OSG plugins and data into /out/deploy.
#
# Usage (inside the container): build_test.sh <test_source_dir> <out_dir>

set -e

SRC=$1
OUT=$2
DEPLOY=$OUT/deploy

cmake -G Ninja -S "$SRC" -B "$OUT/build" -DCMAKE_TOOLCHAIN_FILE=/opt/mingw_toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$DEPLOY"
cmake --build "$OUT/build"
cmake --install "$OUT/build"

# runtime of the MinGW toolchain
cp "$MINGW_GCC_DLL_DIR/libstdc++-6.dll" "$MINGW_GCC_DLL_DIR/libgcc_s_seh-1.dll" \
   "$MINGW_SYS_DLL_DIR/libwinpthread-1.dll" "$DEPLOY/"

# libraries built in the image
if [ -d "$WIN64_PREFIX/bin" ]; then
    cp "$WIN64_PREFIX"/bin/*.dll "$DEPLOY/"
    for dir in "$WIN64_PREFIX"/bin/osgPlugins-* "$WIN64_PREFIX"/lib/osgPlugins-*; do
        if [ -d "$dir" ]; then cp -r "$dir" "$DEPLOY/"; fi
    done
    if [ -d "$WIN64_PREFIX/share/proj" ]; then cp -r "$WIN64_PREFIX/share/proj" "$DEPLOY/proj_data"; fi
    if [ -d "$WIN64_PREFIX/share/gdal" ]; then cp -r "$WIN64_PREFIX/share/gdal" "$DEPLOY/gdal_data"; fi
    if [ -d "$WIN64_PREFIX/share/osgearth_test_data" ]; then
        mkdir -p "$DEPLOY/data" && cp "$WIN64_PREFIX"/share/osgearth_test_data/* "$DEPLOY/data/"
    fi
fi

echo "deployed to $DEPLOY"
