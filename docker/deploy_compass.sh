#!/bin/bash

# exit when any command fails
set -e

echo "os: '$OS_NAME'"

ASAN=0
for arg in "$@"; do
    case "$arg" in
        --asan) ASAN=1 ;;
    esac
done

export ARCH=x86_64
export QT_SELECT=5

cd ${WORKSPACE_BASE:-/app/workspace}/compass/
rm -rf appimage/appdir/*/ # deletes all subfolders
mkdir -p appimage/appdir/bin/

#cp /usr/bin/compass_client appimage/appdir/bin/
mkdir -p appimage/appdir/lib/
#cp /usr/lib/libcompass.a appimage/appdir/lib/

cp -r /usr/lib/osgPlugins-3.6.5 appimage/appdir/lib/ 2>/dev/null || true
cp -r /usr/lib/x86_64-linux-gnu/osgPlugins-3.6.5 appimage/appdir/lib/ 2>/dev/null || true
cp -r /usr/lib64/osgPlugins-3.6.5/ appimage/appdir/lib/ 2>/dev/null || true
cp /usr/lib64/libosgEarth* appimage/appdir/lib/ 2>/dev/null || true

mkdir -p appimage/appdir/compass/
cp -r data appimage/appdir/compass/
cp -r conf appimage/appdir/compass/
cp -r presets appimage/appdir/compass/

mkdir -p appimage/appdir/lib

export EXTRA_QT_PLUGINS="iconengines"
export APPIMAGE_EXTRACT_AND_RUN=1
export DEPLOY_GTK_VERSION=3

export NO_STRIP=1

# When building with AddressSanitizer, the compass binaries dynamically link
# against libasan.so.<N> (N depends on the GCC version used in the build image).
# linuxdeploy's default excludelist skips libasan, so force-include it explicitly
# or the AppImage fails to launch with "cannot open shared object file: libasan.so.<N>".
#
# Resolve the actual path via ldd on the built binary — authoritative across any
# GCC version (8 → libasan.so.5, 10 → .so.6, 12+ → .so.8).
LINUXDEPLOY_EXTRA=()
if [[ $ASAN -eq 1 ]]; then
    LIBASAN_PATH=$(ldd /usr/bin/compass_client 2>/dev/null | awk '/libasan/ {print $3; exit}')
    if [[ -n "$LIBASAN_PATH" && -f "$LIBASAN_PATH" ]]; then
        echo "AppImage bundling libasan: $LIBASAN_PATH"
        LINUXDEPLOY_EXTRA+=(--library="$LIBASAN_PATH")
    else
        echo "ERROR: ASAN=1 but ldd /usr/bin/compass_client shows no libasan link — the binary was not built with -fsanitize=address"
        exit 1
    fi
fi

cd ${WORKSPACE_BASE:-/app/workspace}/compass/docker/linuxdeploy/
./linuxdeploy-x86_64.AppImage --appdir ${WORKSPACE_BASE:-/app/workspace}/compass/appimage/appdir --executable=/usr/bin/compass_handler --executable=/usr/bin/compass_client --desktop-file=${WORKSPACE_BASE:-/app/workspace}/compass/appimage/compass.desktop --plugin qt --plugin gtk --icon-file ${WORKSPACE_BASE:-/app/workspace}/compass/appimage/ats.png --output appimage "${LINUXDEPLOY_EXTRA[@]}"

mv COMPASS-x86_64.AppImage ${WORKSPACE_BASE:-/app/workspace}/compass/COMPASS_$OS_NAME-x86_64.AppImage

cd ${WORKSPACE_BASE:-/app/workspace}/compass/docker


