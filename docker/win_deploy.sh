#!/bin/bash
#
# Collects the Windows build of COMPASS with all runtime files into /out/compass_deploy,
# in the same layout as the AppImage (see Utils::System::appDir):
#
#   compass_client.exe, compass_handler.exe, *.dll   executables and runtime libraries
#   platforms/, styles/, imageformats/, iconengines/  Qt plugins
#   lib/osgPlugins-<version>/                         OSG/osgEarth plugins
#   compass/conf, compass/data, compass/presets       COMPASS configuration, data, view presets
#   proj/                                             PROJ data (proj.db)
#
# DLLs that nothing imports are removed (win_unused_dlls.py). The remaining imports are
# Windows system DLLs only (Windows 10 or later).
#
# Usage (inside the container, after win_build.sh): win_deploy.sh [--with_tests] [--zip]
#   --with_tests  also compass_tests.exe (not for the zip file)
#   --zip         /out/COMPASS_<version>_win64.zip with the folder COMPASS_<version>_win64,
#                 the Windows counterpart of the AppImage

set -e

BUILD=/out/compass_build
DEPLOY=/out/compass_deploy
STRIP=x86_64-w64-mingw32-strip

rm -rf "$DEPLOY"
mkdir -p "$DEPLOY/lib" "$DEPLOY/compass"

WITH_TESTS=0
ZIP=0
for arg in "$@"; do
    case "$arg" in
        --with_tests) WITH_TESTS=1 ;;
        --zip)        ZIP=1 ;;
        *)            echo "unknown option $arg"; exit 1 ;;
    esac
done

EXES="compass_client.exe compass_handler.exe"
if [ $WITH_TESTS == 1 ]; then EXES="$EXES compass_tests.exe"; fi

# executables without debug information (about 2 GB each with it)
for exe in $EXES; do
    $STRIP -o "$DEPLOY/$exe" "$BUILD/bin/$exe"
done

# runtime of the MinGW toolchain (libssp: stack protection, needed by TBB)
cp "$MINGW_GCC_DLL_DIR/libstdc++-6.dll" "$MINGW_GCC_DLL_DIR/libgcc_s_seh-1.dll" \
   "$MINGW_GCC_DLL_DIR/libssp-0.dll" "$MINGW_SYS_DLL_DIR/libwinpthread-1.dll" "$DEPLOY/"

# libraries built in the image, OSG plugins in lib/ as in the AppImage
cp "$WIN64_PREFIX"/bin/*.dll "$DEPLOY/"
cp -r "$WIN64_PREFIX"/bin/osgPlugins-* "$DEPLOY/lib/"
cp -r "$WIN64_PREFIX/share/proj" "$DEPLOY/proj"

# Qt libraries and plugins
cp "$QT_WIN_DIR"/bin/Qt5*.dll "$DEPLOY/"
for plugin_dir in platforms styles imageformats iconengines; do
    if [ -d "$QT_WIN_DIR/plugins/$plugin_dir" ]; then
        cp -r "$QT_WIN_DIR/plugins/$plugin_dir" "$DEPLOY/"
    fi
done

# COMPASS configuration, data and view presets (as in the AppImage), licence
cp -r /src/compass/conf /src/compass/data /src/compass/presets "$DEPLOY/compass/"
cp /src/compass/LICENSE "$DEPLOY/"

# DLLs that nothing imports (e.g. Qt modules COMPASS does not use)
python3 /src/compass/docker/win_unused_dlls.py "$DEPLOY" --remove | tail -1

echo "deployed to $DEPLOY"
du -sh "$DEPLOY"

if [ $ZIP == 1 ]; then
    VERSION=$(grep -oP 'CPACK_PACKAGE_VERSION_(MAJOR|MINOR|PATCH) "\K[0-9]+' /src/compass/CMakeLists.txt \
              | paste -sd.)
    NAME="COMPASS_${VERSION}_win64"
    STAGE=/out/zip_stage
    rm -rf "$STAGE" "/out/$NAME.zip"
    mkdir -p "$STAGE"
    ln -s "$DEPLOY" "$STAGE/$NAME"
    (cd "$STAGE" && 7z a -tzip -mx=7 -l "/out/$NAME.zip" "$NAME" > /dev/null)
    rm -rf "$STAGE"
    echo "zip file /out/$NAME.zip"
    ls -la "/out/$NAME.zip"
fi
