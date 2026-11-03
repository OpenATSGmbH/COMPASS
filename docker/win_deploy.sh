#!/bin/bash
#
# Collects the Windows build of COMPASS with all runtime files into /out/compass_deploy,
# in the same layout as the AppImage (see Utils::System::appDir):
#
#   compass_client.exe, compass_handler.exe, *.dll   executables and runtime libraries
#   platforms/, styles/, imageformats/, iconengines/  Qt plugins
#   lib/osgPlugins-<version>/                         OSG/osgEarth plugins
#   compass/conf, compass/data                        COMPASS configuration and data
#   proj/                                             PROJ data (proj.db)
#
# Usage (inside the container, after win_build.sh): win_deploy.sh [--with_tests]

set -e

BUILD=/out/compass_build
DEPLOY=/out/compass_deploy
STRIP=x86_64-w64-mingw32-strip

rm -rf "$DEPLOY"
mkdir -p "$DEPLOY/lib" "$DEPLOY/compass"

# executables without debug information (about 2 GB each with it)
EXES="compass_client.exe compass_handler.exe"
if [ "$1" == "--with_tests" ]; then EXES="$EXES compass_tests.exe"; fi
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

# COMPASS configuration and data
cp -r /src/compass/conf /src/compass/data "$DEPLOY/compass/"

echo "deployed to $DEPLOY"
du -sh "$DEPLOY"
