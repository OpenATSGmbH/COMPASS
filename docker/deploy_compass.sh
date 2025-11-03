#!/bin/bash

# exit when any command fails
set -e

echo "os: '$1'"

export ARCH=x86_64
export QT_SELECT=5

cd /app/workspace/compass/
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

cd /app/workspace/compass/docker/linuxdeploy/
./linuxdeploy-x86_64.AppImage --appdir /app/workspace/compass/appimage/appdir --executable=/usr/bin/compass_handler --executable=/usr/bin/compass_client --desktop-file=/app/workspace/compass/appimage/compass.desktop --plugin qt --plugin gtk --icon-file /app/workspace/compass/appimage/ats.png --output appimage

mv COMPASS-x86_64.AppImage /app/workspace/compass/COMPASS_$1-x86_64.AppImage

cd /app/workspace/compass/docker


