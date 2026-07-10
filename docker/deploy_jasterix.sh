#!/bin/bash

# exit when any command fails
set -e

echo "os: '$OS_NAME'"

export ARCH=x86_64
export APPIMAGE_EXTRACT_AND_RUN=1
export NO_STRIP=1

cd ${WORKSPACE_BASE:-/app/workspace}/jasterix/
mkdir -p appimage/appdir/bin/
cp /usr/bin/jasterix_client appimage/appdir/bin/
mkdir -p appimage/appdir/lib/
cp /usr/lib/libjasterix.a appimage/appdir/lib/

cd ${WORKSPACE_BASE:-/app/workspace}/compass/docker/linuxdeploy/
./linuxdeploy-x86_64.AppImage --appdir ${WORKSPACE_BASE:-/app/workspace}/jasterix/appimage/appdir --executable=/usr/bin/jasterix_client --desktop-file=${WORKSPACE_BASE:-/app/workspace}/jasterix/appimage/jasterix.desktop --icon-file=${WORKSPACE_BASE:-/app/workspace}/jasterix/appimage/atsdb.png --output appimage

mv jASTERIX*.AppImage ${WORKSPACE_BASE:-/app/workspace}/jasterix/jASTERIX_client_$OS_NAME-x86_64.AppImage

cd ${WORKSPACE_BASE:-/app/workspace}/jasterix/definitions/
zip -r ../jasterix_definitions.zip .

cd ../analyze/
zip -r ../analyze.zip . -x ".*" -x "__*" -x "*/__*"

cd ${WORKSPACE_BASE:-/app/workspace}/compass/docker
