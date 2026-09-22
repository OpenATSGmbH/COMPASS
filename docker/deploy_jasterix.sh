#!/bin/bash

# exit when any command fails
set -e

echo "os: '$OS_NAME'"

export ARCH=x86_64
export APPIMAGE_EXTRACT_AND_RUN=1
export NO_STRIP=1

JASTERIX_DIR=${WORKSPACE_BASE:-/app/workspace}/jasterix
APPDIR=$JASTERIX_DIR/appimage/appdir

cd $JASTERIX_DIR/

# linuxdeploy does not overwrite files which already exist in the AppDir. Leftover
# content from an earlier run is packed again, so the new binary and the new libraries
# never reach the AppImage. Delete all generated content before each run. The
# top-level jasterix.desktop and atsdb.png symlinks are kept, they are in the
# repository and point into usr/share, which linuxdeploy fills again.
mkdir -p "$APPDIR"
find "$APPDIR" -mindepth 1 -maxdepth 1 -type d -exec rm -rf {} +
rm -f "$APPDIR/AppRun"

# linuxdeploy installs the executable into usr/bin and points AppRun there. The AppImage
# carried a second copy in appdir/bin and the static library in appdir/lib, about 180 MB
# uncompressed that nothing reads: AppRun and the desktop Exec entry resolve to usr/bin, and
# users who link jASTERIX build the library themselves.

cd ${WORKSPACE_BASE:-/app/workspace}/compass/docker/linuxdeploy/
./linuxdeploy-x86_64.AppImage --appdir $APPDIR --executable=/usr/bin/jasterix_client --desktop-file=$JASTERIX_DIR/appimage/jasterix.desktop --icon-file=$JASTERIX_DIR/appimage/atsdb.png --output appimage

mv jASTERIX*.AppImage $JASTERIX_DIR/jASTERIX_client_$OS_NAME-x86_64.AppImage

# smoke test: the AppImage must read the definitions of this source tree. This fails
# when an old binary was packed, e.g. one which still expects removed definition keys.
echo "checking AppImage against the definitions"
$JASTERIX_DIR/jASTERIX_client_$OS_NAME-x86_64.AppImage --definition_path $JASTERIX_DIR/definitions --print_cat_info > /dev/null

# zip adds to an existing archive, it never removes entries. Start from scratch, so
# that deleted definition files can not survive in the package.
rm -f $JASTERIX_DIR/jasterix_definitions.zip
cd $JASTERIX_DIR/definitions/
zip -r ../jasterix_definitions.zip . -x ".*" -x "*/.*" -x "*.md"

rm -f $JASTERIX_DIR/analyze.zip
cd ../analyze/
zip -r ../analyze.zip . -x ".*" -x "__*" -x "*/__*"

cd ${WORKSPACE_BASE:-/app/workspace}/compass/docker
