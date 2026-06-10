export ARCH=x86_64
export APPIMAGE_EXTRACT_AND_RUN=1

cd /app/workspace/jasterix/

# fresh AppDir - linuxdeploy populates usr/bin, usr/lib itself
rm -rf appimage/appdir/*/
mkdir -p appimage/appdir/

# build the AppImage with the same linuxdeploy used for COMPASS
# (jASTERIX is a non-Qt CLI tool, so no qt/gtk plugins are needed)
export OUTPUT=jASTERIX_client-x86_64.AppImage

/app/workspace/compass/docker/linuxdeploy/linuxdeploy-x86_64.AppImage \
    --appdir appimage/appdir \
    --executable=/usr/bin/jasterix_client \
    --desktop-file=appimage/appdir/jasterix.desktop \
    --icon-file=appimage/appdir/atsdb.png \
    --output appimage \
    --verbosity=2

cd definitions/
zip -r ../jasterix_definitions.zip .

cd ../analyze/
zip -r ../analyze.zip . -x ".*" -x "__*" -x "*/__*"


cd /app/workspace/compass/docker
