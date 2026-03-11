#!/bin/bash

# exit when any command fails
set -e

echo "os: '$OS_NAME'"

export QT_SELECT=5

if [[ "$1" == "--clean" ]]; then
    rm -rf ${WORKSPACE_BASE:-/app/workspace}/compass/build_$OS_NAME
fi

mkdir -p ${WORKSPACE_BASE:-/app/workspace}/compass/build_$OS_NAME
cd ${WORKSPACE_BASE:-/app/workspace}/compass/build_$OS_NAME
cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=RelWithDebInfo .. # -DCMAKE_PREFIX_PATH=$GL_PATH $CMAKE_OPTS
make -j $(nproc)
sudo make install

cd ${WORKSPACE_BASE:-/app/workspace}/compass/docker
