#!/bin/bash

# exit when any command fails
set -e

echo "os: '$OS_NAME'"

if [[ "$1" == "--clean" ]]; then
    rm -rf ${WORKSPACE_BASE:-/app/workspace}/jasterix/build_$OS_NAME
fi

mkdir -p ${WORKSPACE_BASE:-/app/workspace}/jasterix/build_$OS_NAME
cd ${WORKSPACE_BASE:-/app/workspace}/jasterix/build_$OS_NAME
cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release ..
make -j $(nproc)
sudo make install

cd ${WORKSPACE_BASE:-/app/workspace}/compass/docker
