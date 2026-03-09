#!/bin/bash

# exit when any command fails
set -e

echo "os: '$OS_NAME'"

if [[ "$1" == "--clean" ]]; then
    rm -rf /app/workspace/jasterix/build_$OS_NAME
fi

mkdir -p /app/workspace/jasterix/build_$OS_NAME
cd /app/workspace/jasterix/build_$OS_NAME
cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release ..
make -j $(nproc)
sudo make install

cd /app/workspace/compass/docker
