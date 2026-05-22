#!/bin/bash

# exit when any command fails
set -e

echo "os: '$OS_NAME'"

CLEAN=""
if [[ "$1" == "--clean" ]]; then
    CLEAN="--clean"
fi

./build_jasterix.sh $CLEAN
./build_compass.sh $CLEAN
./deploy_compass.sh
