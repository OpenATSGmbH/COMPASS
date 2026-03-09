#!/bin/bash

# exit when any command fails
set -e

echo "os: '$OS_NAME'"

./build_jasterix.sh
./build_compass.sh
./deploy_compass.sh
