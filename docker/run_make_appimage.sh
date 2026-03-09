#!/bin/bash

# exit when any command fails
set -e

OS=${1:-deb10}
CLEAN=${2:+"$2"}

echo "os: '$OS'"

docker run --rm --user $(id -u):$(id -g) -v $PWD/../../..:/app compass/build_$OS ./make_appimage.sh $CLEAN
