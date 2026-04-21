#!/bin/bash

# exit when any command fails
set -e

echo "os: '$OS_NAME'"

CLEAN=0
ASAN=0
for arg in "$@"; do
    case "$arg" in
        --clean) CLEAN=1 ;;
        --asan)  ASAN=1 ;;
    esac
done

if [[ $CLEAN -eq 1 ]]; then
    rm -rf ${WORKSPACE_BASE:-/app/workspace}/jasterix/build_$OS_NAME
fi

CMAKE_EXTRA=()
if [[ $ASAN -eq 1 ]]; then
    # AddressSanitizer build — must match compass flags so shared symbols line up.
    echo "AddressSanitizer build enabled"
    CMAKE_EXTRA+=(-DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer -O1 -g")
    CMAKE_EXTRA+=(-DCMAKE_C_FLAGS="-fsanitize=address -fno-omit-frame-pointer -O1 -g")
    CMAKE_EXTRA+=(-DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address")
    CMAKE_EXTRA+=(-DCMAKE_SHARED_LINKER_FLAGS="-fsanitize=address")
fi

mkdir -p ${WORKSPACE_BASE:-/app/workspace}/jasterix/build_$OS_NAME
cd ${WORKSPACE_BASE:-/app/workspace}/jasterix/build_$OS_NAME
cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release "${CMAKE_EXTRA[@]}" ..
make -j $(nproc)
sudo make install

cd ${WORKSPACE_BASE:-/app/workspace}/compass/docker
