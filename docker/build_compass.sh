#!/bin/bash

# exit when any command fails
set -e

echo "os: '$OS_NAME'"

export QT_SELECT=5

CLEAN=0
ASAN=0
for arg in "$@"; do
    case "$arg" in
        --clean) CLEAN=1 ;;
        --asan)  ASAN=1 ;;
    esac
done

if [[ $CLEAN -eq 1 ]]; then
    rm -rf ${WORKSPACE_BASE:-/app/workspace}/compass/build_$OS_NAME
fi

CMAKE_EXTRA=()
if [[ $ASAN -eq 1 ]]; then
    # AddressSanitizer build — diagnostics for heap/memory corruption.
    # -O1 keeps optimization low enough for readable stack traces without
    # making the build painfully slow. -fno-omit-frame-pointer ensures ASan
    # can walk the stack reliably.
    echo "AddressSanitizer build enabled"
    CMAKE_EXTRA+=(-DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer -O1 -g")
    CMAKE_EXTRA+=(-DCMAKE_C_FLAGS="-fsanitize=address -fno-omit-frame-pointer -O1 -g")
    CMAKE_EXTRA+=(-DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address")
    CMAKE_EXTRA+=(-DCMAKE_SHARED_LINKER_FLAGS="-fsanitize=address")
fi

mkdir -p ${WORKSPACE_BASE:-/app/workspace}/compass/build_$OS_NAME
cd ${WORKSPACE_BASE:-/app/workspace}/compass/build_$OS_NAME
cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=RelWithDebInfo "${CMAKE_EXTRA[@]}" .. # -DCMAKE_PREFIX_PATH=$GL_PATH $CMAKE_OPTS
make -j $(nproc)
sudo make install

cd ${WORKSPACE_BASE:-/app/workspace}/compass/docker
