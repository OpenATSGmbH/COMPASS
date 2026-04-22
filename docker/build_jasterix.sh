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

BUILD_DIR=${WORKSPACE_BASE:-/app/workspace}/jasterix/build_$OS_NAME
ASAN_MARKER=$BUILD_DIR/.asan_state

# Auto-force clean when the ASan flag toggles (see comment in build_compass.sh).
if [[ -d "$BUILD_DIR" ]]; then
    PREV_ASAN=$(cat "$ASAN_MARKER" 2>/dev/null || echo "")
    if [[ "$PREV_ASAN" != "$ASAN" ]]; then
        echo "ASAN state changed (prev='$PREV_ASAN' now='$ASAN') — forcing clean build"
        CLEAN=1
    fi
fi

if [[ $CLEAN -eq 1 ]]; then
    rm -rf "$BUILD_DIR"
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

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
echo "$ASAN" > "$ASAN_MARKER"
cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release "${CMAKE_EXTRA[@]}" ..
make -j $(nproc)
sudo make install

cd ${WORKSPACE_BASE:-/app/workspace}/compass/docker
