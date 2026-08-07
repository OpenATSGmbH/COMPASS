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

BUILD_DIR=${WORKSPACE_BASE:-/app/workspace}/compass/build_$OS_NAME
ASAN_MARKER=$BUILD_DIR/.asan_state

# Auto-force clean when the ASan flag toggles vs the previous build in this
# directory. Mixing ASan-instrumented and non-instrumented object files produces
# link errors or silently broken binaries because make/ninja rebuild rules use
# timestamps, not compile-flag hashes.
if [[ -d "$BUILD_DIR" ]]; then
    PREV_ASAN=$(cat "$ASAN_MARKER" 2>/dev/null || echo "")
    if [[ "$PREV_ASAN" != "$ASAN" ]]; then
        echo "ASAN state changed (prev='$PREV_ASAN' now='$ASAN') - forcing clean build"
        CLEAN=1
    fi
fi

if [[ $CLEAN -eq 1 ]]; then
    rm -rf "$BUILD_DIR"
fi

CMAKE_EXTRA=()
if [[ $ASAN -eq 1 ]]; then
    # AddressSanitizer build - diagnostics for heap/memory corruption.
    # -O1 keeps optimization low enough for readable stack traces without
    # making the build painfully slow. -fno-omit-frame-pointer ensures ASan
    # can walk the stack reliably.
    echo "AddressSanitizer build enabled"
    CMAKE_EXTRA+=(-DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer -O1 -g")
    CMAKE_EXTRA+=(-DCMAKE_C_FLAGS="-fsanitize=address -fno-omit-frame-pointer -O1 -g")
    CMAKE_EXTRA+=(-DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address")
    CMAKE_EXTRA+=(-DCMAKE_SHARED_LINKER_FLAGS="-fsanitize=address")
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
echo "$ASAN" > "$ASAN_MARKER"
cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=RelWithDebInfo "${CMAKE_EXTRA[@]}" .. # -DCMAKE_PREFIX_PATH=$GL_PATH $CMAKE_OPTS
make -j $(nproc)
# `make install` depends on the `all` target, so it would re-run the compiler as
# root and leave root-owned object files in the shared build dir, which the next
# non-root build cannot overwrite. `cmake --install` only installs.
sudo cmake --install .

cd ${WORKSPACE_BASE:-/app/workspace}/compass/docker
