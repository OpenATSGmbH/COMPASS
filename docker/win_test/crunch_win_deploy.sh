#!/bin/bash
#
# Deploys the Windows build of COMPASS, the Python test framework and test datasets to the
# Windows test PC crunch_win (ssh alias, user HomeZ, PowerShell as SSH shell).
#
# Layout on crunch_win:
#   C:\compass\COMPASS_<version>_win64   package (from build_win/COMPASS_<version>_win64.zip)
#   C:\compass\py                        experimental_src/py (test framework and tests)
#   C:\data\test\<dataset>               test datasets, without temp/ and results/
#   C:\Users\HomeZ\.compass              test config (conf/test, data_contexts, license)
#   C:\tmp                               for tests that write to /tmp (resolves to C:\tmp)
#   C:\compass\gdb                       gdb from WinLibs (stacks on quit timeout and crash)
#   C:\compass\crash                     crash stacks, written by crunch_win_crash_gdb.bat
#
# Usage: crunch_win_deploy.sh [package] [debug] [py] [config] [data <dataset> ...] [gdb]
#   no arguments: package debug py config data at_20230422
#   debug: compass_client.exe.debug and compass_*.dll.debug (debug information, not in the zip)
#          into the package folder
#   gdb:   one time - gdb with its DLLs and Python library, the crash script, the AeDebug
#          registry key that starts the crash script on a crash, and Windows Error Reporting
#          enabled without dialog (AeDebug needs it) - needs admin rights

set -e

HOST=crunch_win
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPASS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
STAGE="$HOME/tmp/crunch_win"
VERSION=$(grep -oP 'CPACK_PACKAGE_VERSION_(MAJOR|MINOR|PATCH) "\K[0-9]+' "$COMPASS_DIR/CMakeLists.txt" | paste -sd.)
NAME="COMPASS_${VERSION}_win64"

mkdir -p "$STAGE"

#runs a PowerShell command on the test PC
remote() {
    ssh -o BatchMode=yes "$HOST" "$1" 2>&1 | grep -v '^\*\*' || true
}

#copies a local tar file to C:\compass and extracts it into the given Windows folder
put_tar() {
    local tar_file="$1" dest="$2"
    remote "New-Item -ItemType Directory -Force -Path '$dest' | Out-Null"
    scp -q -o BatchMode=yes "$tar_file" "$HOST:C:/compass/$(basename "$tar_file")"
    remote "tar -xf 'C:\\compass\\$(basename "$tar_file")' -C '$dest'; Remove-Item 'C:\\compass\\$(basename "$tar_file")'"
}

STEPS=("$@")
if [ ${#STEPS[@]} -eq 0 ]; then STEPS=(package debug py config data at_20230422); fi

#gdb from the WinLibs MinGW-w64 toolchain (UCRT, same GCC family as the cross build)
WINLIBS_URL="https://github.com/brechtsanders/winlibs_mingw/releases/download/16.2.0posix-23.1.3-14.0.0-ucrt-r3/winlibs-x86_64-posix-seh-gcc-16.2.0-mingw-w64ucrt-14.0.0-r3.zip"
GDB_DLLS="libexpat-1.dll libgcc_s_seh-1.dll libgmp-10.dll libiconv-2.dll liblzma-5.dll libmman.dll
          libmpfr-6.dll libncursesw6.dll libpython3.9.dll libsource-highlight-4.dll libstdc++-6.dll
          libwinpthread-1.dll libzstd.dll xxhash.dll zlib1.dll"

remote "New-Item -ItemType Directory -Force -Path C:\\compass,C:\\data\\test,C:\\tmp | Out-Null"

i=0
while [ $i -lt ${#STEPS[@]} ]; do
    step="${STEPS[$i]}"
    case "$step" in
        package)
            echo "package $NAME"
            remote "if (Test-Path 'C:\\compass\\$NAME') { Remove-Item -Recurse -Force 'C:\\compass\\$NAME' }"
            scp -q -o BatchMode=yes "$COMPASS_DIR/build_win/$NAME.zip" "$HOST:C:/compass/$NAME.zip"
            remote "tar -xf 'C:\\compass\\$NAME.zip' -C 'C:\\compass'; Remove-Item 'C:\\compass\\$NAME.zip'"
            ;;
        debug)
            echo "debug information $NAME"
            # compass_client.exe.debug and compass_*.dll.debug (one DLL per module)
            scp -q -o BatchMode=yes "$COMPASS_DIR"/build_win/compass_deploy/*.debug "$HOST:C:/compass/$NAME/"
            ;;
        gdb)
            echo "gdb"
            if [ ! -f "$STAGE/winlibs.zip" ]; then
                curl -sSL -o "$STAGE/winlibs.zip" "$WINLIBS_URL"
            fi
            rm -rf "$STAGE/winlibs" "$STAGE/gdb"
            mkdir -p "$STAGE/winlibs" "$STAGE/gdb/bin" "$STAGE/gdb/lib"
            unzip -q "$STAGE/winlibs.zip" 'mingw64/bin/*' 'mingw64/share/gdb/*' 'mingw64/lib/python3.9/*' \
                -d "$STAGE/winlibs"
            cp "$STAGE/winlibs/mingw64/bin/gdb.exe" "$STAGE/gdb/bin/"
            for dll in $GDB_DLLS; do cp "$STAGE/winlibs/mingw64/bin/$dll" "$STAGE/gdb/bin/"; done
            cp -r "$STAGE/winlibs/mingw64/share" "$STAGE/gdb/"
            cp -r "$STAGE/winlibs/mingw64/lib/python3.9" "$STAGE/gdb/lib/"
            cp "$SCRIPT_DIR/crunch_win_crash_gdb.bat" "$STAGE/gdb/"
            docker run --rm --user "$(id -u):$(id -g)" -v "$SCRIPT_DIR:/w:ro" -v "$STAGE/gdb:/o" compass/build_win_deb12 \
                x86_64-w64-mingw32-gcc -O2 -static -o /o/crunch_win_set_event.exe /w/crunch_win_set_event.c
            tar -cf "$STAGE/gdb.tar" -C "$STAGE/gdb" .
            rm -rf "$STAGE/winlibs" "$STAGE/gdb"
            remote "if (Test-Path 'C:\\compass\\gdb') { Remove-Item -Recurse -Force 'C:\\compass\\gdb' }"
            put_tar "$STAGE/gdb.tar" 'C:\compass\gdb'
            remote "\$k = 'HKLM:\\SOFTWARE\\Microsoft\\Windows NT\\CurrentVersion\\AeDebug'; Set-ItemProperty -Path \$k -Name Debugger -Value 'cmd.exe /c C:\\compass\\gdb\\crunch_win_crash_gdb.bat %ld %ld'; Set-ItemProperty -Path \$k -Name Auto -Value '1'"
            remote "\$k = 'HKLM:\\SOFTWARE\\Microsoft\\Windows\\Windows Error Reporting'; Set-ItemProperty -Path \$k -Name Disabled -Value 0 -Type DWord; Set-ItemProperty -Path \$k -Name DontShowUI -Value 1 -Type DWord"
            remote "& 'C:\\compass\\gdb\\bin\\gdb.exe' -batch -nx -ex 'show version' | Select-Object -First 1"
            ;;
        py)
            echo "test framework"
            tar -cf "$STAGE/py.tar" -C "$COMPASS_DIR/experimental_src/py" --exclude=__pycache__ .
            remote "if (Test-Path 'C:\\compass\\py') { Remove-Item -Recurse -Force 'C:\\compass\\py' }"
            put_tar "$STAGE/py.tar" 'C:\compass\py'
            ;;
        config)
            echo "test config $VERSION"
            tar -cf "$STAGE/config.tar" -C "$HOME/.compass" "$VERSION/conf/test" data_contexts license
            put_tar "$STAGE/config.tar" 'C:\Users\HomeZ\.compass'
            ;;
        data)
            i=$((i + 1))
            dataset="${STEPS[$i]}"
            echo "dataset $dataset"
            tar -cf "$STAGE/data.tar" -C "$HOME/data/test" --exclude="$dataset/temp" --exclude="$dataset/results" "$dataset"
            put_tar "$STAGE/data.tar" 'C:\data\test'
            ;;
        *)
            echo "unknown step $step"; exit 1 ;;
    esac
    i=$((i + 1))
done

rm -f "$STAGE"/*.tar
echo "done"
