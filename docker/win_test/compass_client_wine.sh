#!/bin/bash
#
# Starts the Windows build of the COMPASS client in Wine, for the Python test framework
# (--binary=<this script>). All arguments are passed on. Linux paths in the arguments work
# unchanged: Wine resolves them on drive Z: (/).
#
# Environment:
#   COMPASS_WIN_DEPLOY  deploy folder of win_deploy.sh (default: build_win_deb12/compass_deploy)
#   WINEPREFIX          Wine prefix with the test config in C:\users\<user>\.compass

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY="${COMPASS_WIN_DEPLOY:-$SCRIPT_DIR/../../build_win_deb12/compass_deploy}"

export WINEPREFIX="${WINEPREFIX:-$HOME/tmp/wine_compass_test}"
export WINEDEBUG="${WINEDEBUG:--all}"

# Wine does not pass QT_* variables to Windows processes, it takes them from WINEQT_* (as PATH
# from WINEPATH). QT_SCALE_FACTOR: display scaling test. Not QT_QPA_PLATFORM (offscreen without
# --gui): the Windows offscreen platform has no OpenGL for the Geographic View.
if [ -n "${QT_SCALE_FACTOR+x}" ]; then
    export WINEQT_SCALE_FACTOR="$QT_SCALE_FACTOR"
fi

exec wine "$DEPLOY/compass_client.exe" "$@"
