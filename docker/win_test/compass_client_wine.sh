#!/bin/bash
#
# Starts the Windows build of the COMPASS client in Wine, for the Python test framework
# (--binary=<this script>). All arguments are passed on. Linux paths in the arguments work
# unchanged: Wine resolves them on drive Z: (/).
#
# Environment:
#   COMPASS_WIN_DEPLOY  deploy folder of win_deploy.sh (default: build_win/compass_deploy)
#   WINEPREFIX          Wine prefix with the test config in C:\users\<user>\.compass

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY="${COMPASS_WIN_DEPLOY:-$SCRIPT_DIR/../../build_win/compass_deploy}"

export WINEPREFIX="${WINEPREFIX:-$HOME/tmp/wine_compass_test}"
export WINEDEBUG="${WINEDEBUG:--all}"

exec wine "$DEPLOY/compass_client.exe" "$@"
