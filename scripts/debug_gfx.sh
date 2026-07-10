#!/usr/bin/env bash
#
# debug_gfx.sh - launch COMPASS binary with GL/shader debug logging.
#
# Usage:
#   ./debug_gfx.sh [-o LOGFILE] /path/to/binary [args passed through to the binary...]
#   LOGFILE defaults to debug_gfx_log.txt.
#
# Examples:
#   ./debug_gfx.sh ./path/to/binary/COMPASS-v1.0.0_hf1_deb10.AppImage
#   ./debug_gfx.sh ./path/to/binary/COMPASS-v1.0.0_hf1_deb10.AppImage --some-flag value
#   ./debug_gfx.sh -o my_run.txt ./path/to/binary/COMPASS-v1.0.0_hf1_deb10.AppImage
#
set -euo pipefail
 
LOG_FILE="debug_gfx_log.txt"

# Parse our own options first; they must precede the binary path so they don't
# collide with arguments passed through to the application.
while [[ $# -gt 0 ]]; do
    case "$1" in
        -o|--log)
            if [[ $# -lt 2 ]]; then
                echo "Error: $1 requires a filename argument." >&2
                exit 2
            fi
            LOG_FILE="$2"
            shift 2
            ;;
        --)
            shift
            break
            ;;
        *)
            break
            ;;
    esac
done

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 [-o LOGFILE] /path/to/binary [app args...]" >&2
    exit 2
fi
 
APP="$1"
shift   # remaining "$@" are the application's own arguments
 
if [[ ! -x "$APP" ]]; then
    echo "Error: '$APP' is not an executable file." >&2
    exit 2
fi
 
# --- OSG / osgEarth diagnostic environment (scoped to this launch only) ---
export OSG_NOTIFY_LEVEL=INFO        # INFO shows the capabilities/driver block
export OSGEARTH_NOTIFY_LEVEL=INFO
export OSG_GL_ERROR_CHECKING=ON     # pin GL errors to the offending attribute
export OSGEARTH_GL_DEBUG=1          # verbose GL error detail
export OSGEARTH_VP_DEBUG=1          # VirtualProgram (sprite shader) errors
export OSGEARTH_DUMP_SHADERS=1      # composed GLSL source (verbose)
# export OSGEARTH_MEMORY_PROFILE=1  # uncomment to gauge memory use over time
 
echo "Launching : $APP $*"
echo "Logging to: $LOG_FILE"
 
exec stdbuf -oL -eL "$APP" "$@" 2>&1 | tee "$LOG_FILE"
