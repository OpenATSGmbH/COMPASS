#!/bin/bash
#
# Checks the libraries bundled in the AppImage, run at the end of make_appimage.sh. Fails if
# - an executable, host library (libcompass_*.so) or plugin module (compass_plugins/*.so) installed by the build
#   is missing in the AppImage
# - a plugin module's RUNPATH does not contain $ORIGIN/..
# - a dependency of an executable, host library or plugin module is not found, or a direct dependency is found
#   outside the AppImage although the AppImage bundles a library of that name (broken RUNPATH; transitive
#   dependencies of system libraries such as libGL are resolved via their own search path and not checked)
# - a static COMPASS archive (libcompass_*.a) is bundled
#
# Usage: check_appimage.sh <appimage> <build dir with install_manifest.txt>

set -e

APPIMAGE=$(realpath "$1")
MANIFEST=$(realpath "$2")/install_manifest.txt

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

cd "$WORK"
"$APPIMAGE" --appimage-extract > /dev/null
ROOT=$WORK/squashfs-root

ERRORS=0
fail()
{
    echo "check_appimage: $1"
    ERRORS=$((ERRORS + 1))
}

# installed executables, host libraries and plugin modules => must be in the AppImage at the same path
EXPECTED=$(grep -E '^/usr/(bin/compass_[a-z]+|lib/libcompass_[a-z_]+\.so[.0-9]*|lib/compass_plugins/compass_plugin_[a-z]+\.so)$' \
                "$MANIFEST")
NUM_PLUGINS=0
for file in $EXPECTED; do
    if [[ ! -f "$ROOT$file" ]]; then
        fail "missing $file"
    fi
    if [[ "$file" == /usr/lib/compass_plugins/* ]]; then
        NUM_PLUGINS=$((NUM_PLUGINS + 1))
    fi
done

# plugin modules find the host libraries in usr/lib/
for plugin in "$ROOT"/usr/lib/compass_plugins/*.so; do
    [[ -e "$plugin" ]] || continue # build without experimental source
    runpath=$(readelf -d "$plugin" | sed -n 's/.*(RUNPATH).*\[\(.*\)\]/\1/p')
    if [[ ":$runpath:" != *':$ORIGIN/..:'* ]]; then
        fail "${plugin#$ROOT} has RUNPATH '$runpath' without '\$ORIGIN/..'"
    fi
done

# dependencies: found, and found inside the AppImage if it bundles them
for file in $EXPECTED; do
    [[ -f "$ROOT$file" ]] || continue
    needed=" $(readelf -d "$ROOT$file" | sed -n 's/.*(NEEDED).*\[\(.*\)\]/\1/p' | tr '\n' ' ') "
    while read -r name arrow path rest; do
        [[ "$arrow" == "=>" ]] || continue # loader, vdso
        if [[ "$path" == "not" ]]; then
            fail "$file: $name not found"
        elif [[ "$needed" == *" $name "* && "$path" != "$ROOT"/* && -e "$ROOT/usr/lib/$name" ]]; then
            fail "$file: $name found at $path instead of usr/lib/"
        fi
    done < <(ldd "$ROOT$file")
done

for archive in $(find "$ROOT" -name 'libcompass_*.a'); do
    fail "static archive ${archive#$ROOT} bundled"
done

if [[ $ERRORS -gt 0 ]]; then
    echo "check_appimage: $ERRORS error(s)"
    exit 1
fi

echo "check_appimage: ok ($(echo "$EXPECTED" | wc -w) files checked, $NUM_PLUGINS plugin modules)"
