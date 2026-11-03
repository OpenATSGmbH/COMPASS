#!/usr/bin/env python3
"""
Finds the DLLs in the top folder of a Windows deploy folder that nothing imports: no
executable, no plugin (DLLs in subfolders, loaded at runtime) and no DLL reached from
them (dependency closure of the import tables, read with objdump).

Usage: win_unused_dlls.py <deploy_dir> [--remove]
Without --remove only the list is printed.
"""
import os
import subprocess
import sys

OBJDUMP = "x86_64-w64-mingw32-objdump"


def imports(path):
    out = subprocess.run([OBJDUMP, "-p", path], capture_output=True, text=True).stdout
    return [line.split()[-1].lower() for line in out.splitlines() if "DLL Name:" in line]


def main():
    deploy = sys.argv[1]
    remove = "--remove" in sys.argv[2:]

    top = {f.lower(): f for f in os.listdir(deploy) if f.lower().endswith(".dll")}

    # roots: executables and all DLLs in subfolders (plugins)
    roots = [os.path.join(deploy, f) for f in os.listdir(deploy) if f.lower().endswith(".exe")]
    for entry in os.listdir(deploy):
        path = os.path.join(deploy, entry)
        if os.path.isdir(path):
            for root, _, files in os.walk(path):
                roots += [os.path.join(root, f) for f in files if f.lower().endswith(".dll")]

    reached = set()
    todo = list(roots)
    while todo:
        for name in imports(todo.pop()):
            if name in top and name not in reached:
                reached.add(name)
                todo.append(os.path.join(deploy, top[name]))

    unused = sorted(set(top) - reached)
    size = sum(os.path.getsize(os.path.join(deploy, top[n])) for n in unused)

    for name in unused:
        if remove:
            os.remove(os.path.join(deploy, top[name]))
        print(("removed " if remove else "unused ") + top[name])

    print(f"{len(unused)} unused DLLs, {size // (1024 * 1024)} MB")


if __name__ == "__main__":
    main()
