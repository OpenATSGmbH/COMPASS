# Docker Build Scripts

Dockerfiles and scripts for building the COMPASS AppImage inside an AlmaLinux container, and the Windows version
(cross build with MinGW-w64) inside a Debian 12 container.

## Dockerfiles

- **Dockerfile_alma8** - AlmaLinux 8 build environment for the AppImage (glibc 2.28 as Debian 10, Qt 5.15, gcc-toolset-13)
- **Dockerfile_win_deb12** - Windows build environment on Debian 12: own MinGW-w64 compiler (GCC 14.2), Qt 5.15.3
  and all other Windows libraries built from source. Wine 10 (WineHQ), Xvfb and Python with pandas for the Windows
  tests of the CI (`compass_tests.exe` and the integration tests in Wine, see `experimental_src/py/readme_ci.md`)

Each Dockerfile bakes in an `OS_NAME` env var (`alma8` / `win_deb12`) used by the build scripts.

## Host-side scripts (run on your machine)

| Script | Description |
|---|---|
| `build_docker.sh <os>` | Build the Docker image, e.g. `./build_docker.sh alma8` |
| `run_docker.sh <os>` | Start an interactive shell inside the container (requires X11) |
| `run_make_appimage.sh [os] [--clean]` | Non-interactive AppImage build. Defaults to `alma8`. Pass `--clean` for a clean rebuild. Works over SSH without a TTY. |
| `run_make_windows_build.sh [--clean]` | Non-interactive Windows build. Pass `--clean` for a clean rebuild. Works over SSH without a TTY. |

## In-container scripts (run inside the Docker container)

AppImage (`alma8` container):

| Script | Description |
|---|---|
| `make_appimage.sh [--clean]` | Orchestrates the full build: jASTERIX, COMPASS, then AppImage packaging. Pass `--clean` to delete build dirs first. |
| `build_jasterix.sh [--clean]` | Build and install jASTERIX. Incremental by default, `--clean` removes build dir first. |
| `build_compass.sh [--clean]` | Build and install COMPASS. Incremental by default, `--clean` removes build dir first. |
| `deploy_compass.sh` | Package COMPASS into an AppImage using linuxdeploy |
| `deploy_jasterix.sh` | Package jASTERIX into an AppImage using linuxdeploy |
| `build_toe.sh` | Build TOE (unrelated helper project) |
| `deploy_toe.sh` | Package TOE into an AppImage |
| `run_bench.sh` | Run SQLite benchmark |

Windows (`win_deb12` container):

| Script | Description |
|---|---|
| `make_windows_build.sh [--clean]` | Orchestrates the full build: jASTERIX, COMPASS, then the ZIP file. Pass `--clean` to delete build dirs first. |
| `win_build.sh [jasterix\|compass\|all] [--clean]` | Build jASTERIX and/or COMPASS (default `all`). Incremental by default, `--clean` removes the build dirs of the step first. |
| `win_deploy.sh [--with_tests] [--zip]` | Collect the build with all runtime files into `compass_deploy`. `--zip` also packs `COMPASS_<version>_win64.zip`, `--with_tests` adds `compass_tests.exe` (not for the ZIP file). |
| `win_unused_dlls.py <deploy_dir> [--remove]` | List (or remove) the DLLs nothing imports. Called by `win_deploy.sh`. |

Other Windows files: `mingw_toolchain.cmake` (CMake toolchain file of the cross build) and `qt5_host_tools.cmake`
(Linux moc/uic/rcc for the cross build) are copied into the image. `win_test/` has test programs for the Windows
libraries, `compass_client_wine.sh` (starts the Windows build in Wine) and the scripts for the Windows test PC
(`potato_win_*`).

## Usage

The scripts expect the COMPASS and jASTERIX repositories in `~/workspace/compass` and `~/workspace/jasterix`
(`run_docker.sh` mounts the home folder as `/app`). Run all host-side scripts in `~/workspace/compass/docker`.

### First-time setup

```bash
./build_docker.sh alma8       # AppImage
./build_docker.sh win_deb12   # Windows (builds the compiler and Qt from source, takes hours)
```

Build the image again after its Dockerfile changed.

### Interactive build (local, with X11)

```bash
./run_docker.sh alma8
# inside container:
./make_appimage.sh
```

```bash
./run_docker.sh win_deb12
# inside container:
./make_windows_build.sh
```

### Non-interactive build (remote / SSH)

```bash
./run_make_appimage.sh                # incremental alma8 build
./run_make_appimage.sh alma8 --clean  # clean alma8 build
./run_make_windows_build.sh           # incremental Windows build
./run_make_windows_build.sh --clean   # clean Windows build
```

Each OS has its own build directories (`build_<os>` in compass, for the AppImage also in jasterix), so the builds
do not interfere.

### Results

The AppImage is written to the repository root as `COMPASS_<os>-x86_64.AppImage`.

The Windows build is written to `build_win_deb12` in the repository:

- `COMPASS_<version>_win64.zip` - the file to distribute (version from `CPACK_PACKAGE_VERSION_*` in
  `CMakeLists.txt`). It is not an installer: on Windows 10 or later, extract it anywhere and start
  `COMPASS_<version>_win64\compass_client.exe`.
- `compass_deploy/` - the same content unpacked, plus the debug information (`*.debug`, not in the ZIP file).
  Keep a copy for each released version: with it, gdb shows readable stacks of crashes (the `.debug` files must
  lie next to the executables and DLLs).
