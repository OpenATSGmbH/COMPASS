# Docker Build Scripts

Dockerfiles and scripts for building COMPASS AppImages inside Debian containers.

## Dockerfiles

- **Dockerfile_deb10** - Debian 10 (Buster) build environment (primary)
- **Dockerfile_deb9** - Debian 9 (Stretch) build environment (deprecated)

Each Dockerfile bakes in an `OS_NAME` env var (`deb10` / `deb9`) used by the build scripts.

## Host-side scripts (run on your machine)

| Script | Description |
|---|---|
| `build_docker.sh <os>` | Build the Docker image, e.g. `./build_docker.sh deb10` |
| `run_docker.sh <os>` | Start an interactive shell inside the container (requires X11) |
| `run_make_appimage.sh [os] [--clean]` | Non-interactive AppImage build. Defaults to `deb10`. Pass `--clean` for a clean rebuild. Works over SSH without a TTY. |

## In-container scripts (run inside the Docker container)

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

## Usage

### First-time setup

```bash
./build_docker.sh deb10
```

### Interactive build (local, with X11)

```bash
./run_docker.sh deb10
# inside container:
./make_appimage.sh
```

### Non-interactive build (remote / SSH)

```bash
./run_make_appimage.sh              # incremental deb10 build
./run_make_appimage.sh deb10 --clean  # clean deb10 build
./run_make_appimage.sh deb9         # incremental deb9 build
```

The resulting AppImage is written to the repository root as `COMPASS_<os>-x86_64.AppImage`.
