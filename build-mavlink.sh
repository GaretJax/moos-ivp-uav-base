#!/usr/bin/env bash

# Build MAVSDK (including its MAVLink dependency) and install it into a
# project-local prefix. Nothing is installed system-wide.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MAVSDK_SOURCE_DIR="${SCRIPT_DIR}/MAVSDK"
MAVSDK_BUILD_DIR="${MAVSDK_SOURCE_DIR}/build/default"
MAVSDK_INSTALL_PREFIX="${MAVSDK_BUILD_DIR}/install"
MAVSDK_BUILD_TYPE="${MAVSDK_BUILD_TYPE:-Debug}"

if [ ! -f "${MAVSDK_SOURCE_DIR}/CMakeLists.txt" ]; then
    echo "Initializing the MAVSDK submodule..."
    git -C "${SCRIPT_DIR}" submodule update --init --recursive MAVSDK
fi

if command -v nproc >/dev/null 2>&1; then
    DEFAULT_BUILD_JOBS="$(nproc)"
else
    DEFAULT_BUILD_JOBS=4
fi
BUILD_JOBS="${CMAKE_BUILD_PARALLEL_LEVEL:-${DEFAULT_BUILD_JOBS}}"

echo "Configuring MAVSDK (${MAVSDK_BUILD_TYPE})..."
echo "Local install prefix: ${MAVSDK_INSTALL_PREFIX}"

cmake -S "${MAVSDK_SOURCE_DIR}" -B "${MAVSDK_BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE="${MAVSDK_BUILD_TYPE}" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_INSTALL_PREFIX="${MAVSDK_INSTALL_PREFIX}" \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_TESTS=OFF \
    -DSUPERBUILD=ON

cmake --build "${MAVSDK_BUILD_DIR}" --parallel "${BUILD_JOBS}"
cmake --install "${MAVSDK_BUILD_DIR}"

MAVSDK_CONFIG_FILE="$(find "${MAVSDK_INSTALL_PREFIX}" \
    -type f -path '*/cmake/MAVSDK/MAVSDKConfig.cmake' -print -quit)"

if [ -z "${MAVSDK_CONFIG_FILE}" ]; then
    echo "Error: MAVSDKConfig.cmake was not installed under ${MAVSDK_INSTALL_PREFIX}." >&2
    exit 1
fi

echo "MAVSDK installed locally."
echo "CMake package: $(dirname "${MAVSDK_CONFIG_FILE}")"
