#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-$HOME/DroneSortingProject}"
ROS_WS="${ROS_WS:-$HOME/drone_sorting_ros1_ws}"

SOURCE_PACKAGE="$PROJECT_ROOT/ROS1/src/drone_sorting"
TARGET_PACKAGE="$ROS_WS/src/drone_sorting"

if [[ ! -d "$SOURCE_PACKAGE" ]]; then
    echo "[FAIL] ROS package not found:"
    echo "       $SOURCE_PACKAGE"
    exit 1
fi

mkdir -p "$ROS_WS/src"

rm -rf "$TARGET_PACKAGE"
cp -a "$SOURCE_PACKAGE" "$TARGET_PACKAGE"

echo "[OK] Synced ROS package to:"
echo "     $TARGET_PACKAGE"
