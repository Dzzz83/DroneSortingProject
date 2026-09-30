#!/usr/bin/env bash

set -e

"$HOME/Unity/Hub/Editor/6000.6.0f1/Editor/Unity" \
    -projectPath "$HOME/DroneSortingSimulation" \
    -batchmode \
    -quit \
    -logFile "$HOME/DroneSortingSimulation/unity-recompile.log"

echo "Unity recompilation complete."