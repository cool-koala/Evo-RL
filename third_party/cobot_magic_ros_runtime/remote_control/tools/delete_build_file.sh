#!/bin/bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

for arm_workspace in follow1 follow2 master1 master2; do
    rm -rf \
        "${workspace}/${arm_workspace}/build" \
        "${workspace}/${arm_workspace}/devel" \
        "${workspace}/${arm_workspace}/install" \
        "${workspace}/${arm_workspace}/log" \
        "${workspace}/${arm_workspace}/.catkin_workspace" \
        "${workspace}/${arm_workspace}/src/CMakeLists.txt"
done
