#!/bin/bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
catkin_make_cmd=(
    catkin_make
    -DPYTHON_EXECUTABLE=/usr/bin/python3
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
)

cd "${workspace}/master1"
"${catkin_make_cmd[@]}"

cd ../master2
"${catkin_make_cmd[@]}"

cd ../follow1
"${catkin_make_cmd[@]}"

cd ../follow2
"${catkin_make_cmd[@]}"

cd ..
