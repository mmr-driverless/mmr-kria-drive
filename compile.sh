#!/bin/bash

./compile_packages.sh mmr_base mmr_edf
source install/setup.bash

colcon build --continue-on-error --symlink-install
