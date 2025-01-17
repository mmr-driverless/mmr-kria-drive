# MMR Kria drive

## Dependencies and Compiling

Before compiling using automated scripts, you have to install dependencies on your machine.
\
To do this, run in the shell:

```bash
sudo apt install ros-dev-tools ament-cmake ros-humble-rclcpp ros-humble-ackermann-msgs ros-humble-can-msgs
```

Then, you need to compile `mmr_base` and `mmr_edf` packages first, so run:

```bash
./compile_packages.sh mmr_base mmr_edf
source install/setup.bash
```

Now, you can compile the whole system using `compile.sh`.