# MMR Kria drive

## Dependencies and Compiling

Before compiling using automated scripts, you have to install dependencies on your machine.
\
To do this, run in the shell:

```bash
sudo apt install ros-dev-tools ament-cmake rclcpp ros-humble-rclcpp ros-humble-ackermann-msgs ros-humble-can-msgs
```

Then, you need to compile `mmr_base` package first, so run:

```bash
./compile_packages.sh mmr_base
```

Now, you can compile the whole system using `compile.sh`.