# to launch on Gazebo-classic v11

 - to launch a world
 ```bash
ros2 launch igvc_test launch_sim.launch.py world:=./src/igvc_test/worlds/obstacles.world

```

 - to send keyboard input
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

 - to start image viewer
 ```bash
rqt
```

