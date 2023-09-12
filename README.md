# to launch on Ignition gazebo - fortress

 - to launch a world
 ```bash
ros2 launch IGVC_test launch_sim.launch.py world:=./src/IGVC_test/worlds/obstacles.world

```

 - to send keyboard input
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

 - to start image viewer
 ```bash
rqt
```

