# to launch 2D LiDAR sim
 ```bash
ros2 launch igvc_test launch_test_test_sim.launch.py world:=./src/igvc_test/worlds/maze.world

```
 
 - to launch nav2
 ```bash
ros2 launch igvc_test navigation.launch.py use_sim_time:=true
```

 - to send keyboard input
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy
```


 # to launch 3D LiDAR sim


- to start ball tracker
```bash
ros2 launch igvc_test ball_tracker.launch.py sim_monde:=true
```

