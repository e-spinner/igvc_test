# to launch on Gazebo-classic v11

 - to launch a world
 ```bash
ros2 launch igvc_test launch_sim.launch.py world:=./src/igvc_test/worlds/obstacles.world

```

 - to send keyboard input
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

 - to start RViz2
 ```bash
ros2 run rviz2 rviz2 -d src/igvc_test/config/main.rvis --ros-args -p use_sim_time:=true
 ```

 - to start SLAM
 ```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=/src/test_igvc/config/mapper_params_online_async.yaml use_sim_time:=true

 ```

