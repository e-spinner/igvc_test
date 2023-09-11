# to launch on Ignition gazebo - fortress

 - to launch a world
 ```bash
    ign gazebo /home/dev/test_ws/src/IGVC_test/worlds/basic.sdf
```

 - to load in robot.urdf
 ```bash
 ign service -s /world/basic/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: "/home/dev/test_ws/src/IGVC_test/description/robot.urdf", name: "sarah"'
```

 - to send basic movements
 ```bash
 ign topic -t "/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.0}" 
 ```

 - to send keyboard input
    - start Key Publisher ign-gui in gazebo
```bash
ign topic -e -t /keyboard/keypress
```

