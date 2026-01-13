# Minimum turning radius issue 

1. `ros2 launch planner_playground test_planner.launch.py` (rememeber to change the minimum_turning_radius and costmap resolution in the config directory)
2. `python3 check_dual_path.py` (remember to change the minimum turning radius in the script)
3. Send two goal poses, in my example:

```
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: map
pose:
  position:
    x: 53.808990478515625
    y: 18.24526023864746
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.013367359504648285
    w: 0.9999106528584809
"

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: map
pose:
  position:
    x: 53.04671096801758
    y: 6.655716419219971
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.02546953447283049
    w: 0.9996755987887957
"
```

You should be able to visualize the poses that violates the minimum turning radius. If you want to visualize only unsmoothed plan / smoothed plan, you can comment the other one in the `check_dual_path.py` script.


# planner_playground
Package to let you standalone try planner configurations and visualize paths interactively.

To launch: `ros2 launch planner_playground test_planner.launch.py`

Then, to for a path request, send 2 successive `2D Goal Pose`:

https://github.com/botsandus/planner_playground/assets/15727892/d0cf5293-9403-4ade-adeb-a1affcc47483

You can change the map and the parameters in https://github.com/botsandus/planner_playground/tree/main/config
