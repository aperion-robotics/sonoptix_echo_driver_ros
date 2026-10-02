# ROS2 Migration Notes — sonoptix_echo_driver_ros

Kept as close as possible to the ROS1 (`noetic-devel`) package: same layout (`scripts/`,
`config/`, `launch/`), `ament_cmake`, Python launch files and the same node code, with only the
rospy → rclpy mechanics changed.

## Differences from ROS1

- **dynamic_reconfigure → ROS2 parameters:** the `cfg/SonoptixEcho.cfg` entries are declared as
  node parameters with the same names, defaults and ranges (`ros2 param set`). As with dynamic
  reconfigure, `dyn_reconf_callback` runs once at startup and on every change with the full config.
  Out-of-range values are rejected instead of clamped, and double parameters must be set as
  doubles (`sonar_range 15.0`, not `15`).
- **Threads:** rospy ran every timer/service/subscriber in its own thread; here each gets its own
  `MutuallyExclusiveCallbackGroup` on a `MultiThreadedExecutor(num_threads=8)`, so they run in
  parallel but a callback never overlaps itself.
- **Namespace:** the node runs under `$ROBOT_NAME` (default `bluebot`) and all topics are relative
  (`/bluebot/sonar_scan`, `/bluebot/sonar_image`, `/bluebot/sonar_image_cone_projection`,
  `/bluebot/enable_sonar`). Default `sonar_frame` is `bluebot_sonar_base_link`.
- **Debugging mode fix:** since the new-data flags were added (d2b62be), ROS1 debugging mode published
  nothing with the sonar disabled; `debugging_image_callback` now marks every incoming image as new data.
- **Timer names:** in ROS1 `image_pub_timer` ran the laserscan publisher and `laserscan_pub_timer`
  the image publisher; the names are swapped back to match what each timer publishes.
- **Bug fix:** publishing the sonar image with both output flips disabled raised a `NameError`
  in ROS1; it now publishes the unflipped image.
- The empty `src/sonoptix_echo_driver_ros/__init__.py` only existed for the generated dynamic
  reconfigure module and was dropped.

## Not ported

- `scripts/sonar_cone_projector.py` (standalone cone projection) and `scripts/edge_detection.py`.
- The `scrufy_bringup` overrides (`sonoptix_echo.yaml`, `sonar_dynamic_reconfigure.yaml`) belong
  to the bringup package, which is not ported yet.
