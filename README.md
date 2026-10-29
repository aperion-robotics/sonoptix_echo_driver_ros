# sonoptix_echo_driver_ros

ROS Driver for the Sonoptix ECHO Multibeam Imaging Sonar

## Parameters

- `ip`: IP of the sonar (default: `"192.168.2.42"`)
- `rtsp_port`: RTSP port of the sonar (default: `8554`)
- `api_port`: HTTP API port of the sonar (default: `8000`)
- `laserscan_topic`: Laserscan output topic (default: `"sonar_scan"`)
- `sonar_image_topic`: Sonar image output topic; the cone projection is published on `<topic>_cone_projection` (default: `"sonar_image"`)
- `http_request_timeout`: Timeout in seconds for sonar HTTP API requests (connect, send, and receive) (default: `2.0`)
- `rtsp_open_timeout`: Timeout in seconds for opening the sonar RTSP stream (default: `5.0`)
- `rtsp_read_timeout`: Timeout in seconds for receiving a frame from the sonar RTSP stream, after which the stream is reconnected (default: `5.0`)
- `rtsp_capture_options`: FFmpeg options for the RTSP stream, in `key;value|key;value` format (`OPENCV_FFMPEG_CAPTURE_OPTIONS`); empty keeps OpenCV’s defaults (default: `"rtsp_transport;tcp|fflags;nobuffer"`)
- `publish_hz`: Maximum publishing rate of the laserscan and images, and frame rate of mock hardware (default: `25`)
- `debugging_sonar_image_sub_topic`: When set, sonar images are read from this topic instead of the sensor (default: `""`)
- `sonar_frame`: Frame the sensor is attached to (default: `"bluebot_sonar_base_link"`)
- `sonar_range`: Range of the sensor (default: `10.0`)
- `sonar_operation_mode`: Sensor operating mode (0: auto, 1: HF, 2: LF, 3: LFLR) (default: `0`)
- `default_distance_is_inf`: Use an Inf laserscan range value when no obstacle is found (default: `true`)
- `default_distance_value`: Laserscan range value when no obstacle is found (default: `999.0`)
- `sonar_min_range`: Minimum range considered valid for the laserscan message (default: `0.5`)
- `image_data_factor`: Factor multiplied by the original sonar image data (default: `1.0`)
- `publish_sonar_image`: Publish the original sensor image as a `sensor_msgs/Image` message (default: `false`)
- `publish_cone_image`: Publish the processed cone projection as a `sensor_msgs/Image` message (default: `false`)
- `publish_laserscan`: Publish the original sensor image as a `sensor_msgs/Laserscan` message (default: `true`)
- `flip_input_x_sonar_image`: Flip the sonar image on the x axis before computation (default: `false`)
- `flip_input_y_sonar_image`: Flip the sonar image on the y axis before computation (default: `false`)
- `flip_output_x_sonar_image`: Publish the original sensor image with a flipped x axis (default: `true`)
- `flip_output_y_sonar_image`: Publish the original sensor image with a flipped y axis (default: `true`)
- `flip_output_x_sonar_image_cone`: Publish the cone projection image with a flipped x axis (default: `true`)
- `flip_output_y_sonar_image_cone`: Publish the cone projection image with a flipped y axis (default: `true`)
- `minimum_obstacle_value`: Minimum sensor image pixel value considered an obstacle (default: `1`)
- `mock_hardware`: Do not connect to real hardware; publish random data instead (default: `false`)
- `draw_distance_lines`: Show range rings with labels at a round step, wedge edges, and bearing lines every 30 degrees in the cone projection (default: `true`)
- `enabled`: Start the sensor driver (default: `false`)

Since the `generate_parameter_library` package is utilized to implement dynamic (and static) parameter handling, make sure to consult the `src/sonoptix_echo_driver_parameters.yaml` for the most up-to-date list of parameters.

## Extras

### Breaking changes (vs Python version of the driver)

- The C++ version of the driver uses dynamic parameters and lifecycles. This deems the use of the `enable_sonar` service useless, thus it has been removed.
- Killing the node now attempts to also disable the sensor.

### Component

The node is also a **component** (`sonoptix_echo_driver_ros::SonoptixEchoDriver`).

(For the uninitiated infidels) Loading a component in the same container as the nodes that consume its topics, with `use_intra_process_comms:=true`, avoids copying and serializing the messages.

As an example, to load the component with `use_intra_process_comms` enabled, you can use:

```bash
ros2 component load /container sonoptix_echo_driver_ros sonoptix_echo_driver_ros::SonoptixEchoDriver --extra-argument use_intra_process_comms:=true
```
