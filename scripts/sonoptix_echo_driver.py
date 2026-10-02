#!/usr/bin/python3
import cv2
import time
import rclpy
import requests
import threading
import numpy as np
from cv_bridge import CvBridge
from types import SimpleNamespace
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
from std_srvs.srv import SetBool
from sensor_msgs.msg import LaserScan, Image

# NOTE: Replaces cfg/SonoptixEcho.cfg (dynamic reconfigure).
# name, default, description, (min, max)
RECONFIGURABLE_PARAMETERS = [
    ("sonar_frame", "bluebot_sonar_base_link", "The frame the sensor is attached to", None),
    ("sonar_range", 10.0, "The range of the sensor", (0.0, 200.0)),
    ("sonar_operation_mode", 0, "The operating mode of the sensor (0:auto,1:HF,2:LF,3:LFLR)", (0, 3)),
    ("default_distance_is_inf", True, "Use Inf laserscan range value when no obstacle is found", None),
    ("default_distance_value", 999.0, "The laserscan range value when no obstacle is found", (0.0, 999.0)),
    ("sonar_min_range", 0.5, "The minimum range to be considered as valid for the laserscan message", (0.0, 200.0)),
    ("image_data_factor", 1.0, "A factor that is multiplied on the original sonar image data", (0.0, 200.0)),
    ("publish_sonar_image", False, "When this flag is true, the original sensor image is published as a sensor_msgs/Image message", None),
    ("publish_cone_image", False, "When this flag is true, the processed cone projection of the sonar data is published as a sensor_msgs/Image message", None),
    ("publish_laserscan", True, "When this flag is true, the original sensor image is published as a sensor_msgs/Laserscan message", None),
    ("flip_input_x_sonar_image", False, "When this flag is true, the sonar image produced by the sensor is flipped on the x axis before any computation", None),
    ("flip_input_y_sonar_image", False, "When this flag is true, the sonar image produced by the sensor is flipped on the y axis before any computation", None),
    ("flip_output_x_sonar_image", True, "When this flag is true, and the sonar image is published, the original sensor image is published as a sensor_msgs/Image message with a flipped x axis", None),
    ("flip_output_y_sonar_image", True, "When this flag is true, and the sonar image is published, the original sensor image is published as a sensor_msgs/Image message with a flipped y axis", None),
    ("flip_output_x_sonar_image_cone", True, "When this flag is true, the cone projection image is published as a sensor_msgs/Image message with a flipped x axis", None),
    ("flip_output_y_sonar_image_cone", True, "When this flag is true, the cone projection image is published as a sensor_msgs/Image message with a flipped y axis", None),
    ("minimum_obstacle_value", 1, "The minimum pixel value of the sensor image to be considered as an obstacle", (1, 255)),
    ("mock_hardware", False, "When this flag is true, the driver does not connect to real hardware and publishes random data instead", None),
    ("draw_distance_lines", True, "When this flag is true, the driver draws distance lines on the sonar image", None),
    ("enabled", False, "When this flag is true, the driver starts the sensor", None),
]


class SonoptixEchoDriver(Node):
    def __init__(self):
        super().__init__("sonoptix_echo_driver_node")
        self.enable_sonar_service = self.declare_parameter(
            "enable_sonar_service", "enable_sonar"
        ).value
        self.srv = self.create_service(
            SetBool,
            self.enable_sonar_service,
            self.enable_sonar_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.ip = self.declare_parameter("ip", "192.168.2.42").value
        self.rtsp_port = self.declare_parameter("rtsp_port", 8554).value
        self.api_port = self.declare_parameter("api_port", 8000).value
        self.laserscan_topic = self.declare_parameter("laserscan_topic", "sonar_scan").value
        self.sonar_image_topic = self.declare_parameter("sonar_image_topic", "sonar_image").value
        self.pub_hz = self.declare_parameter("publish_hz", 25).value

        # NOTE: This debugging image topic is used to experiment with
        # laserscan generation using image messages from rosbags since the sensor
        # cannot be used
        self.debugging_sonar_image_sub_topic = self.declare_parameter("debugging_sonar_image_sub_topic", "").value

        # NOTE: The following parameters are included in dynamic reconfigure
        for name, default, description, limits in RECONFIGURABLE_PARAMETERS:
            descriptor = ParameterDescriptor(description=description)
            if isinstance(default, float):
                descriptor.floating_point_range = [
                    FloatingPointRange(from_value=limits[0], to_value=limits[1])
                ]
            elif limits is not None:
                descriptor.integer_range = [
                    IntegerRange(from_value=limits[0], to_value=limits[1], step=1)
                ]
            self.declare_parameter(name, default, descriptor)

        self.sonar_frame = self.get_parameter("sonar_frame").value
        self.sonar_range = self.get_parameter("sonar_range").value
        self.sonar_operation_mode = self.get_parameter("sonar_operation_mode").value #0:auto,1:HF,2:LF,3:LFLR (low frequency,low range)
        self.sonar_min_range = self.get_parameter("sonar_min_range").value
        self.image_data_factor = self.get_parameter("image_data_factor").value
        self.default_distance_value = self.get_parameter("default_distance_value").value
        self.default_distance_is_inf = self.get_parameter("default_distance_is_inf").value

        self.publish_sonar_image = self.get_parameter("publish_sonar_image").value
        self.publish_cone_image = self.get_parameter("publish_cone_image").value
        self.publish_laserscan = self.get_parameter("publish_laserscan").value
        self.flip_input_x_sonar_image = self.get_parameter("flip_input_x_sonar_image").value
        self.flip_input_y_sonar_image = self.get_parameter("flip_input_y_sonar_image").value
        self.flip_output_x_sonar_image = self.get_parameter("flip_output_x_sonar_image").value
        self.flip_output_y_sonar_image = self.get_parameter("flip_output_y_sonar_image").value
        self.flip_output_x_sonar_image_cone = self.get_parameter(
            "flip_output_x_sonar_image_cone"
        ).value
        self.flip_output_y_sonar_image_cone = self.get_parameter(
            "flip_output_y_sonar_image_cone"
        ).value
        self.mock_hardware = self.get_parameter("mock_hardware").value
        self.minimum_obstacle_value = self.get_parameter("minimum_obstacle_value").value
        self.sonar_enabled = self.get_parameter("enabled").value

        self.draw_distance_lines = self.get_parameter("draw_distance_lines").value

        # ------------------------------------------------------------------
        self.laserscan_pub = self.create_publisher(
            LaserScan, self.laserscan_topic, 10
        )
        self.image_pub = self.create_publisher(Image, self.sonar_image_topic, 10)
        self.cone_projection_pub = self.create_publisher(
            Image, self.sonar_image_topic + "_cone_projection", 10
        )
        self.rtsp_url = "rtsp://" + self.ip + ":" + str(self.rtsp_port) + "/raw"
        self.api_url = "http://" + self.ip + ":" + str(self.api_port) + "/api/v1"

        self.laserscan_pub_timer = self.create_timer(
            1.0 / self.pub_hz,
            self.sonar_read_laserscan_pub,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.image_pub_timer = self.create_timer(
            1.0 / self.pub_hz,
            self.sonar_read_image_pub,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.frame_reader_timer = self.create_timer(
            0.5 / self.pub_hz,
            self.frame_reader_worker,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # This is used to keep track of new frames coming from the sonar
        # since reading a frame does not guarantee new data. Each publisher can now
        # turn this flag off to avoid sending old data as new
        self.new_data_for_laserscan_image_and_cone = [False, False, False]

        self.mutex = threading.Lock()
        self.mutex_sonar_capture = threading.Lock()
        self.cv_bridge = CvBridge()
        self.sonar_capture = None
        self.latest_sonar_image = None

        self.pi_deg_ratio = np.pi / 180

        self.sonar_enabled = False
        self.sonar_fov_rad = 0.0
        self.set_sonar_range(self.sonar_range)

        # NOTE: Like the dynamic reconfigure server, the callback runs once at
        # startup and then on every change, always with the full config
        self.add_post_set_parameters_callback(self.parameters_callback)
        self.dyn_reconf_callback(self.reconfigurable_config())
        self.debugging_image_sub = None
        if self.debugging_sonar_image_sub_topic:
            self.debugging_image_sub = self.create_subscription(
                Image,
                self.debugging_sonar_image_sub_topic,
                self.debugging_image_callback,
                10,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )

    def debugging_image_callback(self, image_msg):
        sonar_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        flip_opt = None
        if self.flip_input_x_sonar_image and self.flip_input_y_sonar_image:
            flip_opt = -1
        elif self.flip_input_y_sonar_image:
            flip_opt = 1
        elif self.flip_input_x_sonar_image:
            flip_opt = 0
        if flip_opt is not None:
            sonar_image = cv2.flip(sonar_image, flip_opt)

        # NOTE: Every incoming image is new data, otherwise nothing is published
        # while the sonar is disabled
        with self.mutex:
            self.new_data_for_laserscan_image_and_cone = [True, True, True]

        self.publish_sonar_image_to_laserscan(sonar_image)
        self.publish_sonar_image_to_image(sonar_image)

    def reconfigurable_config(self):
        return SimpleNamespace(
            **{
                name: self.get_parameter(name).value
                for name, _, _, _ in RECONFIGURABLE_PARAMETERS
            }
        )

    def parameters_callback(self, parameters):
        reconfigurable_names = [name for name, _, _, _ in RECONFIGURABLE_PARAMETERS]
        if any(p.name in reconfigurable_names for p in parameters):
            self.dyn_reconf_callback(self.reconfigurable_config())

    def dyn_reconf_callback(self, config):
        self.get_logger().warn(str(config))
        with self.mutex:
            self.sonar_frame = config.sonar_frame
            self.image_data_factor = config.image_data_factor
            self.default_distance_value = config.default_distance_value
            self.default_distance_is_inf = config.default_distance_is_inf
            self.publish_sonar_image = config.publish_sonar_image
            self.publish_cone_image = config.publish_cone_image
            self.publish_laserscan = config.publish_laserscan
            self.flip_input_x_sonar_image = config.flip_input_x_sonar_image
            self.flip_input_y_sonar_image = config.flip_input_y_sonar_image
            self.flip_output_x_sonar_image = config.flip_output_x_sonar_image
            self.flip_output_y_sonar_image = config.flip_output_y_sonar_image
            self.flip_output_x_sonar_image_cone = config.flip_output_x_sonar_image_cone
            self.flip_output_y_sonar_image_cone = config.flip_output_y_sonar_image_cone
            self.sonar_min_range = config.sonar_min_range
            self.mock_hardware = config.mock_hardware
            self.draw_distance_lines = config.draw_distance_lines
            self.minimum_obstacle_value = config.minimum_obstacle_value
            if config.sonar_operation_mode != self.sonar_operation_mode:
                self.set_sonar_operation_mode(config.sonar_operation_mode)
            if config.sonar_range != self.sonar_range:
                self.set_sonar_range(config.sonar_range)
            if config.enabled != self.sonar_enabled:
                self.set_sonar_state(config.enabled)

    def enable_sonar_callback(self, request, response):
        with self.mutex:
            self.set_sonar_state(request.data)
        response.success = True
        response.message = ""
        return response

    def frame_reader_worker(self):
        """Continuously read frames from sonar capture and keep the latest one."""

        with self.mutex:
            if not self.sonar_enabled:
                if self.latest_sonar_image is not None:
                    self.latest_sonar_image = None
                self.get_logger().error("Sonar is not enabled", throttle_duration_sec=10)
                return
            if self.mock_hardware:
                self.new_data_for_laserscan_image_and_cone = [True, True, True]
                return

        with self.mutex_sonar_capture:
            if self.sonar_capture is None or not self.sonar_capture.isOpened():
                self.setup_sonar_capture()

            success, frame = self.sonar_capture.read()
            with self.mutex:
                if not np.array_equal(frame, self.latest_sonar_image):
                    self.latest_sonar_image = frame
                    self.new_data_for_laserscan_image_and_cone = [True, True, True]

            if not success:
                self.sonar_capture.release()
                self.sonar_capture = None
                self.get_logger().error(
                    "Frame reader thread: Error reading from sonar image stream"
                )

    def sonar_read_image_pub(self):
        with self.mutex:

            if not self.sonar_enabled:
                return
            if self.mock_hardware:
                height = 268
                if self.sonar_range >= 30:
                    height = 1024
                elif self.sonar_range >= 5:
                    height = 608

                sonar_image = np.random.randint(
                    0, 256, size=(height, 256, 3), dtype=np.uint8
                )
            else:
                if self.latest_sonar_image is None:
                    return
                sonar_image = self.latest_sonar_image

            # flip logic (fast, but we still measure inside mutex)
            if self.flip_input_x_sonar_image and self.flip_input_y_sonar_image:
                flip_opt = -1
            elif self.flip_input_y_sonar_image:
                flip_opt = 1
            elif self.flip_input_x_sonar_image:
                flip_opt = 0
            else:
                flip_opt = None

        if flip_opt is not None:
            sonar_image = cv2.flip(sonar_image, flip_opt)

        self.publish_sonar_image_to_image(sonar_image)

    def sonar_read_laserscan_pub(self):
        with self.mutex:

            if not self.sonar_enabled:
                return
            if self.mock_hardware:
                height = 268
                if self.sonar_range >= 30:
                    height = 1024
                elif self.sonar_range >= 5:
                    height = 608
                sonar_image = np.random.randint(
                    0, 256, size=(height, 256, 3), dtype=np.uint8
                )

            else:
                if self.latest_sonar_image is None:
                    return
                sonar_image = self.latest_sonar_image

            # flip logic (fast, but we still measure inside mutex)
            if self.flip_input_x_sonar_image and self.flip_input_y_sonar_image:
                flip_opt = -1
            elif self.flip_input_y_sonar_image:
                flip_opt = 1
            elif self.flip_input_x_sonar_image:
                flip_opt = 0
            else:
                flip_opt = None

        if flip_opt is not None:
            sonar_image = cv2.flip(sonar_image, flip_opt)

        self.publish_sonar_image_to_laserscan(sonar_image)

    def set_sonar_state(self, state):
        self.sonar_enabled = state

        if self.mock_hardware:
            return
        try:
            requests.patch(
                self.api_url + "/transponder",
                json={
                    "enable": self.sonar_enabled,
                    "sonar_range": self.sonar_range,
                    "operation_mode": self.sonar_operation_mode,
                },
                timeout=2,
            )
        except Exception as e:
            self.get_logger().error("Failed while enabling sonar:" + str(e))

    def set_sonar_range(self, r):
        previous_sonar_state = self.sonar_enabled
        self.sonar_range = r
        self.sonar_fov_rad = (
            120 * self.pi_deg_ratio
            if self.sonar_range <= 30
            else 90 * self.pi_deg_ratio
        )
        if self.mock_hardware:
            return
        self.set_sonar_state(False)
        if previous_sonar_state:
            time.sleep(1)
            self.set_sonar_state(True)

    def set_sonar_operation_mode(self, mode):
        previous_sonar_state = self.sonar_enabled
        self.sonar_operation_mode = mode
        if self.mock_hardware:
            return
        self.set_sonar_state(False)
        if previous_sonar_state:
            time.sleep(1)
            self.set_sonar_state(True)

    def set_datastream_to_rtsp(self):
        if self.mock_hardware:
            return
        try:
            requests.put(
                self.api_url + "/streamtype",
                json={
                    "value": 2,
                },
                timeout=2,
            )
        except Exception as e:
            self.get_logger().error("Failed while setting sonar datastream to rtsp:" + str(e))

    def setup_sonar_capture(self):

        if self.mock_hardware:
            return

        # self.set_sonar_state(self.sonar_enabled)
        self.set_datastream_to_rtsp()
        self.set_sonar_range(self.sonar_range)

        try:
            self.sonar_capture = cv2.VideoCapture(self.rtsp_url)
        except Exception as e:
            self.get_logger().error("Failed to create sonar image capture:" + str(e))

    def sonar_image_to_ranges(self, sonar_image):
        ranges = []
        height, width, _ = sonar_image.shape
        range_height_ratio = self.sonar_range / height

        with self.mutex:
            if self.default_distance_is_inf:
                self.default_distance_value = np.inf

        for w in range(width):
            beam = sonar_image[:, w, 0]
            distance = self.default_distance_value
            for h in range(height):
                if beam[h] >= self.minimum_obstacle_value:
                    distance_candidate = h * range_height_ratio
                    if distance_candidate >= self.sonar_min_range:
                        distance = distance_candidate
                        break

            ranges.append(distance)
        return ranges

    def sonar_image_to_cone_projection(self, sonar_image):
        """
        Project sonar image from rectangular beam format to polar cone projection.
        Each column of the input image represents a beam, emanating from the sonar position.

        Args:
            sonar_image: Input image where columns are beams and rows are ranges

        Returns:
            Projected image with polar cone layout, black outside FOV
        """
        height, width, channels = sonar_image.shape

        if self.sonar_fov_rad <= 0:
            output_size = height
            return np.zeros((output_size, output_size, channels), dtype=np.uint8)

        range_height_ratio = self.sonar_range / height
        pixels_per_meter = height / self.sonar_range

        # Calculate output size
        cone_width_meters = 2.0 * self.sonar_range * np.sin(self.sonar_fov_rad / 2.0)
        output_width_pixels = int(cone_width_meters * pixels_per_meter)
        output_size = max(output_width_pixels, height)

        cone_image = np.zeros((output_size, output_size, channels), dtype=np.uint8)

        sonar_x = output_size / 2.0
        sonar_y = output_size - 1.0

        angle_increment = self.sonar_fov_rad / (width - 1) if width > 1 else 0
        angle_start = -self.sonar_fov_rad / 2.0
        beam_angles = angle_start + np.arange(width) * angle_increment

        range_indices = np.arange(height)
        distances = range_indices * range_height_ratio

        angles_grid, distances_grid = np.meshgrid(beam_angles, distances)

        sin_angles = np.sin(angles_grid)
        cos_angles = np.cos(angles_grid)

        px_float = sonar_x + distances_grid * sin_angles * pixels_per_meter
        py_float = sonar_y - distances_grid * cos_angles * pixels_per_meter

        px = np.clip(px_float.astype(int), 0, output_size - 1)
        py = np.clip(py_float.astype(int), 0, output_size - 1)

        non_zero_mask = np.any(sonar_image != 0, axis=2)

        valid_indices = non_zero_mask
        cone_image[py[valid_indices], px[valid_indices], :] = sonar_image[
            valid_indices, :
        ]

        if not self.draw_distance_lines:
            return cone_image

        # Draw scale lines on both sides of the cone (red with 2-meter resolution)
        scale_color = (0, 0, 255)  # Red in BGR
        scale_line_length = 15  # Length of each scale tick
        scale_resolution = 2.0  # 2 meter resolution

        # Draw radial dotted lines connecting left and right edges at constant distances
        dot_spacing = 10  # Pixels between dots for dashed line
        dot_length = 5  # Length of each dot

        for dist in np.arange(0, self.sonar_range + 1, scale_resolution):
            # Generate points along the arc at this distance
            num_points = 100
            arc_points = []

            for i in range(num_points):
                # Interpolate angle from left to right
                angle = angle_start + (i / (num_points - 1)) * self.sonar_fov_rad

                # Convert polar to Cartesian
                px = int(sonar_x + dist * np.sin(angle) * pixels_per_meter)
                py = int(sonar_y - dist * np.cos(angle) * pixels_per_meter)

                # Only add points within bounds
                if 0 <= px < output_size and 0 <= py < output_size:
                    arc_points.append((px, py))

            # Draw dotted line connecting the arc points
            for j in range(len(arc_points) - 1):
                if (j // dot_spacing) % 2 == 0:  # Draw dot, skip space
                    pt1 = arc_points[j]
                    pt2 = arc_points[min(j + dot_length, len(arc_points) - 1)]
                    cv2.line(cone_image, pt1, pt2, scale_color, 1)

        # Left edge (negative angle limit)
        left_angle = angle_start
        for dist in np.arange(0, self.sonar_range + 1, scale_resolution):
            # Position on left edge
            px_center = int(sonar_x + dist * np.sin(left_angle) * pixels_per_meter)
            py_center = int(sonar_y - dist * np.cos(left_angle) * pixels_per_meter)

            # Perpendicular direction (inward from left edge)
            perp_angle = left_angle + np.pi / 2
            px_end = int(px_center + scale_line_length * np.cos(perp_angle))
            py_end = int(py_center + scale_line_length * np.sin(perp_angle))

            if 0 <= px_center < output_size and 0 <= py_center < output_size:
                # cv2.line(cone_image, (px_center, py_center), (px_end, py_end), scale_color, 2)
                cv2.circle(cone_image, (px_center, py_center), 1, scale_color, -1)
        # Right edge (positive angle limit)
        right_angle = angle_start + self.sonar_fov_rad
        for dist in np.arange(0, self.sonar_range + 1, scale_resolution):
            # Position on right edge
            px_center = int(sonar_x + dist * np.sin(right_angle) * pixels_per_meter)
            py_center = int(sonar_y - dist * np.cos(right_angle) * pixels_per_meter)

            # Perpendicular direction (inward from right edge)
            perp_angle = right_angle - np.pi / 2
            px_end = int(px_center + scale_line_length * np.cos(perp_angle))
            py_end = int(py_center + scale_line_length * np.sin(perp_angle))

            if 0 <= px_center < output_size and 0 <= py_center < output_size:
                # cv2.line(cone_image, (px_center, py_center), (px_end, py_end), scale_color, 2)
                cv2.circle(cone_image, (px_center, py_center), 1, scale_color, -1)
        # DEBUG: Visualize a random beam with yellow points
        random_beam_idx = int(
            width // 3
        )  # For consistent debugging, use the center beam
        beam_angle = angle_start + random_beam_idx * angle_increment

        for range_idx in range(height):
            pixel = sonar_image[range_idx, random_beam_idx, :]
            if np.all(pixel == 0):
                continue

            distance = range_idx * range_height_ratio
            px = int(sonar_x + distance * np.sin(beam_angle) * pixels_per_meter)
            py = int(sonar_y - distance * np.cos(beam_angle) * pixels_per_meter)

            if 0 <= px < output_size and 0 <= py < output_size:
                cv2.circle(cone_image, (px, py), 1, (0, 255, 255), -1)  # Yellow points

        # DEBUG: Visualize specific ranges - small and large
        small_range_idx = int(height * 0.1)  # 10% from start (close to sensor)
        large_range_idx = int(height * 0.9)  # 90% from start (far from sensor)

        small_distance = small_range_idx * range_height_ratio
        large_distance = large_range_idx * range_height_ratio

        # Mark all beams at small range with red circles
        for beam_idx in range(width):
            beam_angle = angle_start + beam_idx * angle_increment
            px = int(sonar_x + small_distance * np.sin(beam_angle) * pixels_per_meter)
            py = int(sonar_y - small_distance * np.cos(beam_angle) * pixels_per_meter)
            if 0 <= px < output_size and 0 <= py < output_size:
                cv2.circle(cone_image, (px, py), 1, (0, 0, 255), -1)  # Red circles

        # Mark all beams at large range with cyan circles
        for beam_idx in range(width):
            beam_angle = angle_start + beam_idx * angle_increment
            px = int(sonar_x + large_distance * np.sin(beam_angle) * pixels_per_meter)
            py = int(sonar_y - large_distance * np.cos(beam_angle) * pixels_per_meter)
            if 0 <= px < output_size and 0 <= py < output_size:
                cv2.circle(cone_image, (px, py), 1, (255, 255, 0), -1)  # Cyan circles

        return cone_image

    def publish_sonar_image_to_laserscan(self, sonar_image):
        with self.mutex:
            if not (
                self.publish_laserscan and self.new_data_for_laserscan_image_and_cone[0]
            ):
                return
        self.new_data_for_laserscan_image_and_cone[0] = False
        laserscan_msg = LaserScan()
        laserscan_msg.header.frame_id = self.sonar_frame
        laserscan_msg.header.stamp = self.get_clock().now().to_msg()
        laserscan_msg.angle_min = -self.sonar_fov_rad / 2.0
        laserscan_msg.angle_max = self.sonar_fov_rad / 2.0
        laserscan_msg.angle_increment = self.sonar_fov_rad / 256
        laserscan_msg.scan_time = 1.0 / self.pub_hz
        laserscan_msg.range_min = self.sonar_min_range
        laserscan_msg.range_max = np.inf
        laserscan_msg.ranges = self.sonar_image_to_ranges(sonar_image)
        self.laserscan_pub.publish(laserscan_msg)

    def publish_sonar_image_to_image(self, sonar_image):
        if not (self.publish_sonar_image or self.publish_cone_image):
            return

        if self.image_data_factor != 1.0:
            sonar_image = np.clip(
                sonar_image.astype(np.float32) * self.image_data_factor, 0, 255
            ).astype(np.uint8)

        if self.publish_sonar_image and self.new_data_for_laserscan_image_and_cone[1]:
            flip_opt = None
            sonar_image_flipped = sonar_image
            if self.flip_output_x_sonar_image and self.flip_output_y_sonar_image:
                flip_opt = -1
            elif self.flip_output_y_sonar_image:
                flip_opt = 1
            elif self.flip_output_x_sonar_image:
                flip_opt = 0
            if flip_opt is not None:
                sonar_image_flipped = cv2.flip(sonar_image, flip_opt)

            image_msg = self.cv_bridge.cv2_to_imgmsg(
                sonar_image_flipped, encoding="bgr8"
            )
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = self.sonar_frame
            self.image_pub.publish(image_msg)
            self.new_data_for_laserscan_image_and_cone[1] = False

        if self.publish_cone_image and self.new_data_for_laserscan_image_and_cone[2]:

            flip_opt_cone = None
            sonar_image_flipped = sonar_image
            if (
                self.flip_output_x_sonar_image_cone
                and self.flip_output_y_sonar_image_cone
            ):
                flip_opt_cone = -1
            elif self.flip_output_y_sonar_image_cone:
                flip_opt_cone = 1
            elif self.flip_output_x_sonar_image_cone:
                flip_opt_cone = 0
            if flip_opt_cone is not None:
                sonar_image_flipped = cv2.flip(sonar_image, flip_opt_cone)

            # Publish cone projection
            cone_projection = self.sonar_image_to_cone_projection(sonar_image_flipped)
            cone_msg = self.cv_bridge.cv2_to_imgmsg(cone_projection, encoding="bgr8")
            cone_msg.header.stamp = self.get_clock().now().to_msg()
            cone_msg.header.frame_id = self.sonar_frame
            self.cone_projection_pub.publish(cone_msg)
            self.new_data_for_laserscan_image_and_cone[2] = False


def main():
    try:
        rclpy.init()
        # NOTE: rospy ran every timer/service/subscriber in its own thread.
        # Each callback gets its own MutuallyExclusiveCallbackGroup to match it,
        # so enough threads are needed for all of them to run at once
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(SonoptixEchoDriver())
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
