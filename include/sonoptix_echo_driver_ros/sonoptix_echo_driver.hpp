#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>

#include <httplib.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "sonoptix_echo_driver_ros/sonoptix_echo_driver_parameters.hpp"
#include "sonoptix_echo_driver_ros/sonar_image_processing.hpp"
#include "sonoptix_echo_driver_ros/cone_projector.hpp"

namespace sonoptix_echo_driver_ros
{

class SonoptixEchoDriver : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Params = sonoptix_echo_driver::Params;

  explicit SonoptixEchoDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~SonoptixEchoDriver() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

private:
  using Image = sensor_msgs::msg::Image;
  using LaserScan = sensor_msgs::msg::LaserScan;
  using SetBool = std_srvs::srv::SetBool;
  using Clock = std::chrono::steady_clock;

  void start_acquisition();
  void stop_acquisition();
  void deactivate_sonar();
  void destroy_interfaces();

  void reconfigure_callback(const Params& config);
  void debugging_image_callback(const Image::ConstSharedPtr& image_msg);

  void acquisition_loop(std::stop_token stop_token);
  bool wait_for_sonar_enabled(const std::stop_token& stop_token, Clock::duration timeout);
  bool sleep_until(const std::stop_token& stop_token, Clock::time_point deadline);
  bool open_sonar_capture();
  cv::Mat generate_mock_sonar_image();
  bool publish_rate_allows(Clock::time_point now);

  void apply_sonar_config(const Params& config);
  void set_sonar_state(bool state);
  void set_sonar_range(double range);
  void set_sonar_operation_mode(int64_t mode);
  void restart_sonar(bool previous_sonar_state);
  void set_datastream_to_rtsp();
  bool get_mock_hardware() const;

  void publish_sonar_image(const cv::Mat& raw_sonar_image, const rclcpp::Time& stamp);
  void publish_laserscan(const cv::Mat& sonar_image, const Params& config, double sonar_range, double sonar_fov_rad,
                         const rclcpp::Time& stamp);
  void publish_images(const cv::Mat& sonar_image, const Params& config, double sonar_range, double sonar_fov_rad,
                      const rclcpp::Time& stamp);
  void publish_image(rclcpp_lifecycle::LifecyclePublisher<Image>& publisher, const cv::Mat& image,
                     const std::string& frame_id, const rclcpp::Time& stamp);

  sonoptix_echo_driver::ParamListener param_listener_;

  std::string rtsp_url_;
  bool active_{ false };
  int publish_tokens_{ 0 };
  cv::Mat previous_frame_;
  std::mutex state_mutex_;
  double sonar_range_{ 10.0 };
  bool sonar_enabled_{ false };
  double sonar_fov_rad_{ 0.0 };
  ConeProjector cone_projector_;
  cv::VideoCapture sonar_capture_;
  std::jthread acquisition_thread_;
  int64_t sonar_operation_mode_{ 0 };
  Clock::time_point last_token_update_;
  std::unique_ptr<httplib::Client> api_;
  std::condition_variable_any state_changed_;
  SonarImageProcessing sonar_image_processing_;
  rclcpp::Clock steady_clock_{ RCL_STEADY_TIME };
  std::chrono::duration<double> publish_period_{ 0.04 };
  rclcpp::Subscription<Image>::SharedPtr debugging_image_sub_;
  rclcpp_lifecycle::LifecyclePublisher<Image>::SharedPtr image_pub_;
  rclcpp_lifecycle::LifecyclePublisher<LaserScan>::SharedPtr laserscan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<Image>::SharedPtr cone_projection_pub_;
};

}  // namespace sonoptix_echo_driver_ros
