#include "sonoptix_echo_driver_ros/sonoptix_echo_driver.hpp"

#include <httplib.h>

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <utility>

#include <cv_bridge/cv_bridge.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "sonoptix_echo_driver_ros/sonar_image_processing.hpp"

namespace sonoptix_echo_driver_ros
{

// This is used to implement a token bucket rate limiter.
// This mechanism is required since sensor frames are consumed at the instant they become available.
// Since every publish period adds a token to the bucket, using a max of 2 allows for a burst of 2 frames (aka faster
// than publish period) if there was a delay in the acquisition loop.
constexpr int max_publish_tokens = 2;

// get_subscription_count is not enough here since this node can also be a component
template <typename PublisherT>
bool has_subscribers(const PublisherT& publisher)
{ return publisher.get_subscription_count() + publisher.get_intra_process_subscription_count() > 0; }

SonoptixEchoDriver::SonoptixEchoDriver(const rclcpp::NodeOptions& options)
  : rclcpp_lifecycle::LifecycleNode("sonoptix_echo_driver_node", options), param_listener_(this)
{
  param_listener_.setUserCallback([this](const Params& config) { reconfigure_callback(config); });
}

SonoptixEchoDriver::~SonoptixEchoDriver()
{
  stop_acquisition();
  deactivate_sonar();
}

SonoptixEchoDriver::CallbackReturn SonoptixEchoDriver::on_configure(const rclcpp_lifecycle::State&)
{
  const Params params = param_listener_.get_params();

  // Possible options can be seen by running:
  // `ffmpeg -hide_banner -h demuxer=rtsp`
  // (the list is dependent on the ffpeg installation)
  if (!params.rtsp_capture_options.empty())
  {
    setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", params.rtsp_capture_options.c_str(), 1);
  }
  rtsp_url_ = "rtsp://" + params.ip + ":" + std::to_string(params.rtsp_port) + "/raw";
  publish_period_ = std::chrono::duration<double>(1.0 / static_cast<double>(params.publish_hz));

  api_ = std::make_unique<httplib::Client>(params.ip, static_cast<int>(params.api_port));
  const auto http_timeout =
      std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<double>(params.http_request_timeout));
  api_->set_connection_timeout(http_timeout);
  api_->set_read_timeout(http_timeout);
  api_->set_write_timeout(http_timeout);

  laserscan_pub_ = create_publisher<LaserScan>(params.laserscan_topic, 10);
  image_pub_ = create_publisher<Image>(params.sonar_image_topic, 10);
  cone_projection_pub_ = create_publisher<Image>(params.sonar_image_topic + "_cone_projection", 10);

  std::lock_guard<std::mutex> lock(state_mutex_);
  sonar_enabled_ = false;
  sonar_range_ = params.sonar_range;
  // Operatio mode can be 0:auto, 1:HF, 2:LF, 3:LFLR
  sonar_operation_mode_ = params.sonar_operation_mode;
  set_sonar_range(sonar_range_);
  RCLCPP_INFO(get_logger(), "Configured (sonar API %s:%s)", params.ip.c_str(), std::to_string(params.api_port).c_str());
  return CallbackReturn::SUCCESS;
}

SonoptixEchoDriver::CallbackReturn SonoptixEchoDriver::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  // activates the publishers
  LifecycleNode::on_activate(previous_state);

  const Params params = param_listener_.get_params();
  {
    // Applies the parameters that need sonar API calls (like enabled)
    std::lock_guard<std::mutex> lock(state_mutex_);
    active_ = true;
    apply_sonar_config(params);
  }

  if (!params.debugging_sonar_image_sub_topic.empty())
  {
    debugging_image_sub_ =
        create_subscription<Image>(params.debugging_sonar_image_sub_topic, 10,
                                   [this](const Image::ConstSharedPtr& msg) { debugging_image_callback(msg); });
  }

  start_acquisition();
  RCLCPP_INFO(get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

SonoptixEchoDriver::CallbackReturn SonoptixEchoDriver::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  stop_acquisition();
  debugging_image_sub_.reset();
  deactivate_sonar();
  // deactivates the publishers
  LifecycleNode::on_deactivate(previous_state);
  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

SonoptixEchoDriver::CallbackReturn SonoptixEchoDriver::on_cleanup(const rclcpp_lifecycle::State&)
{
  destroy_interfaces();
  return CallbackReturn::SUCCESS;
}

SonoptixEchoDriver::CallbackReturn SonoptixEchoDriver::on_shutdown(const rclcpp_lifecycle::State& previous_state)
{
  if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    stop_acquisition();
    debugging_image_sub_.reset();
    deactivate_sonar();
  }
  destroy_interfaces();
  return CallbackReturn::SUCCESS;
}

SonoptixEchoDriver::CallbackReturn SonoptixEchoDriver::on_error(const rclcpp_lifecycle::State&)
{
  stop_acquisition();
  debugging_image_sub_.reset();
  deactivate_sonar();
  destroy_interfaces();
  return CallbackReturn::SUCCESS;
}

void SonoptixEchoDriver::start_acquisition()
{
  stop_acquisition();
  previous_frame_.release();
  // start with one token to "be ready" publish on first acquisition
  publish_tokens_ = 1;
  last_token_update_ = Clock::now();
  acquisition_thread_ = std::jthread([this](std::stop_token st) { acquisition_loop(st); });
}

void SonoptixEchoDriver::stop_acquisition()
{
  if (acquisition_thread_.joinable())
  {
    acquisition_thread_.request_stop();
    acquisition_thread_.join();
  }
  sonar_capture_.release();
  previous_frame_.release();
}

void SonoptixEchoDriver::deactivate_sonar()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  active_ = false;
  if (api_ && sonar_enabled_)
  {
    set_sonar_state(false);
  }
  sonar_enabled_ = false;
}

void SonoptixEchoDriver::destroy_interfaces()
{
  laserscan_pub_.reset();
  image_pub_.reset();
  cone_projection_pub_.reset();
  api_.reset();
  sonar_capture_.release();
  previous_frame_.release();
}

void SonoptixEchoDriver::reconfigure_callback(const Params& config)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (active_)
  {
    apply_sonar_config(config);
  }
}

void SonoptixEchoDriver::debugging_image_callback(const Image::ConstSharedPtr& image_msg)
{
  const auto cv_image = cv_bridge::toCvShare(image_msg, "bgr8");
  // NOTE: Not taking into acccount if the sensor is enabled
  publish_sonar_image(cv_image->image, now());
}

void SonoptixEchoDriver::acquisition_loop(std::stop_token stop_token)
{
  RCLCPP_DEBUG(get_logger(), "Frame acquisition started");
  while (!stop_token.stop_requested())
  {
    if (!wait_for_sonar_enabled(stop_token, Clock::duration::zero()))
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), steady_clock_, 10000, "Sonar is not enabled");
      wait_for_sonar_enabled(stop_token, std::chrono::seconds(10));
      continue;
    }

    if (get_mock_hardware())
    {
      const auto next_frame_time = Clock::now() + std::chrono::duration_cast<Clock::duration>(publish_period_);
      publish_sonar_image(generate_mock_sonar_image(), now());
      sleep_until(stop_token, next_frame_time);
      continue;
    }

    if (!sonar_capture_.isOpened())
    {
      open_sonar_capture();
    }

    cv::Mat frame;
    if (!sonar_capture_.isOpened() || !sonar_capture_.read(frame))
    {
      sonar_capture_.release();
      previous_frame_.release();
      RCLCPP_ERROR(get_logger(), "Error reading from sonar image stream");
      continue;
    }
    const rclcpp::Time stamp = now();

    // Make sure the frame is actually a new one
    if (sonar_image_processing_.images_equal(frame, previous_frame_))
    {
      continue;
    }
    previous_frame_ = frame;

    if (publish_rate_allows(Clock::now()))
    {
      publish_sonar_image(frame, stamp);
    }
  }
  RCLCPP_DEBUG(get_logger(), "Frame acquisition stopped");
}

bool SonoptixEchoDriver::wait_for_sonar_enabled(const std::stop_token& stop_token, Clock::duration timeout)
{
  std::unique_lock<std::mutex> lock(state_mutex_);
  return state_changed_.wait_for(lock, stop_token, timeout, [this]() { return sonar_enabled_; });
}

bool SonoptixEchoDriver::sleep_until(const std::stop_token& stop_token, Clock::time_point deadline)
{
  std::unique_lock<std::mutex> lock(state_mutex_);
  state_changed_.wait_until(lock, stop_token, deadline, []() { return false; });
  return !stop_token.stop_requested();
}

bool SonoptixEchoDriver::open_sonar_capture()
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (get_mock_hardware())
    {
      return false;
    }
    set_datastream_to_rtsp();
    set_sonar_range(sonar_range_);
  }

  const auto params = param_listener_.get_params();
  try
  {
    sonar_capture_.open(rtsp_url_, cv::CAP_FFMPEG,
                        {
                            cv::CAP_PROP_N_THREADS,
                            1,
                            cv::CAP_PROP_OPEN_TIMEOUT_MSEC,
                            static_cast<int>(params.rtsp_open_timeout * 1000),
                            cv::CAP_PROP_READ_TIMEOUT_MSEC,
                            static_cast<int>(params.rtsp_read_timeout * 1000),
                        });
  }
  catch (const cv::Exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Failure while creating sonar image capture: %s", e.what());
  }
  if (sonar_capture_.isOpened())
  {
    RCLCPP_INFO(get_logger(), "Connected to the sonar image stream %s", rtsp_url_.c_str());
  }
  return sonar_capture_.isOpened();
}

cv::Mat SonoptixEchoDriver::generate_mock_sonar_image()
{
  double sonar_range;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    sonar_range = sonar_range_;
  }
  const int height = sonar_range >= 30 ? 1024 : sonar_range >= 5 ? 608 : 268;
  cv::Mat sonar_image(height, 256, CV_8UC3);
  cv::randu(sonar_image, cv::Scalar::all(0), cv::Scalar::all(256));
  return sonar_image;
}

// Implmementing a publishing rate token bucket here:
// Tokens are replenished in publish_period_ rate, with a maximum of max_publish_tokens
// The node can publish only if a token is available and can burst in case of a delay during acquisition
bool SonoptixEchoDriver::publish_rate_allows(Clock::time_point now)
{
  const double elapsed_periods = std::chrono::duration<double>(now - last_token_update_) / publish_period_;
  last_token_update_ = now;
  publish_tokens_ = std::min(max_publish_tokens, static_cast<int>(publish_tokens_ + elapsed_periods));
  if (publish_tokens_ < 1)
  {
    return false;
  }
  publish_tokens_--;
  return true;
}

void SonoptixEchoDriver::apply_sonar_config(const Params& config)
{
  if (config.sonar_operation_mode != sonar_operation_mode_)
  {
    set_sonar_operation_mode(config.sonar_operation_mode);
  }
  if (config.sonar_range != sonar_range_)
  {
    set_sonar_range(config.sonar_range);
  }
  if (config.enabled != sonar_enabled_)
  {
    set_sonar_state(config.enabled);
  }
}

void SonoptixEchoDriver::set_sonar_state(bool state)
{
  sonar_enabled_ = state;
  state_changed_.notify_all();
  if (get_mock_hardware())
  {
    return;
  }
  const nlohmann::ordered_json body = { { "enable", sonar_enabled_ },
                                        { "sonar_range", sonar_range_ },
                                        { "operation_mode", sonar_operation_mode_ } };
  const auto result = api_->Patch("/api/v1/transponder", body.dump(), "application/json");
  if (!result)
  {
    RCLCPP_ERROR(get_logger(), "Failed while enabling sonar:%s", httplib::to_string(result.error()).c_str());
  }
}

void SonoptixEchoDriver::set_sonar_range(double range)
{
  const bool previous_sonar_state = sonar_enabled_;
  sonar_range_ = range;
  sonar_fov_rad_ = sonar_range_ <= 30 ? 120 * (std::numbers::pi / 180) : 90 * (std::numbers::pi / 180);
  if (get_mock_hardware())
  {
    return;
  }
  restart_sonar(previous_sonar_state);
}

void SonoptixEchoDriver::set_sonar_operation_mode(int64_t mode)
{
  const bool previous_sonar_state = sonar_enabled_;
  sonar_operation_mode_ = mode;
  if (get_mock_hardware())
  {
    return;
  }
  restart_sonar(previous_sonar_state);
}

void SonoptixEchoDriver::restart_sonar(bool previous_sonar_state)
{
  set_sonar_state(false);
  if (previous_sonar_state)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    set_sonar_state(true);
  }
}

void SonoptixEchoDriver::set_datastream_to_rtsp()
{
  if (get_mock_hardware())
  {
    return;
  }
  const nlohmann::ordered_json body = { { "value", 2 } };
  const auto result = api_->Put("/api/v1/streamtype", body.dump(), "application/json");
  if (!result)
  {
    RCLCPP_ERROR(get_logger(), "Failed while setting sonar datastream to rtsp:%s",
                 httplib::to_string(result.error()).c_str());
  }
}

bool SonoptixEchoDriver::get_mock_hardware() const
{ return param_listener_.get_params().mock_hardware; }

void SonoptixEchoDriver::publish_sonar_image(const cv::Mat& raw_sonar_image, const rclcpp::Time& stamp)
{
  const Params config = param_listener_.get_params();
  double sonar_range, sonar_fov_rad;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    sonar_range = sonar_range_;
    sonar_fov_rad = sonar_fov_rad_;
  }
  const cv::Mat sonar_image = sonar_image_processing_.flip_image(raw_sonar_image, config.flip_input_x_sonar_image,
                                                                 config.flip_input_y_sonar_image);

  publish_laserscan(sonar_image, config, sonar_range, sonar_fov_rad, stamp);
  publish_images(sonar_image, config, sonar_range, sonar_fov_rad, stamp);
}

void SonoptixEchoDriver::publish_laserscan(const cv::Mat& sonar_image, const Params& config, double sonar_range,
                                           double sonar_fov_rad, const rclcpp::Time& stamp)
{
  if (!config.publish_laserscan || !has_subscribers(*laserscan_pub_))
  {
    return;
  }
  const double default_distance_value =
      config.default_distance_is_inf ? std::numeric_limits<double>::infinity() : config.default_distance_value;

  auto laserscan_msg = std::make_unique<LaserScan>();
  laserscan_msg->header.frame_id = config.sonar_frame;
  laserscan_msg->header.stamp = stamp;
  laserscan_msg->angle_min = static_cast<float>(-sonar_fov_rad / 2.0);
  laserscan_msg->angle_max = static_cast<float>(sonar_fov_rad / 2.0);
  laserscan_msg->angle_increment = static_cast<float>(sonar_fov_rad / 256);
  laserscan_msg->scan_time = static_cast<float>(publish_period_.count());
  laserscan_msg->range_min = static_cast<float>(config.sonar_min_range);
  laserscan_msg->range_max = std::numeric_limits<float>::infinity();
  laserscan_msg->ranges = sonar_image_processing_.sonar_image_to_ranges(
      sonar_image, sonar_range, default_distance_value, config.sonar_min_range, config.minimum_obstacle_value);
  laserscan_pub_->publish(std::move(laserscan_msg));
}

void SonoptixEchoDriver::publish_images(const cv::Mat& sonar_image, const Params& config, double sonar_range,
                                        double sonar_fov_rad, const rclcpp::Time& stamp)
{
  // NOTE: Skipping publication(s) if noone is subscribed
  const bool publish_sonar_image = config.publish_sonar_image && has_subscribers(*image_pub_);
  const bool publish_cone_image = config.publish_cone_image && has_subscribers(*cone_projection_pub_);
  if (!publish_sonar_image && !publish_cone_image)
  {
    return;
  }

  cv::Mat image = sonar_image;
  if (config.image_data_factor != 1.0)
  {
    image = sonar_image_processing_.apply_image_data_factor(sonar_image, config.image_data_factor);
  }

  if (publish_sonar_image)
  {
    publish_image(*image_pub_,
                  sonar_image_processing_.flip_image(image, config.flip_output_x_sonar_image,
                                                     config.flip_output_y_sonar_image),
                  config.sonar_frame, stamp);
  }

  if (publish_cone_image)
  {
    const cv::Mat flipped = sonar_image_processing_.flip_image(image, config.flip_output_x_sonar_image_cone,
                                                               config.flip_output_y_sonar_image_cone);
    publish_image(*cone_projection_pub_,
                  cone_projector_.project(flipped, sonar_range, sonar_fov_rad, config.draw_distance_lines),
                  config.sonar_frame, stamp);
  }
}

void SonoptixEchoDriver::publish_image(rclcpp_lifecycle::LifecyclePublisher<Image>& publisher, const cv::Mat& image,
                                       const std::string& frame_id, const rclcpp::Time& stamp)
{
  auto msg = std::make_unique<Image>();
  cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg(*msg);
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id;
  publisher.publish(std::move(msg));
}

}  // namespace sonoptix_echo_driver_ros

RCLCPP_COMPONENTS_REGISTER_NODE(sonoptix_echo_driver_ros::SonoptixEchoDriver)
