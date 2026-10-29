#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

namespace sonoptix_echo_driver_ros
{
class SonarImageProcessing
{
public:
  cv::Mat flip_image(const cv::Mat& image, bool flip_x, bool flip_y);

  cv::Mat apply_image_data_factor(const cv::Mat& image, double image_data_factor);

  std::vector<float> sonar_image_to_ranges(const cv::Mat& sonar_image, double sonar_range,
                                           double default_distance_value, double sonar_min_range,
                                           int64_t minimum_obstacle_value);

  bool images_equal(const cv::Mat& a, const cv::Mat& b);

private:
  std::optional<int> flip_code(bool flip_x, bool flip_y);

  cv::Mat cached_lut_;
  double cached_factor_{ 1.0 };
};

}  // namespace sonoptix_echo_driver_ros
