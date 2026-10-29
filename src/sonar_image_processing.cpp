#include "sonoptix_echo_driver_ros/sonar_image_processing.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace sonoptix_echo_driver_ros
{

std::optional<int> SonarImageProcessing::flip_code(bool flip_x, bool flip_y)
{
  if (flip_x && flip_y)
  {
    return -1;
  }
  else if (flip_y)
  {
    return 1;
  }
  else if (flip_x)
  {
    return 0;
  }
  return std::nullopt;
}

cv::Mat SonarImageProcessing::flip_image(const cv::Mat& image, bool flip_x, bool flip_y)
{
  const auto flip_opt = flip_code(flip_x, flip_y);
  if (!flip_opt)
  {
    return image;
  }
  cv::Mat flipped;
  cv::flip(image, flipped, *flip_opt);
  return flipped;
}

cv::Mat SonarImageProcessing::apply_image_data_factor(const cv::Mat& image, double image_data_factor)
{
  // This is also guarded before this function is called, but also guarded here for OCD's sake
  if (image_data_factor == 1.0)
  {
    return image;
  }

  // instead of multiplying each pixel with the factor
  // (while clipping to [0,255]) a LUT is used to maximize performance
  if (image_data_factor != cached_factor_)
  {
    cached_lut_.create(1, 256, CV_8U);

    for (int v = 0; v < 256; ++v)
    {
      const double scaled = std::clamp(v * image_data_factor, 0.0, 255.0);
      cached_lut_.at<uint8_t>(v) = static_cast<uint8_t>(scaled);
    }
    cached_factor_ = image_data_factor;
  }

  cv::Mat out;
  cv::LUT(image, cached_lut_, out);
  return out;
}

std::vector<float> SonarImageProcessing::sonar_image_to_ranges(const cv::Mat& sonar_image, double sonar_range,
                                                               double default_distance_value, double sonar_min_range,
                                                               int64_t minimum_obstacle_value)
{
  const int height = sonar_image.rows;
  const int width = sonar_image.cols;
  const int channels = sonar_image.channels();
  const double range_height_ratio = sonar_range / height;

  std::vector<float> ranges(static_cast<size_t>(width), static_cast<float>(default_distance_value));
  if (width == 0)
  {
    return ranges;
  }

  // Same result as scanning each beam (column) from the sensor outwards and
  // keeping the first hit, but row-major for cache friendliness
  std::vector<uint8_t> found(static_cast<size_t>(width), 0);
  int remaining = width;
  for (int h = 0; h < height && remaining > 0; h++)
  {
    const double distance_candidate = h * range_height_ratio;
    if (!(distance_candidate >= sonar_min_range))
    {
      continue;
    }
    const uint8_t* row = sonar_image.ptr<uint8_t>(h);
    for (int w = 0; w < width; w++)
    {
      if (!found[w] && row[w * channels] >= minimum_obstacle_value)
      {
        ranges[w] = static_cast<float>(distance_candidate);
        found[w] = 1;
        remaining--;
      }
    }
  }
  return ranges;
}

bool SonarImageProcessing::images_equal(const cv::Mat& a, const cv::Mat& b)
{
  if (a.type() != b.type() || a.size() != b.size())
  {
    return false;
  }
  if (a.empty() || b.empty())
  {
    return a.empty() && b.empty();
  }
  if (a.data == b.data && a.step == b.step)
  {
    return true;
  }
  const size_t row_bytes = static_cast<size_t>(a.cols) * a.elemSize();
  for (int r = 0; r < a.rows; r++)
  {
    if (std::memcmp(a.ptr(r), b.ptr(r), row_bytes) != 0)
    {
      return false;
    }
  }
  return true;
}

}  // namespace sonoptix_echo_driver_ros
