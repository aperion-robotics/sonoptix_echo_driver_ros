#pragma once

#include <cstdint>
#include <mutex>
#include <vector>

#include <opencv2/core.hpp>

namespace sonoptix_echo_driver_ros
{

class ConeProjector
{
public:
  cv::Mat project(const cv::Mat& sonar_image, double sonar_range, double sonar_fov_rad, bool draw_distance_lines);

private:
  double ring_step(double sonar_range);

  struct Geometry
  {
    int rows{ 0 };
    int columns{ 0 };
    double fov{ 0.0 };
    cv::Size size;
    cv::Point2d apex;

    struct RowSpan
    {
      int begin;
      int end;
      // position of the row's first pixel in source_indices
      size_t first_index;
    };
    std::vector<RowSpan> row_spans;
    std::vector<int32_t> source_indices;
  };

  struct GridPixel
  {
    // byte offset in the output image
    int offset;
    // 0-255 (opacity)
    uint8_t alpha;
    // pixel value (after applying alpha)
    uint8_t color[3];
  };

  struct Grid
  {
    int rows{ 0 };
    int columns{ 0 };
    double fov{ 0.0 };
    double range{ 0.0 };
    int channels{ 0 };
    std::vector<GridPixel> pixels;
  };

  void update_geometry(int rows, int columns, double fov);
  void update_grid(double sonar_range, int channels);

  Grid grid_;
  std::mutex mutex_;
  Geometry geometry_;
};

}  // namespace sonoptix_echo_driver_ros
