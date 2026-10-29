#include "sonoptix_echo_driver_ros/cone_projector.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace sonoptix_echo_driver_ros
{

namespace
{
// TODO: make the overlay colour configurable
const cv::Scalar grid_color(0, 255, 0);
constexpr double grid_opacity = 0.6;
constexpr double label_opacity = 0.9;
constexpr double bearing_step = std::numbers::pi / 6;  // 30deg
constexpr int target_ring_count = 5;
constexpr int draw_shift = 4;  // sub-pixel precision for antialiasing
constexpr double draw_scale = 1 << draw_shift;

cv::Point subpixel_point(double x, double y)
{ return cv::Point(static_cast<int>(std::lround(x * draw_scale)), static_cast<int>(std::lround(y * draw_scale))); }

std::string distance_label(double distance)
{
  std::ostringstream label;
  label << distance << " m";
  return label.str();
}

}  // namespace

// ring_step calculates the optimal ring distance (step) on the overlayed grid based on how many rings are ideally
// required. It is heavily documented because its math will probably be unintelligible to my future self (or colleagues)
double ConeProjector::ring_step(double sonar_range)
{
  // raw_step is the ideal spacing based on target_ring_count
  const double raw_step = sonar_range / target_ring_count;
  // float_tolerant_threshold is raw_step with float point error tolerance that is later checked upon
  const double float_tolerant_threshold = raw_step * (1.0 - 1e-9);
  // magnitude is the power of 10 of the scale of raw_step
  // i.e. 1 for < 10, 10 for < 100, 100 for < 1000 etc
  // aka we are using a raw_step of units, tens or hundreds
  const double magnitude = std::pow(10.0, std::floor(std::log10(raw_step)));
  // mantissa is set to a predefined "good" spacing value list. Inside the loop, the closest to at least the fault
  // tolerant raw_step is investigated. If not found, a 10x the magnitude is used as the step. More on mantissa:
  // https://en.wikipedia.org/wiki/Common_logarithm#Mantissa_and_characteristic
  for (const double mantissa : { 1.0, 2.0, 2.5, 5.0 })
  {
    const double candidate_step = mantissa * magnitude;
    if (candidate_step >= float_tolerant_threshold)
    {
      return candidate_step;
    }
  }
  return 10.0 * magnitude;
}

cv::Mat ConeProjector::project(const cv::Mat& sonar_image, double sonar_range, double sonar_fov_rad,
                               bool draw_distance_lines)
{
  if (sonar_image.empty())
  {
    return cv::Mat();
  }
  if (sonar_fov_rad <= 0 || sonar_range <= 0)
  {
    // TODO: Does this need to be handled in another way?
    // (probably not, the only problematic point could be the different image dimensions)
    return cv::Mat::zeros(sonar_image.rows, sonar_image.rows, sonar_image.type());
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // If a change is found, re-calucate the lookup table
  if (geometry_.rows != sonar_image.rows || geometry_.columns != sonar_image.cols || geometry_.fov != sonar_fov_rad)
  {
    update_geometry(sonar_image.rows, sonar_image.cols, sonar_fov_rad);
  }

  // Gather every pixel inside the wedge from its source sample
  const cv::Mat source = sonar_image.isContinuous() ? sonar_image : sonar_image.clone();
  cv::Mat cone_image = cv::Mat::zeros(geometry_.size, source.type());
  const size_t pixel_size = source.elemSize();
  // using OpenCV's parallel to concurrently process each rows
  // More info here: https://docs.opencv.org/4.13.0/dc/ddf/tutorial_how_to_use_OpenCV_parallel_for_new.html
  cv::parallel_for_(cv::Range(0, geometry_.size.height), [&](const cv::Range& rows) {
    for (int y = rows.start; y < rows.end; y++)
    {
      const Geometry::RowSpan& span = geometry_.row_spans[y];
      const int32_t* index = geometry_.source_indices.data() + span.first_index;
      uint8_t* out = cone_image.ptr<uint8_t>(y) + span.begin * pixel_size;
      if (pixel_size == 3)
      {
        for (int x = span.begin; x < span.end; x++, index++, out += 3)
        {
          if (*index >= 0)
          {
            const uint8_t* in = source.data + static_cast<size_t>(*index) * 3;
            out[0] = in[0];
            out[1] = in[1];
            out[2] = in[2];
          }
        }
      }
      else
      {
        for (int x = span.begin; x < span.end; x++, index++, out += pixel_size)
        {
          if (*index >= 0)
          {
            std::memcpy(out, source.data + static_cast<size_t>(*index) * pixel_size, pixel_size);
          }
        }
      }
    }
  });

  if (draw_distance_lines)
  {
    // If a change is found, re-calucate the overlay grid
    if (grid_.rows != geometry_.rows || grid_.columns != geometry_.columns || grid_.fov != geometry_.fov ||
        grid_.range != sonar_range || grid_.channels != cone_image.channels())
    {
      update_grid(sonar_range, cone_image.channels());
    }
    // overlay (more like blend) the cached grid over the image
    const int channels = std::min(cone_image.channels(), 3);
    uint8_t* data = cone_image.data;
    for (const GridPixel& grid_pixel : grid_.pixels)
    {
      uint8_t* pixel = data + grid_pixel.offset;
      const int keep = 255 - grid_pixel.alpha;
      for (int c = 0; c < channels; c++)
      {
        pixel[c] = static_cast<uint8_t>((pixel[c] * keep + 127) / 255 + grid_pixel.color[c]);
      }
    }
  }

  return cone_image;
}

// update_geometry creates a lookup table to avoid having to calculate the sample-to-pixel matrix on each frame.
// rows are the distance bins
// columns are the angular beams accros the fov
void ConeProjector::update_geometry(int rows, int columns, double fov)
{
  const double half_fov = fov / 2;
  const double angle_increment = columns > 1 ? fov / (columns - 1) : 0.0;
  // The edge beams and the last range bin cover half a sample beyond their center
  const double angular_extent = std::min(half_fov + angle_increment / 2, std::numbers::pi);
  // image coordinate system is used, so -0.5 actually means extending the radius by 0.5 (half a bin)
  const double radius = rows - 0.5;

  // calculation for the width of the image, dependent on the angular extent:
  // if it is larger than 90 degrees, the edge of the image reaches the full radius width (at exactly 90 degrees) else
  // it calculates the maximum reach
  const int half_width =
      static_cast<int>(std::ceil(radius * (angular_extent >= std::numbers::pi / 2 ? 1.0 : std::sin(angular_extent))));
  const int top = static_cast<int>(std::ceil(radius));
  // the bottom part of the image is also tricky, since larger than 90 degrees of angular_extent lead to extension at
  // the bottom of the image. This is what this calculation cares for with the same rationale as the image's width
  const int bottom =
      angular_extent > std::numbers::pi / 2 ? static_cast<int>(std::ceil(-radius * std::cos(angular_extent))) : 0;

  geometry_.rows = rows;
  geometry_.columns = columns;
  geometry_.fov = fov;
  geometry_.size = cv::Size(2 * half_width + 1, top + bottom + 1);
  geometry_.apex = cv::Point2d(half_width, top);
  geometry_.row_spans.assign(static_cast<size_t>(geometry_.size.height), { 0, 0, 0 });
  geometry_.source_indices.clear();

  // the actual loop in which the lookup table is created
  std::vector<int32_t> row_indices(static_cast<size_t>(geometry_.size.width));
  for (int y = 0; y < geometry_.size.height; y++)
  {
    const double dy = geometry_.apex.y - y;
    int begin = -1, end = -1;
    for (int x = 0; x < geometry_.size.width; x++)
    {
      const double dx = x - geometry_.apex.x;
      const double rho = std::hypot(dx, dy);    // in range bins
      const double theta = std::atan2(dx, dy);  // from straight ahead, positive to the right
      row_indices[x] = -1;
      if (rho > radius || std::abs(theta) > angular_extent)
      {
        continue;
      }
      const long column = angle_increment > 0 ? std::lround((theta + half_fov) / angle_increment) : 0;
      if (column < 0 || column >= columns)
      {
        continue;
      }
      const long row = std::min<long>(std::lround(rho), rows - 1);
      // flatten the (row,column) calculation
      row_indices[x] = static_cast<int32_t>(row * columns + column);
      if (begin < 0)
      {
        begin = x;
      }
      end = x + 1;
    }
    if (begin < 0)
    {
      // discard the empty row
      continue;
    }
    // first and last coordinate
    geometry_.row_spans[y] = { begin, end, geometry_.source_indices.size() };
    geometry_.source_indices.insert(geometry_.source_indices.end(), row_indices.begin() + begin,
                                    row_indices.begin() + end);
  }
}

// update_grid calculates the cool overlay grid
void ConeProjector::update_grid(double sonar_range, int channels)
{
  const Geometry& g = geometry_;
  // overlay is the grid_color
  cv::Mat overlay = cv::Mat::zeros(g.size, CV_8UC3);
  // mask is just black, to be used for antialiasing of the grid
  cv::Mat mask = cv::Mat::zeros(g.size, CV_8UC1);
  // label mask is just black, to be used for antialiasing of the labels
  cv::Mat label_mask = cv::Mat::zeros(g.size, CV_8UC1);
  auto draw = [&](auto&& draw_on) {
    draw_on(overlay, grid_color);
    draw_on(mask, cv::Scalar(255));
  };

  const double pixels_per_meter = g.rows / sonar_range;
  const double radius = g.rows - 0.5;
  const double half_fov = g.fov / 2;
  const double half_fov_deg = half_fov * 180 / std::numbers::pi;
  const cv::Point center = subpixel_point(g.apex.x, g.apex.y);

  // helper function to draw the arcs of the grid
  auto arc = [&](double arc_radius) {
    const int r = static_cast<int>(std::lround(arc_radius * draw_scale));
    draw([&](cv::Mat& image, const cv::Scalar& color) {
      cv::ellipse(image, center, cv::Size(r, r), -90, -half_fov_deg, half_fov_deg, color, 1, cv::LINE_AA, draw_shift);
    });
  };
  // helper function to draw the bearing lines of the grid (apex to edge)
  auto bearing_line = [&](double angle) {
    const cv::Point end = subpixel_point(g.apex.x + radius * std::sin(angle), g.apex.y - radius * std::cos(angle));
    draw([&](cv::Mat& image, const cv::Scalar& color) {
      cv::line(image, center, end, color, 1, cv::LINE_AA, draw_shift);
    });
  };

  // outline arc
  arc(radius);
  // bearing lines at the limits (min and max)
  bearing_line(-half_fov);
  bearing_line(half_fov);
  // generate an internal bearing line every bearing_step
  for (double angle = bearing_step; angle < half_fov - 1e-6; angle += bearing_step)
  {
    bearing_line(-angle);
    bearing_line(angle);
  }
  // add a central line
  if (half_fov > 1e-6)
  {
    bearing_line(0.0);
  }

  // find the distance (step) between rings (arcs)
  const double step = ring_step(sonar_range);
  const double font_scale = 0.4 * std::max(1.0, g.size.height / 600.0);
  for (int k = 1; k * step <= sonar_range * (1 + 1e-9); k++)
  {
    // draw the ring
    const double distance = k * step;
    const double ring_radius = distance * pixels_per_meter;
    arc(ring_radius);

    // draw the label for this ring
    const std::string label = distance_label(distance);
    int baseline = 0;
    const cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, font_scale, 1, &baseline);
    const cv::Point origin(static_cast<int>(g.apex.x + ring_radius * std::sin(half_fov)) - text_size.width - 4,
                           static_cast<int>(g.apex.y - ring_radius * std::cos(half_fov)) - 4);
    // create a dark background first, so the label stays readable
    cv::putText(overlay, label, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar::all(0), 3, cv::LINE_AA);
    cv::putText(label_mask, label, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255), 3, cv::LINE_AA);
    cv::putText(overlay, label, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale, grid_color, 1, cv::LINE_AA);
  }

  // keep only the grid pixels (with opacity) to be applied as an overlay
  grid_.pixels.clear();
  for (int y = 0; y < g.size.height; y++)
  {
    const uint8_t* mask_row = mask.ptr<uint8_t>(y);
    const uint8_t* label_mask_row = label_mask.ptr<uint8_t>(y);
    const cv::Vec3b* overlay_row = overlay.ptr<cv::Vec3b>(y);
    for (int x = 0; x < g.size.width; x++)
    {
      if (mask_row[x] == 0 && label_mask_row[x] == 0)
      {
        continue;
      }
      // labels are less transparent than the lines
      const bool is_label = label_mask_row[x] > 0;
      const double opacity = is_label ? label_opacity : grid_opacity;
      GridPixel pixel;
      pixel.offset = (y * g.size.width + x) * channels;
      pixel.alpha = static_cast<uint8_t>(std::lround((is_label ? label_mask_row[x] : mask_row[x]) * opacity));
      const cv::Vec3b& bgr = overlay_row[x];
      if (channels >= 3)
      {
        for (int c = 0; c < 3; c++)
        {
          pixel.color[c] = static_cast<uint8_t>(std::lround(bgr[c] * opacity));
        }
      }
      else
      {
        const double gray = 0.114 * bgr[0] + 0.587 * bgr[1] + 0.299 * bgr[2];
        pixel.color[0] = pixel.color[1] = pixel.color[2] = static_cast<uint8_t>(std::lround(gray * opacity));
      }
      grid_.pixels.push_back(pixel);
    }
  }
  grid_.rows = g.rows;
  grid_.columns = g.columns;
  grid_.fov = g.fov;
  grid_.range = sonar_range;
  grid_.channels = channels;
}

}  // namespace sonoptix_echo_driver_ros
