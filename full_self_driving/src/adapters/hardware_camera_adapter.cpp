#include "hardware_camera_adapter.hpp"

namespace full_self_driving::adapters
{

HardwareCameraAdapter::HardwareCameraAdapter(
  std::string device_path,
  std::string driver,
  int width,
  int height,
  int framerate_hz,
  std::string pixel_format,
  std::string calibration_file,
  std::string calibration_sha256)
: device_path_(std::move(device_path)),
  driver_(std::move(driver)),
  width_(width),
  height_(height),
  framerate_hz_(framerate_hz),
  pixel_format_(std::move(pixel_format)),
  calibration_file_(std::move(calibration_file)),
  calibration_sha256_(std::move(calibration_sha256)),
  capturing_(false),
  healthy_(false)
{
}

std::string HardwareCameraAdapter::get_adapter_id() const
{
  return adapter_id_;
}

bool HardwareCameraAdapter::is_capturing() const
{
  return capturing_;
}

bool HardwareCameraAdapter::is_healthy() const
{
  return healthy_;
}

std::string HardwareCameraAdapter::get_device_path() const
{
  return device_path_;
}

std::string HardwareCameraAdapter::get_driver() const
{
  return driver_;
}

int HardwareCameraAdapter::get_width() const
{
  return width_;
}

int HardwareCameraAdapter::get_height() const
{
  return height_;
}

int HardwareCameraAdapter::get_framerate_hz() const
{
  return framerate_hz_;
}

std::string HardwareCameraAdapter::get_pixel_format() const
{
  return pixel_format_;
}

std::string HardwareCameraAdapter::get_calibration_file() const
{
  return calibration_file_;
}

std::string HardwareCameraAdapter::get_calibration_sha256() const
{
  return calibration_sha256_;
}

}  // namespace full_self_driving::adapters
