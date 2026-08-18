#pragma once

#include <string>

namespace full_self_driving::adapters
{

class HardwareCameraAdapter
{
public:
  HardwareCameraAdapter(
    std::string device_path = "/dev/video0",
    std::string driver = "v4l2",
    int width = 1280,
    int height = 720,
    int framerate_hz = 30,
    std::string pixel_format = "YUYV",
    std::string calibration_file = "",
    std::string calibration_sha256 = "");

  virtual ~HardwareCameraAdapter() = default;

  std::string get_adapter_id() const;
  bool is_capturing() const;
  bool is_healthy() const;
  std::string get_device_path() const;
  std::string get_driver() const;
  int get_width() const;
  int get_height() const;
  int get_framerate_hz() const;
  std::string get_pixel_format() const;
  std::string get_calibration_file() const;
  std::string get_calibration_sha256() const;

private:
  std::string adapter_id_{"v4l2_hardware_camera"};
  std::string device_path_;
  std::string driver_;
  int width_;
  int height_;
  int framerate_hz_;
  std::string pixel_format_;
  std::string calibration_file_;
  std::string calibration_sha256_;
  bool capturing_{false};
  bool healthy_{false};
};

}  // namespace full_self_driving::adapters
