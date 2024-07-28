#ifndef SV_PANDAR_SDK_H_
#define SV_PANDAR_SDK_H_

#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "sv_pandar_types.h"

namespace sv {
namespace lidar {

class SVPandarCoreSDK;

class SVPandarSDK {
 public:
  SVPandarSDK(
      const std::string &device_ip, const uint16_t lidar_port, const uint16_t start_angle,
      const std::function<void(const SVPointCloud_t &)> &callback,
      const std::string &correction_path, const std::string &firetimes_path);

  ~SVPandarSDK();

  void Start();
  void Stop();

 private:
  void GetCalibration();

 private:
  std::unique_ptr<SVPandarCoreSDK> pandar_core_;

  bool enable_get_calibration_;
  std::thread *get_calibration_thread_;
  bool is_calibration_available_;

  std::string correction_path_;
  std::string firetimes_path_;
};

}  // namespace lidar
}  // namespace sv

#endif  // SV_PANDAR_SDK_H_
