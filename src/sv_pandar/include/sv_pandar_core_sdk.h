#ifndef SV_PANDAR_CORE_SDK_H_
#define SV_PANDAR_CORE_SDK_H_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "sv_pandar_types.h"

namespace sv {
namespace lidar {

class SVPandarCoreSDK {
 public:
  SVPandarCoreSDK(const std::string &device_ip, const uint16_t lidar_port, const uint16_t start_angle,
                  const std::function<void(const SVPointCloud_t &)> &callback,
                  const std::string &correction_path, const std::string &firetimes_path);

  ~SVPandarCoreSDK();

  void Start();
  void Stop();

  void GetCalibration();

 private:
  void LoadCorrection(std::string &correction_content);
  void LoadFiretimes(std::string &firetimes_content);
};

}  // namespace lidar
}  // namespace sv

#endif  // SV_PANDAR_CORE_SDK_H_