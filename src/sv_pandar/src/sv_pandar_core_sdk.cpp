
#include "sv_pandar_core_sdk.h"

#include <memory>
#include <string>
#include <vector>

namespace sv {
namespace lidar {

SVPandarCoreSDK::SVPandarCoreSDK(const std::string &device_ip, const uint16_t lidar_port, const uint16_t start_angle,
                                 const std::function<void(const SVPointCloud_t &)> &callback,
                                 const std::string &correction_path, const std::string &firetimes_path) {
}

SVPandarCoreSDK ::~SVPandarCoreSDK() {
}

void SVPandarCoreSDK::Start() {
}
void SVPandarCoreSDK::Stop() {
}

void SVPandarCoreSDK::GetCalibration() {
}

void SVPandarCoreSDK::LoadCorrection(std::string &correction_content) {
}
void SVPandarCoreSDK::LoadFiretimes(std::string &firetimes_content) {
}

}  // namespace lidar
}  // namespace sv
