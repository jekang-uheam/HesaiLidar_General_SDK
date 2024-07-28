#ifndef SV_PANDAR_TYPES_H_
#define SV_PANDAR_TYPES_H_

#include <memory>
#include <vector>

namespace sv {
namespace lidar {

typedef struct SVPoint_s {
  float x;
  float y;
  float z;
  float intensity;
  double timestamp;
  uint16_t ring;
  uint16_t angle;
} __attribute__((__packed__)) SVPoint_t;

typedef struct SVPointCloud_s {
  uint16_t return_mode_;
  uint16_t spin_speed_;
  uint8_t work_mode_;

  uint32_t points_num_;
  std::vector<SVPoint_t> points_;

} SVPointCloud_t;

}  // namespace lidar
}  // namespace sv

#endif  // SV_PANDAR_TYPES_H_