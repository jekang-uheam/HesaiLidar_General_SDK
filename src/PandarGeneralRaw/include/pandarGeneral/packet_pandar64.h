#ifndef SV_HESAI_LIDAR_PACKET_PANDAR_64CH_H_
#define SV_HESAI_LIDAR_PACKET_PANDAR_64CH_H_

#include "packet.h"

/**
 * Pandar 64ch
 */

// Head
#define HS_LIDAR_64CH_HEADER_SIZE 8

// Body
#define HS_LIDAR_64CH_AZIMUTH_SIZE 2
#define HS_LIDAR_64CH_UNIT_NUM 64
#define HS_LIDAR_64CH_UNIT_SIZE 3
#define HS_LIDAR_64CH_BLOCK_NUM 6

// Tail
#define HS_LIDAR_64H_TAIL_INFO_SIZE 22

#define HS_LIDAR_64CH_PACKET_SIZE (HS_LIDAR_64CH_HEADER_SIZE + (HS_LIDAR_64CH_BLOCK_NUM * (HS_LIDAR_64CH_AZIMUTH_SIZE + (HS_LIDAR_64CH_UNIT_NUM * HS_LIDAR_64CH_UNIT_SIZE))) + HS_LIDAR_64H_TAIL_INFO_SIZE)

typedef struct HS_LIDAR_64CH_HEADER_s {
  static const uint16_t kDelimiter = 0xffee;

  uint16_t delimiter_;  // 0xFFEE 2bytes
  uint8_t laser_num_;   // laser number 1byte
  uint8_t block_num_;   // block number 1byte
  uint8_t reserved_1_;
  uint8_t dist_unit_;  // Distance unit, 6mm/5mm/4mm
  uint8_t reserved_2_;
  uint8_t reserved_3_;

  uint8_t GetLaserNum() const { return laser_num_; }
  uint8_t GetBlockNum() const { return block_num_; }
  double GetDistUnit() const { return dist_unit_ / 1000.f; }

  void Print() const {
    printf("HS_LIDAR_64CH_HEADER:\n  LaserNum:%02u, BlockNum:%02u, DistUnit:%g\n",
           GetLaserNum(), GetBlockNum(), GetDistUnit());
  }

} __attribute__((__packed__)) HS_LIDAR_64CH_HEADER_t;

typedef struct HS_LIDAR_64CH_UNIT_AZIMUTH_s {
  uint16_t azimuth_;

  uint16_t GetAzimuth() const { return LittleToNative(azimuth_); }

  void Print() const {
    printf("HS_LIDAR_64CH_UNIT_AZIMUTH\n  Azimuth:%u\n", GetAzimuth());
  }
} __attribute__((__packed__)) HS_LIDAR_64CH_UNIT_AZIMUTH_t;
typedef struct HS_LIDAR_64CH_UNIT_BLOCK_s {
  uint16_t distance_;
  uint8_t reflectivity_;

  uint16_t GetDistance() const { return LittleToNative(distance_); }
  uint8_t GetReflectivity() const { return reflectivity_; }

  void Print() const {
    printf("HS_LIDAR_64CH_UNIT_BLOCK:\n  Distance: %u, Reflectivity: %u\n",
           GetDistance(), GetReflectivity());
  }
} __attribute__((__packed__)) HS_LIDAR_64CH_UNIT_BLOCK_t;

typedef struct HS_LIDAR_64CH_BLOCK_s {
  uint16_t azimuth_;  // packet angle, Azimuth = RealAzimuth * 100
  HS_LIDAR_64CH_UNIT_BLOCK_t units_[HS_LIDAR_64CH_UNIT_NUM];
} __attribute__((__packed__)) HS_LIDAR_64CH_BLOCK_t;

typedef struct HS_LIDAR_64CH_TAIL_INFO_s {
  // Temperature
  static const uint8_t kHighTemperature = 0x01;
  static const uint8_t kNormalTemperature = 0x00;

  // return mode
  static const uint8_t kStrongestReturn = 0x37;
  static const uint8_t kLastReturn = 0x38;
  static const uint8_t kDualReturn = 0x39;

  uint8_t reserved_1_;
  uint8_t reserved_2_;
  uint8_t reserved_3_;
  uint8_t reserved_4_;
  uint8_t reserved_5_;
  uint8_t shutdown_flag_;
  uint8_t reserved_6_;
  uint8_t reserved_7_;
  uint16_t motor_speed_;
  uint32_t timestamp_;
  uint8_t return_mode_;
  uint8_t factory_info_;
  uint8_t utc_[6];

  bool ShutdownFlag() const { return shutdown_flag_ == kHighTemperature; }

  uint16_t GetMotorSpeed() const { return LittleToNative(motor_speed_); }

  uint32_t GetTimestamp() const { return LittleToNative(timestamp_); }

  uint8_t GetReturnMode() const { return return_mode_; }
  bool IsLastReturn() const { return return_mode_ == kLastReturn; }
  bool IsStrongestReturn() const { return return_mode_ == kStrongestReturn; }
  bool IsDualReturn() const { return return_mode_ == kDualReturn; }

  uint8_t GetFactoryInfo() const { return factory_info_; }

  uint8_t GetUTCData(uint8_t index) const { return utc_[index < sizeof(utc_) ? index : 0]; }
  int64_t GetMicroLidarTimeU64() const {
    if (utc_[0] != 0) {
      struct tm t = {0};
      t.tm_year = utc_[0];
      if (t.tm_year < 70) {
        return 0;
      }
      t.tm_mon = utc_[1] - 1;
      t.tm_mday = utc_[2] + 1;
      t.tm_hour = utc_[3];
      t.tm_min = utc_[4];
      t.tm_sec = utc_[5];
      t.tm_isdst = 0;
      return (mktime(&t) - timezone - 86400) * 1000000 + GetTimestamp();
    } else {
      uint32_t utc_time_big = *(uint32_t *)(&utc_[0] + 2);
      int64_t unix_second = ((utc_time_big >> 24) & 0xff) |
                            ((utc_time_big >> 8) & 0xff00) |
                            ((utc_time_big << 8) & 0xff0000) |
                            ((utc_time_big << 24));
      return unix_second * 1000000 + GetTimestamp();
    }
  }
  double GetTimestampPoint() const {
    struct tm tTm = {0};
    // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
    // and mktime's year start from 1900 which is 0. so we need add 100 year.
    tTm.tm_year = utc_[0] + 100;

    // in case of time error
    if (tTm.tm_year >= 200) {
      tTm.tm_year -= 100;
    }

    // UTC's month start from 1, but mktime only accept month from 0.
    tTm.tm_mon = utc_[1] - 1;
    tTm.tm_mday = utc_[2];
    tTm.tm_hour = utc_[3];
    tTm.tm_min = utc_[4];
    tTm.tm_sec = utc_[5];
    tTm.tm_isdst = 0;
    return mktime(&tTm) + static_cast<double>(timestamp_) / 1000000.0;
  }

  void Print() const {
    printf("HS_LIDAR_64CH_TAIL_INFO:\n  ShutdownFlag:%d, MotorSpeed:%u, Timestamp:%u, ReturnMode:0x%02x, FactoryInfo:0x%02x, utc:%u %u %u %u %u %u\n",
           ShutdownFlag(), GetMotorSpeed(), GetTimestamp(), GetReturnMode(), GetFactoryInfo(), GetUTCData(0), GetUTCData(1), GetUTCData(2), GetUTCData(3), GetUTCData(4), GetUTCData(5));
  }
} __attribute__((__packed__)) HS_LIDAR_64CH_TAIL_INFO_t;

typedef struct HS_LIDAR_64CH_PACKET_s {
  // return mode
  static const uint8_t kStrongestReturn = 0x37;
  static const uint8_t kLastReturn = 0x38;
  static const uint8_t kDualReturn = 0x39;

  uint8_t block_num_;
  uint8_t laser_num_;
  uint8_t dist_unit_;

  HS_LIDAR_64CH_BLOCK_t blocks_[HS_LIDAR_64CH_BLOCK_NUM];

  bool shutdown_flag_;
  uint16_t motor_speed_;
  uint32_t timestamp_;
  uint8_t return_mode_;
  uint8_t factory_info_;
  double timestamp_point_;


  void set_timestamp_point(double timestamp_point) { timestamp_point_ = timestamp_point;}
double timestamp_point() const {return timestamp_point_;}

  uint8_t GetReturnMode() const { return return_mode_; }
  bool IsLastReturn() const { return return_mode_ == kLastReturn; }
  bool IsStrongestReturn() const { return return_mode_ == kStrongestReturn; }
  bool IsDualReturn() const { return return_mode_ == kDualReturn; }
} __attribute__((__packed__)) HS_LIDAR_64CH_PACKET_t;

const float pandar_64ch_elev_angle_map[] = {
    14.708f, 10.858f, 7.885f, 4.883f, 2.866f, 1.854f, 1.686f, 1.514f,
    1.348f, 1.177f, 1.01f, 0.839f, 0.672f, 0.501f, 0.334f, 0.163f,
    -0.005f, -0.174f, -0.343f, -0.511f, -0.682f, -0.849f, -1.019f, -1.187f,
    -1.358f, -1.525f, -1.696f, -1.862f, -2.034f, -2.202f, -2.372f, -2.539f,
    -2.71f, -2.874f, -3.047f, -3.214f, -3.384f, -3.549f, -3.722f, -3.886f,
    -4.058f, -4.224f, -4.395f, -4.559f, -4.732f, -4.894f, -5.066f, -5.231f,
    -5.403f, -5.565f, -5.739f, -5.9f, -6.072f, -6.235f, -7.237f, -8.233f,
    -9.234f, -10.059f, -11.206f, -12.18f, -13.148f, -14.104f, -19.063f, -25.071f};

const float pandar_64ch_horizatal_azimuth_offset_map[] = {
    -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, 1.042f, 3.125f,
    5.208f, -5.208f, -3.125f, -1.042f, 1.042f, 3.125f, 5.208f, -5.208f,
    -3.125f, -1.042f, 1.042f, 3.125f, 5.208f, -5.208f, -3.125f, -1.042f,
    1.042f, 3.125f, 5.208f, -5.208f, -3.125f, -1.042f, 1.042f, 3.125f,
    5.208f, -5.208f, -3.125f, -1.042f, 1.042f, 3.125f, 5.208f, -5.208f,
    -3.125f, -1.042f, 1.042f, 3.125f, 5.208f, -5.208f, -3.125f, -1.042f,
    1.042f, 3.125f, 5.208f, -5.208f, -3.125f, -1.042f, -1.042f, -1.042f,
    -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f};

#endif  // SV_HESAI_LIDAR_PACKET_PANDAR_64CH_H_