#ifndef SV_HESAI_LIDAR_PACKET_PANDAR_128CH_H_
#define SV_HESAI_LIDAR_PACKET_PANDAR_128CH_H_

#include "packet.h"

/**
 * Pandar 128ch
 */

// Head
#define HS_LIDAR_128CH_PRE_HEADER_SIZE 6
#define HS_LIDAR_128CH_HEADER_SIZE 6

// Body
#define HS_LIDAR_128CH_AZIMUTH_SIZE 2
#define HS_LIDAR_128CH_UNIT_SIZE 3
#define HS_LIDAR_128CH_UNIT_NUM 128
#define HS_LIDAR_128CH_BLOCK_NUM 2
#define HS_LIDAR_128CH_BODY_CRC_SIZE 4

// Func Safty
#define HS_LIDAR_128CH_FUNC_SAFETY_SIZE 17

// Tail
#define HS_LIDAR_128CH_TAIL_INFO_SIZE 26
#define HS_LIDAR_128CH_TAIL_SEQ_NUM_SIZE 4
#define HS_LIDAR_128CH_TAIL_IMU_SIZE 22
#define HS_LIDAR_128CH_TAIL_CRC_SIZE 4

#define HS_LIDAR_128CH_PACKET_SIZE (HS_LIDAR_128CH_PRE_HEADER_SIZE + HS_LIDAR_128CH_HEADER_SIZE + (HS_LIDAR_128CH_BLOCK_NUM * (HS_LIDAR_128CH_AZIMUTH_SIZE + (HS_LIDAR_128CH_UNIT_NUM * HS_LIDAR_128CH_UNIT_SIZE))) + HS_LIDAR_128CH_BODY_CRC_SIZE + HS_LIDAR_128CH_FUNC_SAFETY_SIZE + HS_LIDAR_128CH_TAIL_INFO_SIZE + HS_LIDAR_128CH_TAIL_SEQ_NUM_SIZE + HS_LIDAR_128CH_TAIL_IMU_SIZE + HS_LIDAR_128CH_TAIL_CRC_SIZE)

typedef struct HS_LIDAR_128CH_PRE_HEADER_s {
  static const uint16_t kDelimiter = 0xffee;
  // major version
  static const uint8_t kME = 0x01;  // mechanical lidar
  static const uint8_t kQT = 0x03;
  static const uint8_t kST = 0x04;
  static const uint8_t kFT = 0x07;
  static const uint8_t kXT = 0x06;

  // minor version
  static const uint8_t kV1 = 0x01;  // reserved
  static const uint8_t kV2 = 0x02;  // used by P128 series / POROS
  static const uint8_t kV3 = 0x03;  // used by P128 series
  static const uint8_t kV4 = 0x04;  // used by P128 series

  // status info version
  static const uint8_t kStatusInfoV0 = 0;

  uint16_t delimiter_;
  uint8_t version_major_;
  uint8_t version_minor_;
  uint8_t status_info_version_;
  uint8_t reserved_1_;

  bool IsValidDelimiter() const {
    return LittleToNative(delimiter_) == kDelimiter;
  }
  uint8_t GetDelimiter() const { return LittleToNative(delimiter_); }
  uint8_t GetVersionMajor() const { return version_major_; }
  uint8_t GetVersionMinor() const { return version_minor_; }
  uint8_t GetStatusInfoVersion() const { return status_info_version_; }

  void Init(uint8_t version_major, uint8_t version_minor, uint8_t status_info_version = kStatusInfoV0) {
    delimiter_ = kDelimiter;
    version_major_ = version_major;
    version_minor_ = version_minor;
    status_info_version_ = status_info_version;
  }

  void Print() const {
    printf("HS_LIDAR_128CH_PRE_HEADER:\n  Delimiter:%02x, valid:%d, Ver Major: %02x, minor: %02x, StatusInfoVer:%02x\n",
           GetDelimiter(), IsValidDelimiter(), GetVersionMajor(), GetVersionMinor(), GetStatusInfoVersion());
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_PRE_HEADER_t;
typedef struct HS_LIDAR_128CH_HEADER_s {
  static const uint8_t kSequenceNum = 0x01;
  static const uint8_t kIMU = 0x02;
  static const uint8_t kFunctionSafety = 0x04;
  static const uint8_t kCyberSecurity = 0x08;
  static const uint8_t kConfidenceLevel = 0x10;
  static const uint8_t kDistUnit = 0x04;

  static const uint8_t kSingleReturn = 0x00;
  static const uint8_t kFirstBlockLastReturn = 0x01;
  static const uint8_t kFirstBlockStrongestReturn = 0x02;

  uint8_t laser_num_;
  uint8_t block_num_;
  uint8_t echo_count_;
  uint8_t dist_unit_;
  uint8_t echo_num_;
  uint8_t status_;

  uint8_t GetLaserNum() const { return laser_num_; }
  uint8_t GetBlockNum() const { return block_num_; }
  double GetDistUnit() const { return dist_unit_ / 1000.f; }
  uint8_t GetEchoCount() const { return echo_count_; }
  uint8_t GetEchoNum() const { return echo_num_; }
  bool HasSeqNum() const { return status_ & kSequenceNum; }
  bool HasIMU() const { return status_ & kIMU; }
  bool HasFuncSafety() const { return status_ & kFunctionSafety; }
  bool HasCyberSecurity() const { return status_ & kCyberSecurity; }
  bool HasConfidenceLevel() const { return status_ & kConfidenceLevel; }
  bool IsFirstBlockLastReturn() const { return echo_count_ == kFirstBlockLastReturn; }
  bool IsFirstBlockStrongestReturn() const { return echo_count_ == kFirstBlockStrongestReturn; }

  uint16_t GetPacketSize() const {
    return HS_LIDAR_128CH_PRE_HEADER_SIZE + HS_LIDAR_128CH_HEADER_SIZE +
           (GetBlockNum() * (HS_LIDAR_128CH_AZIMUTH_SIZE + (GetLaserNum() * HS_LIDAR_128CH_UNIT_SIZE))) +
           HS_LIDAR_128CH_BODY_CRC_SIZE +
           HS_LIDAR_128CH_FUNC_SAFETY_SIZE +
           HS_LIDAR_128CH_TAIL_INFO_SIZE +
           HS_LIDAR_128CH_TAIL_SEQ_NUM_SIZE +
           HS_LIDAR_128CH_TAIL_IMU_SIZE +
           HS_LIDAR_128CH_TAIL_CRC_SIZE;
  }

  void Print() const {
    printf("HS_LIDAR_128CH_HEADER:\n  LaserNum:%02u, BlockNum:%02u, DistUnit:%g, EchoCnt:%02u, EchoNum:%02u, HasSeqNum:%d, HasIMU:%d, HasFuncSafety:%d, HasCyberSecurity:%d, HasConfidence:%d\n",
           GetLaserNum(), GetBlockNum(), GetDistUnit(), GetEchoCount(), GetEchoNum(), HasSeqNum(), HasIMU(), HasFuncSafety(), HasCyberSecurity(), HasConfidenceLevel());
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_HEADER_t;

typedef struct HS_LIDAR_128CH_UNIT_AZIMUTH_s {
  uint16_t azimuth_;

  uint16_t GetAzimuth() const { return LittleToNative(azimuth_); }

  void Print() const {
    printf("HS_LIDAR_128CH_UNIT_AZIMUTH\n  Azimuth:%u\n", GetAzimuth());
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_UNIT_AZIMUTH_t;


typedef struct HS_LIDAR_128CH_BODY_CRC_s {
  uint32_t crc_;

  uint32_t GetCrc() const { return LittleToNative(crc_); }

  void Print() const {
    printf("HS_LIDAR_128CH_BODY_CRC:\n  crc:0x%08x\n", GetCrc());
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_BODY_CRC_t;

typedef struct HS_LIDAR_128CH_FUNC_SAFETY_s {
  static const uint8_t kLidarStateShiftBits = 5;
  static const uint8_t kLidarStateMask = 0x07;
  static const uint8_t kRollingCntShiftBits = 0;
  static const uint8_t kRollingCntMask = 0x07;
  static const uint8_t kFaultTypeCurrent = 0x08;
  static const uint8_t kFaultTypeHistory = 0x10;
  static const uint8_t kFaultNumShiftBits = 4;
  static const uint8_t kFaultNumMask = 0x0F;
  static const uint8_t kFaultIDShiftBits = 0;
  static const uint8_t kFaultIDMask = 0x0F;

  uint8_t version_;
  uint8_t status_;
  uint8_t fault_info_;
  uint16_t fault_code_;
  uint8_t reserved_[8];
  uint32_t crc_;

  uint8_t GetVersion() const { return version_; }

  uint8_t GetLidarState() const { return (status_ >> kLidarStateShiftBits) & kLidarStateMask; }
  bool IsHistoryFault() const { return status_ & kFaultTypeHistory; }
  bool IsCurrentFault() const { return status_ & kFaultTypeCurrent; }
  uint8_t GetRollingCnt() const { return (status_ >> kRollingCntShiftBits) & kRollingCntMask; }

  uint8_t GetFaultNum() const { return (fault_info_ >> kFaultNumShiftBits) & kFaultNumMask; }
  uint8_t GetFaultID() const { return (fault_info_ >> kFaultIDShiftBits) & kFaultIDMask; }

  uint16_t GetFaultCode() const { return LittleToNative(fault_code_); }
  uint32_t GetCrc() const { return LittleToNative(crc_); }

  void Print() const {
    printf("HS_LIDAR_128CH_FUNC_SAFETY:\n  Version:%u, LidarState:%u, RollingCnt:%u, IsCurrentFault:%u, FaultNum:%u, FaultID:%u, FaultCode:0x%04x, Crc:0x%08x\n",
           GetVersion(), GetLidarState(), GetRollingCnt(), IsCurrentFault(), GetFaultNum(), GetFaultID(), GetFaultCode(), GetCrc());
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_FUNC_SAFETY_t;

typedef struct HS_LIDAR_128CH_TAIL_INFO_s {
  // shutdown flag, bit 0
  static const uint8_t kHighPerformanceMode = 0x00;
  static const uint8_t kShutdown = 0x01;
  static const uint8_t kStandardMode = 0x02;
  static const uint8_t kEnergySavingMode = 0x03;

  // return mode
  static const uint8_t kFirstReturn = 0x33;
  static const uint8_t kSecondReturn = 0x34;
  static const uint8_t kThirdReturn = 0x35;
  static const uint8_t kFourthReturn = 0x36;
  static const uint8_t kStrongestReturn = 0x37;
  static const uint8_t kLastReturn = 0x38;
  static const uint8_t kDualReturn = 0x39;
  static const uint8_t kFirstSecondReturn = 0x3a;
  static const uint8_t kStongestFirstReturn = 0x3b;

  ReservedInfo1_t reserved_info_1_;
  ReservedInfo2_t reserved_info_2_;
  ReservedInfo3_t reserved_info_3_;
  uint16_t azimuth_state_;
  uint8_t running_mode_;
  uint8_t return_mode_;
  uint16_t motor_speed_;
  uint8_t utc_[6];
  uint32_t timestamp_;
  uint8_t factory_info_;

  uint8_t GetStsID0() const { return reserved_info_1_.GetID(); }
  uint16_t GetData0() const { return reserved_info_1_.GetData(); }
  uint8_t GetStsID1() const { return reserved_info_2_.GetID(); }
  uint16_t GetData1() const { return reserved_info_2_.GetData(); }
  uint8_t GetStsID2() const { return reserved_info_3_.GetID(); }
  uint16_t GetData2() const { return reserved_info_3_.GetData(); }

  uint8_t HasShutdown() const { return running_mode_ == kShutdown; }
  uint8_t GetReturnMode() const { return return_mode_; }
  uint16_t GetMotorSpeed() const { return LittleToNative(motor_speed_); }
  uint32_t GetTimestamp() const { return LittleToNative(timestamp_); }
  uint8_t GetOperationMode() const { return running_mode_ & 0x0f; }
  uint8_t GetAngleState(int block_index) const { return (azimuth_state_ >> (2 * (7 - block_index))) & 0x03; }  // Block1(index:0)[15:14], Block2(index:1) [13:12]

  bool IsFirstReturn() const { return return_mode_ == kFirstReturn; }
  bool IsSecondReturn() const { return return_mode_ == kSecondReturn; }
  bool IsThirdReturn() const { return return_mode_ == kThirdReturn; }
  bool IsFourthReturn() const { return return_mode_ == kFourthReturn; }
  bool IsLastReturn() const { return return_mode_ == kLastReturn; }
  bool IsStrongestReturn() const { return return_mode_ == kStrongestReturn; }
  bool IsDualReturn() const { return return_mode_ == kDualReturn; }
  bool IsFirstSecondReturn() const { return return_mode_ == kFirstSecondReturn; }
  bool IsStongestFirstReturn() const { return return_mode_ == kStongestFirstReturn; }
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

  uint8_t GetFactoryInfo() const { return factory_info_; }
  uint8_t GetUTCData(uint8_t index) const {
    return utc_[index < sizeof(utc_) ? index : 0];
  }

  void Print() const {
    printf("HS_LIDAR_128CH_TAIL_INFO:\n  StsID0:%d, Data0:%d, StsID1:%d, Data1:%d, StsID2:%d, Data2:%d, HasShutdown:%d, MotorSpeed:%u, Timestamp:%u, ReturnMode:0x%02x, FactoryInfo:0x%02x, utc:%u %u %u %u %u %u\n",
           GetStsID0(), GetData0(), GetStsID1(), GetData1(), GetStsID2(), GetData2(), HasShutdown(), GetMotorSpeed(), GetTimestamp(), GetReturnMode(), GetFactoryInfo(), GetUTCData(0), GetUTCData(1), GetUTCData(2), GetUTCData(3), GetUTCData(4), GetUTCData(5));
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_TAIL_INFO_t;
typedef struct HS_LIDAR_128CH_TAIL_SEQ_NUM_s {
  uint32_t seq_num_;

  uint32_t GetSeqNum() const { return LittleToNative(seq_num_); }
  static uint32_t GetSeqNumSize() { return sizeof(seq_num_); }

  void CalPktLoss(uint32_t &start_seq_num, uint32_t &last_seq_num, uint32_t &loss_count, uint32_t &start_time, uint32_t &total_loss_count, uint32_t &total_start_seq_num) const {
    if (start_seq_num == 0) {
      loss_count = 0;
      total_loss_count = 0;
      start_time = GetMicroTickCount();
      start_seq_num = seq_num_;
      last_seq_num = seq_num_;
      total_start_seq_num = seq_num_;
      return;
    }
    if (seq_num_ - last_seq_num > 1) {
      loss_count += (seq_num_ - last_seq_num - 1);
      total_loss_count += (seq_num_ - last_seq_num - 1);
    }

    // print log every 1s
    if (GetMicroTickCount() - start_time >= 1 * 1000 * 1000) {
      printf("pkt loss freq: %u/%u\n", loss_count,
             seq_num_ - start_seq_num);
      loss_count = 0;
      start_time = GetMicroTickCount();
      start_seq_num = seq_num_;
    }

    last_seq_num = seq_num_;
  }

  void CalPktLoss(uint32_t &start_seq_num, uint32_t &last_seq_num, uint32_t &loss_count, uint32_t &start_time) const {
    if (seq_num_ - last_seq_num > 1) {
      loss_count += (seq_num_ - last_seq_num - 1);
    }

    // print log every 1s
    if (GetMicroTickCount() - start_time >= 1 * 1000 * 1000) {
      printf("pkt loss freq: %u/%u\n", loss_count, seq_num_ - start_seq_num);
      loss_count = 0;
      start_time = GetMicroTickCount();
      start_seq_num = seq_num_;
    }

    last_seq_num = seq_num_;
  }

  void Print() const {
    printf("HS_LIDAR_128CH_TAIL_SEQ_NUM:\n  seqNum: %u\n", GetSeqNum());
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_TAIL_SEQ_NUM_t;
typedef struct HS_LIDAR_128CH_TAIL_IMU_s {
  int16_t imu_temperature_;
  uint16_t imu_accel_unit_;
  uint16_t imt_ang_vel_unit_;
  uint32_t imu_timestamp_;
  int16_t imu_x_accel_;
  int16_t imu_y_accel_;
  int16_t imu_z_accel_;
  int16_t imu_x_ang_vel_;
  int16_t imu_y_ang_vel_;
  int16_t imu_z_ang_vel_;

  int16_t GetIMUTemperature() const {
    return LittleToNative(imu_temperature_);
  }
  double GetIMUAccelUnit() const {
    return LittleToNative(imu_accel_unit_) / 1000.f;
  }
  double GetIMUAngVelUnit() const {
    return LittleToNative(imt_ang_vel_unit_) / 100.f;
  }
  uint32_t GetIMUTimestamp() const {
    return LittleToNative(imu_timestamp_);
  }
  double GetIMUXAccel() const {
    return LittleToNative(imu_x_accel_) * GetIMUAccelUnit();
  }
  double GetIMUYAccel() const {
    return LittleToNative(imu_y_accel_) * GetIMUAccelUnit();
  }
  double GetIMUZAccel() const {
    return LittleToNative(imu_z_accel_) * GetIMUAccelUnit();
  }
  double GetIMUXAngVel() const {
    return LittleToNative(imu_x_ang_vel_) * GetIMUAngVelUnit();
  }
  double GetIMUYAngVel() const {
    return LittleToNative(imu_y_ang_vel_) * GetIMUAngVelUnit();
  }
  double GetIMUZAngVel() const {
    return LittleToNative(imu_z_ang_vel_) * GetIMUAngVelUnit();
  }

  void Print() const {
    printf(
        "HS_LIDAR_128CH_TAIL_IMU:\n  Temperature:%u, AccelUnit:%f, AngVelUnit:%f, Timestamp:%u, XAccel:%f, YAccel:%f, ZAccel:%f, XAngVel:%f, YAngVel:%f, ZAngVel:%f\n",
        GetIMUTemperature(), GetIMUAccelUnit(), GetIMUAngVelUnit(), GetIMUTimestamp(), GetIMUXAccel(), GetIMUYAccel(), GetIMUZAccel(), GetIMUXAngVel(), GetIMUYAngVel(), GetIMUZAngVel());
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_TAIL_IMU_t;
typedef struct HS_LIDAR_128CH_TAIL_CRC_s {
  uint32_t crc_;

  uint32_t GetCrc() const { return LittleToNative(crc_); }

  void Print() const {
    printf("HS_LIDAR_128CH_TAIL_CRC:\n  crc:0x%08x\n", GetCrc());
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_TAIL_CRC_t;

typedef struct HS_LIDAR_128CH_UNIT_BLOCK_s {
 private:
  double distance_;
  uint8_t reflectivity_;

 public:
  void set_distance(const double distance) { distance_ = distance; }
  double distance() const { return distance_; }

  void set_reflectivity(const uint8_t reflectivity) { reflectivity_ = reflectivity; }
  uint8_t reflectivity() const { return reflectivity_; }
} __attribute__((__packed__)) HS_LIDAR_128CH_UNIT_BLOCK_t;
typedef struct HS_LIDAR_128CH_BLOCK_s {
 private:
  uint16_t azimuth_;
  HS_LIDAR_128CH_UNIT_BLOCK_t units_[HS_LIDAR_128CH_UNIT_NUM];

 public:
  void set_azimuth(const uint16_t azimuth) { azimuth_ = azimuth; }
  uint16_t azimuth() const { return azimuth_; }

  HS_LIDAR_128CH_UNIT_BLOCK_t* unit(const uint8_t unit_index) { return &units_[unit_index]; }

} __attribute__((__packed__)) HS_LIDAR_128CH_BLOCK_t;

typedef struct HS_LIDAR_128CH_PACKET_s {
 private:
  // return mode
  static const uint8_t kFirstReturn = 0x33;
  static const uint8_t kSecondReturn = 0x34;
  static const uint8_t kThirdReturn = 0x35;
  static const uint8_t kFourthReturn = 0x36;
  static const uint8_t kStrongestReturn = 0x37;
  static const uint8_t kLastReturn = 0x38;
  static const uint8_t kDualReturn = 0x39;
  static const uint8_t kFirstSecondReturn = 0x3a;
  static const uint8_t kStongestFirstReturn = 0x3b;

  // operation mode
  static const uint8_t kHighResolution = 0;
  static const uint8_t kStandard = 2;
  static const uint8_t kEnergySaving = 3;

  uint8_t laser_num_;
  uint8_t block_num_;
  double dist_unit_;

  HS_LIDAR_128CH_BLOCK_t blocks_[HS_LIDAR_128CH_BLOCK_NUM];

  uint16_t motor_speed_;
  int64_t timestamp_;
  uint8_t return_mode_;
  uint8_t operation_mode_;
  uint8_t angle_state_[HS_LIDAR_128CH_BLOCK_NUM];

  int64_t timestamp_point_;

 public:
  void set_laser_num(const uint8_t laser_num) { laser_num_ = laser_num; }
  uint8_t laser_num() const { return laser_num_; }
  void set_block_num(const uint8_t block_num) { block_num_ = block_num; }
  uint8_t block_num() const { return block_num_; }
  void set_dist_unit(const double dist_unit) { dist_unit_ = dist_unit; }
  double dist_unit() const { return dist_unit_; }

  void set_motor_speed(const uint16_t motor_speed) { motor_speed_ = motor_speed; }
  uint16_t motor_speed() const { return motor_speed_; }
  void set_timestamp(const int64_t timestamp) { timestamp_ = timestamp; }
  int64_t timestamp() const { return timestamp_; }

  void set_return_mode(const uint8_t return_mode) { return_mode_ = return_mode; }
  uint8_t return_mode() const { return return_mode_; }

  void set_operation_mode(const uint8_t operation_mode) { operation_mode_ = operation_mode; }
  uint8_t operation_mode() const { return operation_mode_; }

  void set_angle_state(const size_t block_index, const uint8_t angle_state) {
    if (block_index >= HS_LIDAR_128CH_BLOCK_NUM) {
      // error
      return;
    }
    angle_state_[block_index] = angle_state;
  }
  uint8_t angle_state(int block_index) const {
    if (block_index >= HS_LIDAR_128CH_BLOCK_NUM) {
      // error
      return 0;
    }
    return angle_state_[block_index];
  }

  HS_LIDAR_128CH_BLOCK_t* block(const uint8_t block_index) { return &blocks_[block_index]; }

  void set_timestamp_point(double timestamp_point) { timestamp_point_ = timestamp_point;}
double timestamp_point() const {return timestamp_point_;}

  bool IsFirstReturn() const { return return_mode_ == kFirstReturn; }
  bool IsSecondReturn() const { return return_mode_ == kSecondReturn; }
  bool IsThirdReturn() const { return return_mode_ == kThirdReturn; }
  bool IsFourthReturn() const { return return_mode_ == kFourthReturn; }
  bool IsLastReturn() const { return return_mode_ == kLastReturn; }
  bool IsStrongestReturn() const { return return_mode_ == kStrongestReturn; }
  bool IsDualReturn() const { return return_mode_ == kDualReturn; }
  bool IsFirstSecondReturn() const { return return_mode_ == kFirstSecondReturn; }
  bool IsStongestFirstReturn() const { return return_mode_ == kStongestFirstReturn; }

  bool IsHighResolution() const { return operation_mode_ == kHighResolution; }
  bool IsStandard() const { return operation_mode_ == kStandard; }
  bool IsEnergySaving() const { return operation_mode_ == kEnergySaving; }

} __attribute__((__packed__)) HS_LIDAR_128CH_PACKET_t;

const float pandar_128ch_elev_angle_map[] = {
    14.436f, 13.535f, 13.081786f, 12.624f, 12.165246f, 11.702f, 11.238522f, 10.771f,
    10.305007f, 9.83f, 9.356123f, 8.88f, 8.401321f, 7.921f, 7.43808f, 6.952581f,
    6.466905f, 5.977753f, 5.487f, 4.995801f, 4.501f, 4.007293f, 3.509f, 3.012822f,
    2.512f, 2.013f, 1.885f, 1.761f, 1.637f, 1.511f, 1.385875f, 1.2582f,
    1.13f, 1.008459f, 0.88f, 0.756f, 0.63f, 0.505f, 0.378591f, 0.251f,
    0.124f, -0.00015f, -0.129f, -0.2541f, -0.38f, -0.5061259f, -0.63235f, -0.7597898f,
    -0.8872418f, -1.012168f, -1.141f, -1.2662f, -1.393f, -1.519337f, -1.646275f, -1.773301f,
    -1.900587f, -2.026912f, -2.155f, -2.2815f, -2.409f, -2.534932f, -2.662501f, -2.789024f,
    -2.916055f, -3.043698f, -3.172f, -3.299f, -3.425f, -3.552222f, -3.680335f, -3.806265f,
    -3.932954f, -4.06183f, -4.19f, -4.318f, -4.444f, -4.570508f, -4.699079f, -4.824327f,
    -4.950584f, -5.080608f, -5.209f, -5.336f, -5.463f, -5.589088f, -5.718031f, -5.842508f,
    -5.968246f, -6.099661f, -6.607262f, -7.117295f, -7.624327f, -8.133802f, -8.639587f, -9.149f,
    -9.652353f, -10.16f, -10.665443f, -11.17f, -11.671568f, -12.174f, -12.673194f, -13.173f,
    -13.669682f, -14.166f, -14.660411f, -15.154f, -15.644783f, -16.135f, -16.622221f, -17.106088f,
    -17.592171f, -18.071976f, -18.54765f, -19.029597f, -19.50071f, -19.978461f, -20.44479f, -20.918108f,
    -21.37943f, -21.848107f, -22.30422f, -22.768055f, -23.21878f, -23.677577f, -24.12274f, -25.01577f};

const float pandar_128ch_horizatal_azimuth_offset_map[] = {
    3.257f, 3.263f, 1.091414f, 3.268f, 1.092504f, 3.273f, 1.093621f, 3.278f,
    1.094766f, 3.283f, 1.095941f, 3.288f, 1.097146f, 3.291f, 1.098384f, -1.101114f,
    1.099655f, -1.103649f, -3.306f, -1.106118f, -3.311f, -1.108519f, -3.318f, -1.110852f,
    -3.324f, -1.113115f, 7.72f, 5.535f, 3.325f, -3.33f, 1.10673f, -5.538034933f,
    -7.726107248f, -1.115309f, 7.731f, 5.543f, 3.329f, -3.336f, 1.108227f, -5.546813133f,
    -7.738214933f, -1.117431f, 7.743f, 5.550783f, 3.335f, -3.341759f, 1.109762f, -5.555361f,
    -7.750039f, -1.119482f, 7.757f, 5.559905f, 3.34f, -3.347338f, 1.111338f, -5.564415f,
    -7.762486f, -1.12146f, 7.768f, 5.568891f, 3.345f, -3.352812f, 1.112953f, -5.573332f,
    -7.774765f, -1.123366f, 7.78f, 5.577739f, 3.351f, -3.358181f, 1.114609f, -5.582111f,
    -7.786874f, -1.125199f, 7.792f, 5.586449f, 3.356f, -3.363443f, 1.116305f, -5.590751f,
    -7.798811f, -1.126958f, 7.804f, 5.595019f, 3.36f, -3.368599f, 1.118042f, -5.599251f,
    -7.810576f, -1.128643f, -3.373647f, -1.130255f, -3.378587f, -1.131792f, -3.383419f, 3.381f,
    -3.388143f, 3.386f, 1.127077f, 3.39f, 1.129045f, 3.395f, 1.131048f, 3.401f,
    1.133088f, 3.406f, 1.135163f, 3.41f, 1.137272f, 3.416f, 1.139412f, -1.142319f,
    1.141584f, -1.143128f, -3.425708f, -1.143867f, -3.429329f, -1.144538f, -3.432841f, -1.14514f,
    -3.436242f, -1.145675f, -3.439533f, -1.146145f, -3.442714f, -1.146549f, -3.445786f, -3.448749f};

#endif  // SV_HESAI_LIDAR_PACKET_PANDAR_128CH_H_