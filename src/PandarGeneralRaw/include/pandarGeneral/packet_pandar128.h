#ifndef SV_HESAI_LIDAR_PACKET_PANDAR_128CH_H_
#define SV_HESAI_LIDAR_PACKET_PANDAR_128CH_H_

#include "packet.h"

/**
 * Pandar 128ch
 */

// Head

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

  uint16_t m_u16Delimiter;
  uint8_t m_u8VersionMajor;
  uint8_t m_u8VersionMinor;
  uint8_t m_u8StatusInfoVersion;
  uint8_t m_u8Reserved1;

  bool IsValidDelimiter() const {
    return LittleToNative(m_u16Delimiter) == kDelimiter;
  }
  uint8_t GetDelimiter() const { return LittleToNative(m_u16Delimiter); }
  uint8_t GetVersionMajor() const { return m_u8VersionMajor; }
  uint8_t GetVersionMinor() const { return m_u8VersionMinor; }
  uint8_t GetStatusInfoVersion() const { return m_u8StatusInfoVersion; }

  void Init(uint8_t u8VerMajor, uint8_t u8VerMinor,
            uint8_t u8StatusInfoVer = kStatusInfoV0) {
    m_u16Delimiter = 0xffee;
    m_u8VersionMajor = u8VerMajor;
    m_u8VersionMinor = u8VerMinor;
    m_u8StatusInfoVersion = u8StatusInfoVer;
  }

  void Print() const {
    printf("HS_LIDAR_128CH_PRE_HEADER:\n");
    printf(
        "Delimiter:%02x, valid:%d, Ver Major: %02x, minor: %02x, "
        "StatusInfoVer:%02x\n",
        GetDelimiter(), IsValidDelimiter(), GetVersionMajor(),
        GetVersionMinor(), GetStatusInfoVersion());
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

  uint8_t m_u8LaserNum;
  uint8_t m_u8BlockNum;
  uint8_t m_u8EchoCount;
  uint8_t m_u8DistUnit;
  uint8_t m_u8EchoNum;
  uint8_t m_u8Status;

  uint8_t GetLaserNum() const { return m_u8LaserNum; }
  uint8_t GetBlockNum() const { return m_u8BlockNum; }
  double GetDistUnit() const { return m_u8DistUnit / 1000.f; }
  uint8_t GetEchoCount() const { return m_u8EchoCount; }
  uint8_t GetEchoNum() const { return m_u8EchoNum; }
  bool HasSeqNum() const { return m_u8Status & kSequenceNum; }
  bool HasIMU() const { return m_u8Status & kIMU; }
  bool HasFuncSafety() const { return m_u8Status & kFunctionSafety; }
  bool HasCyberSecurity() const { return m_u8Status & kCyberSecurity; }
  bool HasConfidenceLevel() const { return m_u8Status & kConfidenceLevel; }
  bool IsFirstBlockLastReturn() const {
    return m_u8EchoCount == kFirstBlockLastReturn;
  }
  bool IsFirstBlockStrongestReturn() const {
    return m_u8EchoCount == kFirstBlockStrongestReturn;
  }
  void Print() const {
    printf("HS_LIDAR_HEADER_ME_V4:\n");
    printf(
        "laserNum:%02u, block_num:%02u, DistUnit:%g, EchoCnt:%02u, "
        "EchoNum:%02u, HasSeqNum:%d, HasIMU:%d, "
        "HasFuncSafety:%d, HasCyberSecurity:%d, HasConfidence:%d\n",
        GetLaserNum(), GetBlockNum(), GetDistUnit(), GetEchoCount(),
        GetEchoNum(), HasSeqNum(), HasIMU(), HasFuncSafety(),
        HasCyberSecurity(), HasConfidenceLevel());
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_HEADER_t;

typedef struct HS_LIDAR_128CH_BODY_AZIMUTH_s {
  uint16_t m_u16Azimuth;

  uint16_t GetAzimuth() const { return LittleToNative(m_u16Azimuth); }

  void Print() const {
    printf("HS_LIDAR_BODY_AZIMUTH_ME_V4: azimuth:%u\n", GetAzimuth());
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_BODY_AZIMUTH_t;

typedef struct HS_LIDAR_128CH_BODY_CHN_NO_CONF_s {
  uint16_t m_u16Distance;
  uint8_t m_u8Reflectivity;

  uint16_t GetDistance() const { return LittleToNative(m_u16Distance); }
  uint8_t GetReflectivity() const { return m_u8Reflectivity; }
  void Print() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4:\n");
    printf("Dist:%u, Reflectivity: %u\n", GetDistance(), GetReflectivity());
  }
  void PrintMixData() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_NO_CONF_ME_V4:\n");
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_BODY_CHN_NO_CONF_t;

typedef struct HS_LIDAR_128CH_BODY_CHN_s {
  uint16_t m_u16Distance;
  uint8_t m_u8Reflectivity;
  uint8_t m_u8Confidence;

  uint16_t GetDistance() const { return LittleToNative(m_u16Distance); }
  uint8_t GetReflectivity() const { return m_u8Reflectivity; }
  uint8_t GetConfidenceLevel() const { return m_u8Confidence; }

  void Print() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_ME_V4:\n");
    printf("Dist:%u, Reflectivity: %u, confidenceLevel:%d\n", GetDistance(),
           GetReflectivity(), GetConfidenceLevel());
  }
  void PrintMixData() const {
    printf("HS_LIDAR_BODY_CHN_UNIT_ME_V4:\n");
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_BODY_CHN_t;

typedef struct HS_LIDAR_128CH_BODY_CRC_s {
  uint32_t m_u32Crc;

  uint32_t GetCrc() const { return LittleToNative(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_BODY_CRC_ME_V4:\n");
    printf("crc:0x%08x\n", GetCrc());
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_BODY_CRC_t;

typedef struct HS_LIDAR_128CH_CYBER_SECURITY_s {
  uint8_t m_u8Signature[32];

  uint8_t GetSignatureData(uint8_t index) const {
    return m_u8Signature[index < sizeof(m_u8Signature) ? index : 0];
  }

  void Print() const {
    printf("HS_LIDAR_CYBER_SECURITY_ME_V4:\n");
    for (uint8_t i = 0; i < sizeof(m_u8Signature); i++)
      printf("Signature%d:%d, ", i, GetSignatureData(i));
    printf("\n");
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_CYBER_SECURITY_t;

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

  uint8_t m_u8Version;
  uint8_t m_u8Status;
  uint8_t m_u8FaultInfo;
  uint16_t m_u16FaultCode;
  uint8_t m_u8Reserved[8];
  uint32_t m_u32Crc;

  uint8_t GetVersion() const { return m_u8Version; }

  uint8_t GetLidarState() const {
    return (m_u8Status >> kLidarStateShiftBits) & kLidarStateMask;
  }
  bool IsHistoryFault() const { return m_u8Status & kFaultTypeHistory; }
  bool IsCurrentFault() const { return m_u8Status & kFaultTypeCurrent; }

  uint8_t GetRollingCnt() const {
    return (m_u8Status >> kRollingCntShiftBits) & kRollingCntMask;
  }

  uint8_t GetFaultNum() const {
    return (m_u8FaultInfo >> kFaultNumShiftBits) & kFaultNumMask;
  }
  uint8_t GetFaultID() const {
    return (m_u8FaultInfo >> kFaultIDShiftBits) & kFaultIDMask;
  }

  uint16_t GetFaultCode() const { return LittleToNative(m_u16FaultCode); }

  uint32_t GetCrc() const { return LittleToNative(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_FUNC_SAFETY_ME_V4:\n");
    printf(
        "ver:%u, lidarState:%u, rollingCnt:%u, isCurrFault:%u, "
        "faultNum:%u, faultID:%u, faultCode:0x%04x, crc:0x%08x\n",
        GetVersion(), GetLidarState(), GetRollingCnt(), IsCurrentFault(),
        GetFaultNum(), GetFaultID(), GetFaultCode(), GetCrc());
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_FUNC_SAFETY_t;

typedef struct HS_LIDAR_128CH_TAIL_s {
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

  ReservedInfo1_t m_reservedInfo1;
  ReservedInfo2_t m_reservedInfo2;
  ReservedInfo3_t m_reservedInfo3;
  uint16_t m_u16AzimuthFlag;
  uint8_t m_u8RunningMode;
  uint8_t m_u8ReturnMode;
  uint16_t m_u16MotorSpeed;
  uint8_t m_u8UTC[6];
  uint32_t m_u32Timestamp;
  uint8_t m_u8FactoryInfo;

  uint8_t GetStsID0() const { return m_reservedInfo1.GetID(); }
  uint16_t GetData0() const { return m_reservedInfo1.GetData(); }
  uint8_t GetStsID1() const { return m_reservedInfo2.GetID(); }
  uint16_t GetData1() const { return m_reservedInfo2.GetData(); }
  uint8_t GetStsID2() const { return m_reservedInfo3.GetID(); }
  uint16_t GetData2() const { return m_reservedInfo3.GetData(); }

  uint8_t HasShutdown() const { return m_u8RunningMode == kShutdown; }
  uint8_t GetReturnMode() const { return m_u8ReturnMode; }
  uint16_t GetMotorSpeed() const { return LittleToNative(m_u16MotorSpeed); }
  uint32_t GetTimestamp() const { return LittleToNative(m_u32Timestamp); }
  uint8_t getOperationMode() const { return m_u8RunningMode & 0x0f; }
  uint8_t getAngleState(int blockIndex) const {
    return (m_u16AzimuthFlag >> (2 * (7 - blockIndex))) & 0x03;
  }

  bool IsFirstReturn() const { return m_u8ReturnMode == kFirstReturn; }
  bool IsSecondReturn() const { return m_u8ReturnMode == kSecondReturn; }
  bool IsThirdReturn() const { return m_u8ReturnMode == kThirdReturn; }
  bool IsFourthReturn() const { return m_u8ReturnMode == kFourthReturn; }
  bool IsLastReturn() const { return m_u8ReturnMode == kLastReturn; }
  bool IsStrongestReturn() const { return m_u8ReturnMode == kStrongestReturn; }
  bool IsDualReturn() const { return m_u8ReturnMode == kDualReturn; }
  bool IsFirstSecondReturn() const {
    return m_u8ReturnMode == kFirstSecondReturn;
  }
  bool IsStongestFirstReturn() const {
    return m_u8ReturnMode == kStongestFirstReturn;
  }
  int64_t GetMicroLidarTimeU64() const {
    if (m_u8UTC[0] != 0) {
      struct tm t = {0};
      t.tm_year = m_u8UTC[0];
      if (t.tm_year < 70) {
        return 0;
      }
      t.tm_mon = m_u8UTC[1] - 1;
      t.tm_mday = m_u8UTC[2] + 1;
      t.tm_hour = m_u8UTC[3];
      t.tm_min = m_u8UTC[4];
      t.tm_sec = m_u8UTC[5];
      t.tm_isdst = 0;
      return (mktime(&t) - timezone - 86400) * 1000000 + GetTimestamp();
    } else {
      uint32_t utc_time_big = *(uint32_t *)(&m_u8UTC[0] + 2);
      int64_t unix_second = ((utc_time_big >> 24) & 0xff) |
                            ((utc_time_big >> 8) & 0xff00) |
                            ((utc_time_big << 8) & 0xff0000) |
                            ((utc_time_big << 24));
      return unix_second * 1000000 + GetTimestamp();
    }
  }

  double GetTimestampPoint() const {
    if (m_u8UTC[0] != 0) {
      struct tm t = {0};
      t.tm_year = m_u8UTC[0];
      if (t.tm_year < 70) {
        return 0;
      }
      t.tm_mon = m_u8UTC[1] - 1;
      t.tm_mday = m_u8UTC[2] + 1;
      t.tm_hour = m_u8UTC[3];
      t.tm_min = m_u8UTC[4];
      t.tm_sec = m_u8UTC[5];
      t.tm_isdst = 0;
      return (mktime(&t) - 86400) + GetTimestamp() / 1000000.0f;
    } else {
      uint32_t utc_time_big = *(uint32_t *)(&m_u8UTC[0] + 2);
      int64_t unix_second = ((utc_time_big >> 24) & 0xff) |
                            ((utc_time_big >> 8) & 0xff00) |
                            ((utc_time_big << 8) & 0xff0000) |
                            ((utc_time_big << 24));
      return unix_second + GetTimestamp() / 1000000.0f;
    }
  }

  uint8_t GetFactoryInfo() const { return m_u8FactoryInfo; }
  uint8_t GetUTCData(uint8_t index) const {
    return m_u8UTC[index < sizeof(m_u8UTC) ? index : 0];
  }

  void Print() const {
    printf("HS_LIDAR_TAIL_ME_V4:\n");
    printf(
        "sts0:%d, data0:%d, sts1:%d, data1:%d, sts2:%d, data2:%d, "
        "shutDown:%d, motorSpeed:%u, timestamp:%u, return_mode:0x%02x, "
        "factoryInfo:0x%02x, utc:%u %u %u %u %u %u\n",
        GetStsID0(), GetData0(), GetStsID1(), GetData1(), GetStsID2(),
        GetData2(), HasShutdown(), GetMotorSpeed(), GetTimestamp(),
        GetReturnMode(), GetFactoryInfo(), GetUTCData(0), GetUTCData(1),
        GetUTCData(2), GetUTCData(3), GetUTCData(4), GetUTCData(5));
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_TAIL_t;

typedef struct HS_LIDAR_128CH_TAIL_SEQ_NUM_s {
  uint32_t m_u32SeqNum;

  uint32_t GetSeqNum() const { return LittleToNative(m_u32SeqNum); }
  static uint32_t GetSeqNumSize() { return sizeof(m_u32SeqNum); }

  void CalPktLoss(uint32_t &u32StartSeqNum, uint32_t &u32LastSeqNum, uint32_t &u32LossCount,
                  uint32_t &u32StartTime, uint32_t &u32TotalLossCount, uint32_t &u32TotalStartSeqNum) const {
    // clean-up // bool print = false;
    if (u32StartSeqNum == 0) {
      u32LossCount = 0;
      u32TotalLossCount = 0;
      u32StartTime = GetMicroTickCount();
      u32StartSeqNum = m_u32SeqNum;
      u32LastSeqNum = m_u32SeqNum;
      u32TotalStartSeqNum = m_u32SeqNum;
      return;
    }
    if (m_u32SeqNum - u32LastSeqNum > 1) {
      u32LossCount += (m_u32SeqNum - u32LastSeqNum - 1);
      u32TotalLossCount += (m_u32SeqNum - u32LastSeqNum - 1);
      // clean-up // print = true;
      // if (m_u32SeqNum - u32LastSeqNum - 1 > 1000)
      // printf("%d,  %u, %u\n", m_u32SeqNum - u32LastSeqNum - 1, u32LastSeqNum,
      // m_u32SeqNum);
    }

    // print log every 1s
    if (GetMicroTickCount() - u32StartTime >= 1 * 1000 * 1000) {
      printf("pkt loss freq: %u/%u\n", u32LossCount,
             m_u32SeqNum - u32StartSeqNum);
      u32LossCount = 0;
      u32StartTime = GetMicroTickCount();
      u32StartSeqNum = m_u32SeqNum;
    }

    u32LastSeqNum = m_u32SeqNum;
  }

  void CalPktLoss(uint32_t &u32StartSeqNum, uint32_t &u32LastSeqNum, uint32_t &u32LossCount, uint32_t &u32StartTime) const {
    // clean-up // bool print = false;
    if (m_u32SeqNum - u32LastSeqNum > 1) {
      u32LossCount += (m_u32SeqNum - u32LastSeqNum - 1);
      // clean-up // print = true;
      // if (m_u32SeqNum - u32LastSeqNum - 1 > 1000)
      // printf("%d,  %u, %u\n", m_u32SeqNum - u32LastSeqNum - 1, u32LastSeqNum,
      // m_u32SeqNum);
    }

    // print log every 1s
    if (GetMicroTickCount() - u32StartTime >= 1 * 1000 * 1000) {
      printf("pkt loss freq: %u/%u\n", u32LossCount,
             m_u32SeqNum - u32StartSeqNum);
      u32LossCount = 0;
      u32StartTime = GetMicroTickCount();
      u32StartSeqNum = m_u32SeqNum;
    }

    u32LastSeqNum = m_u32SeqNum;
  }

  void Print() const {
    printf("HS_LIDAR_TAIL_SEQ_NUM_ME_V4:\n");
    printf("seqNum: %u\n", GetSeqNum());
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_TAIL_SEQ_NUM_t;

typedef struct HS_LIDAR_128CH_TAIL_IMU_s {
  int16_t m_i16IMUTemperature;
  uint16_t m_u16IMUAccelUnit;
  uint16_t m_u16IMUAngVelUnit;
  uint32_t m_u32IMUTimeStamp;
  int16_t m_i16IMUXAccel;
  int16_t m_i16IMUYAccel;
  int16_t m_i16IMUZAccel;
  int16_t m_i16IMUXAngVel;
  int16_t m_i16IMUYAngVel;
  int16_t m_i16IMUZAngVel;

  int16_t GetIMUTemperature() const {
    return LittleToNative(m_i16IMUTemperature);
  }
  double GetIMUAccelUnit() const {
    return LittleToNative(m_u16IMUAccelUnit) / 1000.f;
  }
  double GetIMUAngVelUnit() const {
    return LittleToNative(m_u16IMUAngVelUnit) / 100.f;
  }
  uint32_t GetIMUTimestamp() const {
    return LittleToNative(m_u32IMUTimeStamp);
  }
  double GetIMUXAccel() const {
    return LittleToNative(m_i16IMUXAccel) * GetIMUAccelUnit();
  }
  double GetIMUYAccel() const {
    return LittleToNative(m_i16IMUYAccel) * GetIMUAccelUnit();
  }
  double GetIMUZAccel() const {
    return LittleToNative(m_i16IMUZAccel) * GetIMUAccelUnit();
  }
  double GetIMUXAngVel() const {
    return LittleToNative(m_i16IMUXAngVel) * GetIMUAngVelUnit();
  }
  double GetIMUYAngVel() const {
    return LittleToNative(m_i16IMUYAngVel) * GetIMUAngVelUnit();
  }
  double GetIMUZAngVel() const {
    return LittleToNative(m_i16IMUZAngVel) * GetIMUAngVelUnit();
  }

  void Print() const {
    printf("HS_LIDAR_TAIL_IMU_ME_V4:\n");
    printf(
        "Temp:%u, AccelUnit:%f, AngVelUnit:%f, TimeStamp:%u, XAccel:%f, "
        "YAccel:%f, ZAccel:%f, XAngVel:%f, YAngVel:%f, ZAngVel:%f\n",
        GetIMUTemperature(), GetIMUAccelUnit(), GetIMUAngVelUnit(),
        GetIMUTimestamp(), GetIMUXAccel(), GetIMUYAccel(), GetIMUZAccel(),
        GetIMUXAngVel(), GetIMUYAngVel(), GetIMUZAngVel());
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_TAIL_IMU_t;

typedef struct HS_LIDAR_128CH_TAIL_CRC_s {
  uint32_t m_u32Crc;

  uint32_t GetCrc() const { return LittleToNative(m_u32Crc); }

  void Print() const {
    printf("HS_LIDAR_TAIL_CRC_ME_V4:\n");
    printf("crc:0x%08x\n", GetCrc());
  }
} __attribute__((__packed__)) HS_LIDAR_128CH_TAIL_CRC_t;

typedef struct HS_LIDAR_128CH_UNIT_s {
  double distance_;
  uint16_t intensity_;
  uint16_t azimuth_;
} HS_LIDAR_128CH_UNIT_t;

typedef struct HS_LIDAR_128CH_BLOCK_s {
  uint8_t azimuth_state_;
  uint16_t block_azimuth_;
  HS_LIDAR_128CH_UNIT_t units_[HS_LIDAR_128CH_UNIT_NUM];
} HS_LIDAR_128CH_BLOCK_t;

typedef struct HS_LIDAR_128CH_PACKET_s {
  uint8_t laser_num_;
  uint8_t block_num_;
  double dist_unit_;

  HS_LIDAR_128CH_BLOCK_t blocks_[HS_LIDAR_128CH_BLOCK_NUM];

  uint16_t motor_speed_;
  int64_t timestamp_;
  uint8_t return_mode_;
  uint8_t operation_mode_;

  double timestamp_point_;

} HS_LIDAR_128CH_PACKET_t;

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

static const size_t kSizeOfLidar128chPreHeader = sizeof(HS_LIDAR_128CH_PRE_HEADER_t);
static const size_t kSizeOfLidar128chHeader = sizeof(HS_LIDAR_128CH_HEADER_t);
static const size_t kSizeOfLidar128chBodyAzimuth = sizeof(HS_LIDAR_128CH_BODY_AZIMUTH_t);
static const size_t kSizeOfLidar128chBodyCHNNoConf = sizeof(HS_LIDAR_128CH_BODY_CHN_NO_CONF_t);
static const size_t kSizeOfLidar128chBodyCHN = sizeof(HS_LIDAR_128CH_BODY_CHN_t);
static const size_t kSizeOfLidar128chBodyCRC = sizeof(HS_LIDAR_128CH_BODY_CRC_t);
static const size_t kSizeOfLidar128chCyberSecurity = sizeof(HS_LIDAR_128CH_CYBER_SECURITY_t);
static const size_t kSizeOfLidar128chFuncSafety = sizeof(HS_LIDAR_128CH_FUNC_SAFETY_t);
static const size_t kSizeOfLidar128chTail = sizeof(HS_LIDAR_128CH_TAIL_t);
static const size_t kSizeOfLidar128chTailSeqNum = sizeof(HS_LIDAR_128CH_TAIL_SEQ_NUM_t);
static const size_t kSizeOfLidar128chTailIMU = sizeof(HS_LIDAR_128CH_TAIL_IMU_t);
static const size_t kSizeOfLidar128chTailCRC = sizeof(HS_LIDAR_128CH_TAIL_CRC_t);

static const size_t GetSizeOfLidar128chBodyCHN(const bool has_conf) { return has_conf ? kSizeOfLidar128chBodyCHN : kSizeOfLidar128chBodyCHNNoConf; }
static const size_t GetSizeOfLidar128chBody(const uint8_t block_num, const uint8_t laser_num, const bool has_conf) {
  return block_num * (kSizeOfLidar128chBodyAzimuth + (laser_num * GetSizeOfLidar128chBodyCHN(has_conf))) + kSizeOfLidar128chBodyCRC;
}

#define HS_LIDAR_128CH_PACKET_SIZE (kSizeOfLidar128chPreHeader + kSizeOfLidar128chHeader + GetSizeOfLidar128chBody(HS_LIDAR_128CH_BLOCK_NUM, HS_LIDAR_128CH_UNIT_NUM, false) + kSizeOfLidar128chFuncSafety + kSizeOfLidar128chTail + kSizeOfLidar128chTailSeqNum + kSizeOfLidar128chTailIMU + kSizeOfLidar128chTailCRC)
#define HS_LIDAR_128CH_NOCONF_PACKET_SIZE (kSizeOfLidar128chPreHeader + kSizeOfLidar128chHeader + GetSizeOfLidar128chBody(HS_LIDAR_128CH_BLOCK_NUM, HS_LIDAR_128CH_UNIT_NUM, false) + kSizeOfLidar128chFuncSafety + kSizeOfLidar128chTail + kSizeOfLidar128chTailSeqNum + kSizeOfLidar128chTailIMU + kSizeOfLidar128chTailCRC)

// Has ConfidenceLevel
static const size_t kSizeOfLidar128chPacketConf = kSizeOfLidar128chPreHeader + kSizeOfLidar128chHeader +
                                                  GetSizeOfLidar128chBody(HS_LIDAR_128CH_BLOCK_NUM, HS_LIDAR_128CH_UNIT_NUM, true) +
                                                  kSizeOfLidar128chFuncSafety + kSizeOfLidar128chTail + kSizeOfLidar128chTailSeqNum +
                                                  kSizeOfLidar128chTailIMU + kSizeOfLidar128chTailCRC;
static const size_t kSizeOfLidar128chPacketNoConf = kSizeOfLidar128chPreHeader + kSizeOfLidar128chHeader +
                                                    GetSizeOfLidar128chBody(HS_LIDAR_128CH_BLOCK_NUM, HS_LIDAR_128CH_UNIT_NUM, false) +
                                                    kSizeOfLidar128chFuncSafety + kSizeOfLidar128chTail + kSizeOfLidar128chTailSeqNum +
                                                    kSizeOfLidar128chTailIMU + kSizeOfLidar128chTailCRC;

#endif  // SV_HESAI_LIDAR_PACKET_PANDAR_128CH_H_