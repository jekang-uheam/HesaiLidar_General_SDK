#ifndef SV_HESAI_LIDAR_PACKET_H_
#define SV_HESAI_LIDAR_PACKET_H_

#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include <string>

static bool IsLittleEndian() {
  const int a = 1;
  const unsigned char *p = reinterpret_cast<const unsigned char *>(&a);
  return (*p == 1);
}

template <typename T>
T LittleToNative(T data) {
  T out = 0;
  if (IsLittleEndian()) {
    out = data;
  } else {
    size_t data_size = sizeof(data);
    unsigned char *src = reinterpret_cast<unsigned char *>(&data + data_size - 1);
    unsigned char *dst = reinterpret_cast<unsigned char *>(&out);
    for (size_t i = 0; i < data_size; ++i) {
      *dst++ = *src--;
    }
  }
  return out;
}

typedef struct ReservedInfo1_s {
  uint16_t data_;
  uint8_t id_;

  uint16_t GetData() const { return LittleToNative(data_); }
  uint8_t GetID() const { return id_; }
} __attribute__((__packed__)) ReservedInfo1_t;

typedef struct ReservedInfo2_s {
  uint16_t data_;
  uint8_t id_;

  uint16_t GetData() const { return LittleToNative(data_); }
  uint8_t GetID() const { return id_; }

  void Print() const {
    printf("lowerBoard ID:%u, STS:0x%02x\n", id_, data_);
  }
} __attribute__((__packed__)) ReservedInfo2_t;

typedef struct ReservedInfo3_s {
  uint16_t data_;
  uint8_t id_;

  uint16_t GetData() const { return LittleToNative(data_); }
  uint8_t GetID() const { return id_; }
} __attribute__((__packed__)) ReservedInfo3_t;

static unsigned int GetMicroTickCount() {
  unsigned int ret = 0;
#ifdef _MSC_VER
  FILETIME time;
  LARGE_INTEGER larger_int;
  GetSystemTimeAsFileTime(&time);
  larger_int.LowPart = time.dwLowDateTime;
  larger_int.HighPart = time.dwHighDateTime;
  ret = (larger_int.QuadPart - EPOCHFILETIME) / 10;
#else
  timespec time;
  memset(&time, 0, sizeof(time));
  if (clock_gettime(CLOCK_MONOTONIC, &time) == 0) {
    ret = time.tv_nsec / 1000 + time.tv_sec * 1000000;
  }
#endif
  return ret;
}

#endif  // SV_HESAI_LIDAR_PACKET_H_