#ifndef __RGBD_VO_HELPER__
#define __RGBD_VO_HELPER__

#include <cstdio>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <emmintrin.h>
#include <string>
#include <map>

#include "settings.hpp"

namespace rgbdvo {

class Option {
  public:
    Option() = delete;
    ~Option() = delete;
    static bool GetInt(const Options& options, std::string key, int32_t* ret);
    static bool GetBool(const Options& options, std::string key, bool *ret);
    static bool GetDouble(const Options& options, std::string key, double *ret);
    static int32_t GetIntOrDefault(const Options& options, std::string key,
                                   const Options& defaults);
    static bool GetBoolOrDefault(const Options& options, std::string key,
                                    const Options& defaults);
    static double GetDoubleOrDefault(const Options& options,
                                     std::string key,
                                     const Options& defaults);
};

template<class T>
bool IS_ALIGNED16(T x) {
  return ((uintptr_t)(x) & 0xF) == 0;
}

static inline int32_t
round_up_to_multiple(int32_t x, int32_t a) {
  int32_t rem = x % a;
  if(rem)
    return x + (a - rem);
  return x;
}

static inline Eigen::Vector3d
QuatToRPY(const Eigen::Quaterniond&q) {
  double roll_a = 2 * (q.w() * q.x() + q.y() * q.z());
  double roll_b = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  double roll = atan2(roll_a, roll_b);

  double pitch_sin = 2 * (q.w() * q.y() - q.z() * q.x());
  double pitch = asin(pitch_sin);

  double yaw_a = 2 * (q.w() * q.z() + q.x() * q.y());
  double yaw_b = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  double yaw = atan2(yaw_a, yaw_b);
  return Eigen::Vector3d(roll, pitch, yaw);
}

static inline Eigen::Quaterniond
RPYToQuat(const Eigen::Vector3d rpy) {
  double roll = rpy(0), pitch = rpy(1), yaw = rpy(2);

  double halfroll = roll / 2;
  double halfpitch = pitch / 2;
  double halfyaw = yaw / 2;

  double sin_r2 = sin(halfroll);
  double sin_p2 = sin(halfpitch);
  double sin_y2 = sin(halfyaw);

  double cos_r2 = cos(halfroll);
  double cos_p2 = cos(halfpitch);
  double cos_y2 = cos(halfyaw);

  Eigen::Quaterniond q;
  q.w() = cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2;
  q.x() = sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2;
  q.y() = cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2;
  q.z() = cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2;
  return q;
}

static inline void
PrintIsometry(const Eigen::Isometry3d & iso) {
  const Eigen::Vector3d & t = iso.translation();
  Eigen::Vector3d rpy =
    QuatToRPY(Eigen::Quaterniond(iso.rotation())) * 180.0 / M_PI;
  fprintf(stderr,
          "trans:(% 6.3f % 6.3f % 6.3f) rot:(% 6.3f % 6.3f % 6.3f)",
          t(0), t(1), t(2), rpy(0), rpy(1), rpy(2));
}

}

#endif
