// The MIT License (MIT)

// Copyright (c) 2014.4 JZ Xuan <jzxuanuni@gmail.com>

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef __RGBD_SLAM_HELPER__
#define __RGBD_SLAM_HELPER__
#include <string>
#include <unordered_map>
#include "settings.hpp"
namespace rgbdslam {

class DepthToPointCloud {
 public:
  DepthToPointCloud(double hfov, double vfov, double width, double height)
    : hfov_(hfov), vfov_(vfov), width_(width), height_(height) {
    XtoZ_ = 2 * tan(hfov_ / 2);
    YtoZ_ = 2 * tan(vfov_ / 2);
  }
  void operator()(const uint16_t* raw_depth, float* point_cloud);
  float GetX(const uint16_t* raw_depth, int32_t u, int32_t v) {
    int32_t idx = v * width_ + u;
    float z = raw_depth[idx] * 0.001;
    if (isnan(z)) return 0.0;
    return (static_cast<double>(u) / width_ - 0.5) * z * XtoZ_;
  }

  float GetY(const uint16_t* raw_depth, int32_t u, int32_t v) {
    int32_t idx = v * width_ + u;
    float z = raw_depth[idx] * 0.001;
    if (isnan(z)) return 0.0;
    return (0.5 - static_cast<double>(v) / height_) * z * YtoZ_;
  }

  float GetZ(const uint16_t* raw_depth, int32_t u, int32_t v) {
    int32_t idx = v * width_ + u;
    float z = raw_depth[idx] * 0.001;
    return isnan(z) ? 0.0 : z;
  }

  DepthToPointCloud(const DepthToPointCloud& rhs) = delete;
  const DepthToPointCloud& operator=(const DepthToPointCloud& rhs) = delete;
 private:
  double hfov_, vfov_;
  double width_, height_;
  double XtoZ_, YtoZ_;
};

class Setting {
 public:
   Setting() = delete;
   ~Setting() = delete;
   static bool GetInt(const Settings& settings, std::string key, int32_t* ret);
   static bool GetBool(const Settings& settings, std::string key, bool *ret);
   static bool GetDouble(const Settings& settings, std::string key, double *ret);
   static int32_t GetIntOrDefault(const Settings& settings, std::string key,
                                  const Settings& defaults);
   static bool GetBoolOrDefault(const Settings& settings, std::string key,
                                const Settings& defaults);
   static double GetDoubleOrDefault(const Settings& settings,
                                    std::string key,
                                    const Settings& defaults);
};

void ConvertBGR2Gray(int32_t width, int32_t height,
                     const uint8_t* bgr, uint8_t* gray);

} // namespace rgbdslam
#endif