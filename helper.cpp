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

#include <iostream>
#include <cassert>

#include "helper.hpp"
using namespace std;
namespace rgbdslam {

void DepthToPointCloud::
operator()(const uint16_t* raw_depth, float* point_cloud) {
  /*
    X_realworld = (X_project / width_ - 0.5) * z * XtoZ
    Y_realworld = (0.5 - Y_project / height_) * z * YtoZ
   */
  for (int32_t y = 0; y < height_; ++y) {
    for (int32_t x = 0; x < width_; ++x) {
      int32_t idx = y * width_ + x;
      float z = raw_depth[idx] * 0.001;
      if (isnan(z)) {
        point_cloud[idx * 3 + 0] = 0;
        point_cloud[idx * 3 + 1] = 0;
        point_cloud[idx * 3 + 2] = 0;
      } else {
        point_cloud[idx * 3 + 0] =
          (static_cast<double>(x) / width_ - 0.5) * z * XtoZ_;
        point_cloud[idx * 3 + 1] =
          (0.5 - static_cast<double>(y) / height_) * z * YtoZ_;
        point_cloud[idx * 3 + 2] = z;
      }
    }
  }
}

// gray = 0.299 * R + 0.587 * G + 0.114 * B
void ConvertBGR2Gray(int32_t width, int32_t height,
                     const uint8_t* bgr, uint8_t* gray) {
  size_t sz = width * height;
  for (int32_t i = 0, j = 0; i < sz; ++i, j += 3) {
    gray[i] = 0.114 * bgr[j + 0] +  // B
              0.587 * bgr[j + 1] +  // G
              0.299 * bgr[j + 2];   // R
  }
}

bool
Setting::GetInt(const Settings& settings, std::string key, int32_t* ret) {
  auto it = settings.find(key);
  if (it == settings.end()) {
    cerr << "Setting " << key << " not specified!" << endl;
    return false;
  }
  *ret = stoi(it->second);
  return true;
}

bool
Setting::GetBool(const Settings& settings, std::string key, bool *ret) {
  auto it = settings.find(key);
  if (it == settings.end()) {
    cerr << "Setting " << key  << " not specified!" << endl;
    return false;
  }

  auto val_copy(it->second);
  // trim string and ignore case.
  val_copy.erase(std::remove_if(val_copy.begin(), val_copy.end(), ::isspace),
                 val_copy.end());
  // ignore case.
  std::transform(val_copy.begin(), val_copy.end(), val_copy.begin(), ::tolower);
  if (val_copy == "true") {
    *ret = true;
  } else if (val_copy == "false") {
    *ret = false;
  } else {
    cerr << "Illegal value of " << it->second
         << " for boolean option " << key << endl;
    return false;
  }
  return true;
}

bool
Setting::GetDouble(const Settings& settings, std::string key, double *ret) {
  auto it = settings.find(key);
  if (it == settings.end()) {
    cerr << "Setting " << key  << " not specified!" << endl;
    return false;
  }
  *ret = stod(it->second);
  return true;
}

int32_t
Setting::GetIntOrDefault(const Settings& settings, std::string key,
                                  const Settings& defaults) {
  int32_t ret;
  if (GetInt(settings, key, &ret)) {
    return ret;
  }
  assert(GetInt(defaults, key, &ret));
  cerr << "Using default value of " << ret << " for setting" << key << endl;
  return ret;
}

bool
Setting::GetBoolOrDefault(const Settings& settings, std::string key,
                                const Settings& defaults) {
  bool ret;
  if (GetBool(settings, key, &ret)) {
    return ret;
  }

  assert(GetBool(defaults, key, &ret));
  cerr << "Using default value of "
       << (ret ? "true" : "false")
       << " for setting " << key << endl;
  return ret;
}

double
Setting::GetDoubleOrDefault(const Settings& settings,
                            std::string key,
                            const Settings& defaults) {
  double ret;
  if (GetDouble(settings, key, &ret)) {
    return ret;
  }

  assert(GetDouble(defaults, key, &ret));
  cerr << "Using default value of " << ret << " for setting" << key << endl;
  return ret;
}

} // namespace rgbdslam