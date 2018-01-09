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

#ifndef __RGBD_SLAM_SETTINGS__
#define __RGBD_SLAM_SETTINGS__
#include <string>
#include <unordered_map>

namespace rgbdslam {
typedef std::unordered_map<std::string, std::string> Settings;

inline
Settings GetPoseGraphDefulatSettings() {
  return Settings {
    {"loop-closure-distance-threshold", "0.5"},    // in meter
    {"loop-closure-angle-threshold", "1.0"},       // in radian
    {"loop-closure-linear-window-size", "-1"},     // -1 means check all
    {"use-single-thread-optimization", "true"}
  };
}
} // namespace rgbdslam
#endif