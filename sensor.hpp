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

#ifndef __RGBD_SLAM_SENSOR__
#define __RGBD_SLAM_SENSOR__
#include "opencv2/opencv.hpp"
#include "OpenNI.h"
#include <Eigen/Geometry>
namespace sensor {
class XtionProLive {
 public:
  struct IntrinsicParameters {
    IntrinsicParameters() {
      memset(this, 0, sizeof(IntrinsicParameters));
    }
    int32_t width, height;
    int32_t depth_width, depth_height;
    double fx, fy;
    double cx, cy;
    double k1, k2, k3;
    double p1, p2;
    double h_fov, v_fov;
  };

 public:
  static bool Connect(int32_t width = 640, int32_t height = 480, int32_t fps = 30);
  static XtionProLive* instance();
  //
  const IntrinsicParameters& GetIntrinsicParamters() const {
    return params_;
  }

  IntrinsicParameters& GetIntrinsicParamters() {
    return params_;
  }

  // Focal Length in Pixel
  double FocalLength() const;

  // Depth Pixel Size in mm
  double DepthPixelSize() const;

  // Log out the device info
  void LogDeviceInfo() const;

  // capture the rgb and depth image
  bool Grab();

  // capture one frame and
  // prepare the necessary data: bgr, gray, raw depth, point cloud
  bool PrepareAll();

  //
  bool WriteDeviceInfoToFile(const std::string fn);

  bool SaveBGR(const std::string fn);
  bool SaveGray(const std::string fn);
  bool SaveRawDepth(const std::string fn);
  bool SavePointCloud(const std::string fn);
  bool SaveAll(const std::string fn_prefix);

  inline const cv::Mat& BGR() const;
  inline const cv::Mat& Gray() const;
  inline const cv::Mat& RawDepth() const;
  inline const cv::Mat& PointCloud() const;

 private:
  static XtionProLive* instance_;
  openni::Device device_;
  openni::VideoStream depth_sensor_;
  openni::VideoStream color_sensor_;
  openni::VideoFrameRef depth_frame_;
  openni::VideoFrameRef color_frame_;
  cv::Mat bgr_;
  cv::Mat gray_;
  cv::Mat raw_depth_;
  cv::Mat point_cloud_;
  IntrinsicParameters params_;

 private:
  XtionProLive();
  ~XtionProLive();
};

inline const cv::Mat&
XtionProLive::BGR() const {
  return bgr_;
}

inline const cv::Mat&
XtionProLive::Gray() const {
  return gray_;
}

inline const cv::Mat&
XtionProLive::RawDepth() const {
  return raw_depth_;
}

inline const cv::Mat&
XtionProLive::PointCloud() const {
  return point_cloud_;
}


} // namespace sensor
#endif
