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

#include "sensor.hpp"
#include "PS1080.h"
#include <iostream>
using namespace std;
namespace sensor {

static void RawDepthToPointCloud(const uint16_t* raw_depth, float* point_cloud,
                                 int32_t width, int32_t height,
                                 double h_fov, double v_fov);

XtionProLive* XtionProLive::instance_ = nullptr;

XtionProLive*
XtionProLive::instance() {
  assert(instance_);
  return instance_;
}

XtionProLive::XtionProLive() {}

XtionProLive::~XtionProLive() {}

bool XtionProLive::
Connect(int32_t width, int32_t height, int32_t fps) {
  using namespace openni;
  instance_ = new XtionProLive();
  if (OpenNI::initialize() != STATUS_OK) {
    cerr << "Cannot initilize:"
         << OpenNI::getExtendedError() << endl;
    instance_ = nullptr;
    return false;
  }

  auto& device = instance_->device_;
  if (device.open(ANY_DEVICE) != STATUS_OK) {
    cerr << "Cannot open device:"
         << OpenNI::getExtendedError() << endl;
         instance_ = nullptr;
         return false;
  }

  // Create and set the property of depth sensor
  if (!device.hasSensor(SENSOR_DEPTH)) {
    cerr << "Device dones't support Depth Sensor:"
         << OpenNI::getExtendedError() << endl;
    return false;
  }

  auto& depth_sensor = instance_->depth_sensor_;
  if (depth_sensor.create(device, SENSOR_DEPTH) != STATUS_OK) {
    cerr << "Cannot Create Depth Sensor:"
         << OpenNI::getExtendedError() << endl;
    return false;
  }

  // Disable mirroring
  if (depth_sensor.setMirroringEnabled(false) != STATUS_OK) {
    cerr << "Cannot disable depth snesor mirroring:"
         << OpenNI::getExtendedError() << endl;
    return false;
  }

  {
    VideoMode depth_mode;
    depth_mode.setFps(fps);
    depth_mode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
    depth_mode.setResolution(width, height);
    if (depth_sensor.setVideoMode(depth_mode) != STATUS_OK) {
      cerr << "Cannot set depth sensor property:"
            << OpenNI::getExtendedError() << endl;
      return false;
    }
  }

  // Create and set the property of color sensor
  if (!device.hasSensor(SENSOR_COLOR)) {
    cerr << "Device doesn't support Color Sensor:"
         << OpenNI::getExtendedError() << endl;
  }
  auto& color_sensor = instance_->color_sensor_;
  if (color_sensor.create(device, SENSOR_COLOR) != STATUS_OK) {
    cerr << "Cannot create Color Sensor:"
         << OpenNI::getExtendedError() << endl;
    return false;
  }

  // Disable mirroing
  if (color_sensor.setMirroringEnabled(false) != STATUS_OK) {
    cerr << "Cannot disable color snesor mirroring:"
         << OpenNI::getExtendedError() << endl;
    return false;
  }

  {
    VideoMode color_mode;
    color_mode.setFps(fps);
    color_mode.setPixelFormat(PIXEL_FORMAT_RGB888);
    color_mode.setResolution(640, 480);
    if (color_sensor.setVideoMode(color_mode) != STATUS_OK) {
      cerr << "Cannot set color sensor property:"
           << OpenNI::getExtendedError() << endl;
      return false;
    }
  }

  if (!device.isImageRegistrationModeSupported(
                IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
    cerr << "Device doesn't support depth to color registration:"
         << OpenNI::getExtendedError() << endl;
    return false;
  }

  device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);

  // Set params
  auto color_info = color_sensor.getVideoMode();
  instance_->params_.width  = color_info.getResolutionX();
  instance_->params_.height = color_info.getResolutionY();
  instance_->params_.h_fov = color_sensor.getHorizontalFieldOfView();
  instance_->params_.v_fov = color_sensor.getVerticalFieldOfView();

  auto depth_info = depth_sensor.getVideoMode();
  instance_->params_.depth_width = depth_info.getResolutionX();
  instance_->params_.depth_height = depth_info.getResolutionY();
  instance_->params_.fx = instance_->FocalLength();
  instance_->params_.fy = instance_->params_.fx;

  instance_->params_.cx = instance_->params_.width / 2.0;
  instance_->params_.cy = instance_->params_.height / 2.0;

  // Loginfo
  instance_->LogDeviceInfo();

  // Start Streaming
  depth_sensor.start();
  color_sensor.start();

  // Other
  instance_->point_cloud_.create(height, width, CV_32FC3);
  return true;
}

double XtionProLive::
FocalLength() const {
  // Focal length in mm
  uint64_t zero_plan_distance;
  depth_sensor_.getProperty(
    XN_STREAM_PROPERTY_ZERO_PLANE_DISTANCE, &zero_plan_distance);
  if (zero_plan_distance == 0) {
    cerr << "Zero Plane Distance is 0, exit" << endl;
    exit(EXIT_FAILURE);
  }
  // Convert focal length into pixel
  return static_cast<double>(zero_plan_distance) / DepthPixelSize();
}

double XtionProLive::
DepthPixelSize() const {
  // zero plane pixel size
  double pixel_size = 0.0;
  depth_sensor_.getProperty(XN_STREAM_PROPERTY_ZERO_PLANE_PIXEL_SIZE, &pixel_size);
  auto info = depth_sensor_.getVideoMode();
  // The depth is generated by downsample from the depth image of size
  // 1280 * 960
  int ratio = 1280 / info.getResolutionX();
  pixel_size *= ratio;
  if (pixel_size == 0.0) {
    cerr << "Zero Plane Pixel Size is 0, exit" << endl;
    exit(EXIT_FAILURE);
  }
  return pixel_size;
}

void XtionProLive::
LogDeviceInfo() const {
  auto color_info = color_sensor_.getVideoMode();
  cerr << "RGB Image Info: "
       << "[ "
       << "width: " << color_info.getResolutionX() << ", "
       << "height: " << color_info.getResolutionY() << ", "
       << "fps: " << color_info.getFps() << ", "
       << "H FOV: " << color_sensor_.getHorizontalFieldOfView() << ", "
       << "V FOV: " << color_sensor_.getVerticalFieldOfView()
       << " ]" << endl;

  auto depth_info = depth_sensor_.getVideoMode();
  cerr << "Depth Image Info: "
       << "[ "
       << "widht: " << depth_info.getResolutionX() << ", "
       << "height: " << depth_info.getResolutionY() << ", "
       << "fps: " << depth_info.getFps() << ", "
       << "H FOV: " << depth_sensor_.getHorizontalFieldOfView() << ", "
       << "V FOV: " << depth_sensor_.getVerticalFieldOfView()
       << " ]" << endl
       << "Depth Focal Length (in pixel): [ "
       << FocalLength()
       << " ]"
       << endl;
}

bool XtionProLive::
Grab() {
  using namespace openni;
  if (color_sensor_.readFrame(&color_frame_) != STATUS_OK) {
    cerr << "color streaming error:"
         << OpenNI::getExtendedError() << endl;
    exit(EXIT_FAILURE);
  }
  if (depth_sensor_.readFrame(&depth_frame_) != STATUS_OK) {
    cerr << "depth streaming error:"
         << OpenNI::getExtendedError() << endl;
    exit(EXIT_FAILURE);
  }
  return true;
}

bool XtionProLive::
PrepareAll() {
  if (!Grab()) return false;

  // Prepare bgr
  const cv::Mat rgb(color_frame_.getHeight(),
                    color_frame_.getWidth(),
                    CV_8UC3, (void *)color_frame_.getData());
  cv::cvtColor(rgb, bgr_, CV_RGB2BGR);

  // Prepare gray
  cv::cvtColor(bgr_, gray_, CV_BGR2GRAY);

  // Prepare depth
  raw_depth_ = cv::Mat(depth_frame_.getHeight(),
                       depth_frame_.getWidth(),
                       CV_16UC1,
                       (void *)depth_frame_.getData());

  // prepare point cloud
  RawDepthToPointCloud((const uint16_t*)raw_depth_.data,
                       (float*)point_cloud_.data,
                       depth_frame_.getWidth(),
                       depth_frame_.getHeight(),
                       depth_sensor_.getHorizontalFieldOfView(),
                       depth_sensor_.getVerticalFieldOfView());
  return true;
}

void RawDepthToPointCloud(const uint16_t* raw_depth,
                          float* point_cloud,
                          int32_t width,
                          int32_t height,
                          double h_fov,
                          double v_fov) {
  double XtoZ = 2 * tan(h_fov / 2);
  double YtoZ = 2 * tan(v_fov / 2);
  double X_RES = static_cast<double>(640);
  double Y_RES = static_cast<double>(480);

  /*
    X_realworld = (X_project / X_RES - 0.5) * z * XtoZ
    Y_realworld = (0.5 - Y_project / Y_RES) * z * YtoZ
  */
  for (int32_t y = 0; y < Y_RES; ++y) {
    for (int32_t x = 0; x < X_RES; ++x) {
      int32_t idx = y * X_RES + x;
      float z = raw_depth[idx] * 0.001;
      if (isnan(z)) {
        point_cloud[idx * 3 + 0] = 0;
        point_cloud[idx * 3 + 1] = 0;
        point_cloud[idx * 3 + 2] = 0;
      } else {
        point_cloud[idx * 3 + 0] =
          (static_cast<double>(x) / X_RES - 0.5) * z * XtoZ;
        point_cloud[idx * 3 + 1] =
          (0.5 - static_cast<double>(y) / Y_RES) * z * YtoZ;
        point_cloud[idx * 3 + 2] = z;
      }
    }
  }
}
} // namespace sensor
