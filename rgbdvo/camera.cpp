#include "camera.hpp"
#include <iostream>
namespace rgbdvo {

Camera::Camera(const IntrinsicParameters& params) {
  input_cam_params_ = params;
  rectified_cam_params_ = params;
  rectified_cam_params_.k1 = 0;
  rectified_cam_params_.k2 = 0;
  rectified_cam_params_.k3 = 0;
  rectified_cam_params_.p1 = 0;
  rectified_cam_params_.p2 = 0;
  MakeXYMap();
}

void
Camera::MakeXYMap() {
  assert(input_cam_params_.width > 0 && input_cam_params_.height > 0);

  Xmap_.reset(new float[input_cam_params_.width * input_cam_params_.height]);
  Ymap_.reset(new float[input_cam_params_.width * input_cam_params_.height]);

  int input_width = input_cam_params_.width;
  int input_height = input_cam_params_.height;
  double fx  = input_cam_params_.fx;
  double fy  = input_cam_params_.fy;
  double cx  = input_cam_params_.cx;
  double cy  = input_cam_params_.cy;
  double fxp = rectified_cam_params_.fx;
  double fyp = rectified_cam_params_.fy;
  double cxp = rectified_cam_params_.cx;
  double cyp = rectified_cam_params_.cy;
  double k1  = input_cam_params_.k1;
  double k2  = input_cam_params_.k2;
  double k3  = input_cam_params_.k3;
  double p1  = input_cam_params_.p1;
  double p2  = input_cam_params_.p2;

  // this code is based on OpenCV cvUndistortPoints
  for (int y = 0; y < input_height; ++y) {
    for (int x = 0; x < input_width; ++x) {
      // normalize according to principal point and focal length
      double x1 = (x - cx)/fx;
      double y1 = (y - cy)/fy;
      double x0 = x1;
      double y0 = y1;
      // Iteratively undistort point
      for(int j = 0; j < 5; ++j) {
        double r2 = x1*x1 + y1*y1;
        double icdist = 1./(1. + ((k3*r2 + k2)*r2 + k1)*r2);
        double deltaX = 2*p1*x1*y1 + p2*(r2 + 2*x1*x1);
        double deltaY = p1*(r2 + 2*y1*y1) + 2*p2*x1*y1;
        x1 = (x0 - deltaX)*icdist;
        y1 = (y0 - deltaY)*icdist;
      }
      //Camera
      Eigen::Vector3d xyw = Eigen::Vector3d(x1,  y1, 1);
      x1 = xyw(0) / xyw(2);
      y1 = xyw(1) / xyw(2);
      // projection for rectified rectified image
      x1 = x1 * fxp + cxp;
      y1 = y1 * fyp + cyp;
      Xmap_[y * input_width + x] = x1;
      Ymap_[y * input_width + x] = y1;
    }
  }
}

Camera::~Camera() {}

Camera*
Camera::MakeCopy() const {
  Camera* ret = new Camera();
  ret->input_cam_params_ = input_cam_params_;
  ret->rectified_cam_params_ = rectified_cam_params_;
  int32_t num_elem = input_cam_params_.width * input_cam_params_.height;
  ret->Xmap_.reset(new float[num_elem]);
  ret->Ymap_.reset(new float[num_elem]);
  std::copy(Xmap_.get(), Xmap_.get() + num_elem, ret->Xmap_.get());
  std::copy(Ymap_.get(), Ymap_.get() + num_elem, ret->Ymap_.get());
  return ret;
}

}