#include "fm.hpp"
#include "helper.hpp"

namespace rgbdvo {

FrameMatcher::
FrameMatcher(const Camera& camera, const Options& options)
  : camera_(camera.MakeCopy()) {

  ref_frame_.reset(new Frame(camera_.get(), options));
  tar_frame_.reset(new Frame(camera_.get(), options));
  motion_estimator_.reset(new MotionEstimator(camera_.get(), options));

  assert(Option::GetInt(options, "fast-init-threshold", &fast_threshold_));

  auto params = camera_->GetInputCameraParameters();
  num_depth_pixels_ = params.depth_width * params.depth_height;
  depth_buffer_.reset(new float[num_depth_pixels_]);
  depth_.reset(new Depth(params));
}

void FrameMatcher::
SetRefFrame(const uint8_t* gray, const uint16_t* raw_depth) {
  for (int32_t i = 0; i < num_depth_pixels_; ++i) {
    uint16_t d = raw_depth[i];
    depth_buffer_[i] = d ? d * 1e-3 : NAN;
  }
  depth_->SetDepth(depth_buffer_.get());
  ref_frame_->PrepareFrame(gray, fast_threshold_, depth_.get());
}

bool FrameMatcher::
Match(const uint8_t* gray, const uint16_t* raw_depth) {
  for (int32_t i = 0; i < num_depth_pixels_; ++i) {
    uint16_t d = raw_depth[i];
    depth_buffer_[i] = d ? d * 1e-3 : NAN;
  }
  depth_->SetDepth(depth_buffer_.get());
  tar_frame_->PrepareFrame(gray, fast_threshold_, depth_.get());

  Eigen::Isometry3d init_motion_estimate(Eigen::Isometry3d::Identity());
  Eigen::MatrixXd init_motion_cov;
  init_motion_cov.setIdentity();
  Eigen::Quaterniond init_rotation_est(
    motion_estimator_->EstimateInitialRotation(ref_frame_.get(),
                                               tar_frame_.get()));
  init_motion_estimate.rotate(init_rotation_est);
  motion_estimator_->EstimateMotion(ref_frame_.get(),
                                    tar_frame_.get(),
                                    depth_.get(),
                                    init_motion_estimate,
                                    init_motion_cov);
  if (!motion_estimator_->IsMotionEstimateValid()) {
    return false;
  }

  return true;
}

} // namespace rgbdvo