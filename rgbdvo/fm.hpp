#ifndef __RGBD_VO_FM__
#define __RGBD_VO_FM__
#include <memory>
#include <string>

#include "motion_estimate.hpp"
#include "depth.hpp"
#include "frame.hpp"
#include "settings.hpp"

namespace rgbdvo {


class FrameMatcher {
 public:
  FrameMatcher(const Camera& camera,
               const Options& options = GetFrameMatcherDefaultOptions());
  ~FrameMatcher() {}
  void SetRefFrame(const uint8_t* gray, const uint16_t* raw_depth);
  bool Match(const uint8_t* gray, const uint16_t* raw_depth);
  const Eigen::Isometry3d& GetMotionEstimate() const {
    return motion_estimator_->GetMotionEstimate();
  }

  const Eigen::MatrixXd& GetCov() const {
    return motion_estimator_->GetMotionEstimateCov();
  }

  const IntrinsicParameters& GetInputCameraParameters() const {
    return camera_->GetInputCameraParameters();
  }

 private:
  std::unique_ptr<Camera> camera_;
  std::unique_ptr<Frame> ref_frame_;
  std::unique_ptr<Frame> tar_frame_;
  std::unique_ptr<MotionEstimator> motion_estimator_;

  int32_t num_depth_pixels_;
  std::unique_ptr<float[]> depth_buffer_;
  std::unique_ptr<Depth> depth_;

  int32_t fast_threshold_;
};

} // namespace rgbdvo
#endif