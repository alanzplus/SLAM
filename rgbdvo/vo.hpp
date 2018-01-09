#ifndef __RGBD_VO__
#define __RGBD_VO__
#include <string>
#include <map>
#include <memory>

#include "camera.hpp"
#include "motion_estimate.hpp"
#include "depth.hpp"
#include "settings.hpp"

namespace rgbdvo {

class Frame;
class RGBDVO {
  public:
    static void Create(const Camera& camera, const Options& options);
    static RGBDVO* instance();

    void ProcessFrame(const uint8_t* gray, const uint16_t* raw_depth);

    const Eigen::Isometry3d& GetPose() const {
      return pose_;
    }

    const Eigen::Isometry3d& GetCurrToRefMotionEstimate() const {
      return curr_to_ref_trans_;
    }

    const Eigen::MatrixXd& GetCurrToRefCov() const {
      return curr_to_ref_cov_;
    }

    bool IsKeyFrameChanged() const {
      return reference_frame_changed_;
    }

    const MotionEstimator* GetMotionEstimator() const {
      return motion_estimator_.get();
    }

    const void SetInitPose(Eigen::Isometry3d pose =
                           Eigen::Isometry3d::Identity()) {
      pose_ = pose;
    }

  private:
    RGBDVO(const Camera& camera, const Options& options);
    ~RGBDVO();

  private:
    static RGBDVO* instance_;

    std::unique_ptr<Camera> camera_;
    // Options options_;

    size_t init_cnt_;
    int32_t target_num_features_;

    int32_t num_depth_pixels_;
    std::unique_ptr<float[]> depth_buffer_;
    std::unique_ptr<Depth> depth_;

    bool reference_frame_changed_;
    size_t frame_cnt_;

    Frame* ref_frame_;
    Frame* prev_frame_;
    Frame* curr_frame_;

    std::unique_ptr<MotionEstimator> motion_estimator_;

    //Options
    int32_t fast_threshold_min_, fast_threshold_max_;
    int32_t fast_threshold_;
    double fast_adaptive_gain_;
    int32_t target_pixels_per_feature_;
    int32_t reference_frame_change_threshold_;

    // pose estimate for current frame
    Eigen::Isometry3d pose_;

    // transformation from reference to previous frame
    Eigen::Isometry3d ref_to_prev_trans_;

    // transformation from current to previous frame
    Eigen::Isometry3d curr_to_prev_trans_;


    // transformation from current to reference frame
    Eigen::Isometry3d curr_to_ref_trans_;

    // 6x6 motion estimate covariance
    Eigen::MatrixXd motion_estimate_cov_;

    // 6x6 motion estimate covariance
    Eigen::MatrixXd curr_to_ref_cov_;

    Eigen::Matrix3d initial_homography_est_;
    Eigen::Isometry3d initial_motion_estimate_;
    Eigen::MatrixXd initial_motion_cov_;
};

}
#endif