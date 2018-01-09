#include "vo.hpp"
#include "camera.hpp"
#include "frame.hpp"
#include "helper.hpp"

using namespace std;
namespace rgbdvo {

RGBDVO* RGBDVO::instance_ = nullptr;

void RGBDVO::
Create(const Camera& camera, const Options& options) {
  if (instance_) return;
  instance_ = new RGBDVO(camera, options);
}

RGBDVO* RGBDVO::instance() {
  assert(instance_);
  return instance_;
}

RGBDVO::RGBDVO(const Camera& camera, const Options& options)
: camera_(camera.MakeCopy()), init_cnt_(0),
  reference_frame_changed_(false), frame_cnt_(0) {

  ref_frame_ = new Frame(camera_.get(), options);
  prev_frame_ = new Frame(camera_.get(), options);
  curr_frame_ = new Frame(camera_.get(), options);
  motion_estimator_.reset(new MotionEstimator(camera_.get(), options));

  assert(Option::GetInt(options, "fast-threshold-min", &fast_threshold_min_));
  assert(Option::GetInt(options, "fast-threshold-max", &fast_threshold_max_));
  assert(Option::GetInt(options, "fast-init-threshold", &fast_threshold_));
  assert(Option::GetDouble(options,
                           "fast-adaptive-gain",
                           &fast_adaptive_gain_));
  assert(Option::GetInt(options,
                        "target-pixels-per-feature",
                        &target_pixels_per_feature_));

  const auto& params = camera_->GetInputCameraParameters();
  target_num_features_ =
    params.width * params.height / target_pixels_per_feature_;
  num_depth_pixels_ = params.depth_width * params.depth_height;
  depth_buffer_.reset(new float[num_depth_pixels_]);
  depth_.reset(new Depth(params));
}

RGBDVO::~RGBDVO() {
  delete ref_frame_;
  delete prev_frame_;
  delete curr_frame_;
}

void RGBDVO::
ProcessFrame(const uint8_t* gray, const uint16_t* raw_depth) {

  if (init_cnt_ < 100) {
    if (init_cnt_ == 0) {
      cerr << "Init RGBVO.";
    } else if (init_cnt_ % 20 == 0) {
      cerr << ".";
    }
    ++init_cnt_;
    if (init_cnt_ == 100) {
      cerr << "start." << endl;
    }
    else return;
  }

  if (reference_frame_changed_) {
    swap(ref_frame_, curr_frame_);
    ref_to_prev_trans_.setIdentity();
  } else {
    swap(prev_frame_, curr_frame_);
  }

  bool ref_changed = reference_frame_changed_;
  reference_frame_changed_ = false;
  curr_to_prev_trans_.setIdentity();

  // set depth
  for (int32_t i = 0; i < num_depth_pixels_; ++i) {
    uint16_t d = raw_depth[i];
    depth_buffer_[i] = d ? d * 1e-3 : NAN;
  }
  depth_->SetDepth(depth_buffer_.get());

  // Prepare Frame
  curr_frame_->PrepareFrame(gray, fast_threshold_, depth_.get());

  { // Adative fast threshold
    int32_t err = curr_frame_->GetNumDetectedKeypoints() - target_num_features_;
    int32_t adjust =
      static_cast<int32_t>(((err) * fast_adaptive_gain_));
    fast_threshold_ += adjust;
    fast_threshold_ = min(fast_threshold_, fast_threshold_max_);
    fast_threshold_ = max(fast_threshold_, fast_threshold_min_);
  }

  ++frame_cnt_;
  if (frame_cnt_ < 2) {
    reference_frame_changed_ = true;
    return;
  }

  Eigen::Quaterniond init_rotation_est =
    ref_changed ?
      motion_estimator_->EstimateInitialRotation(ref_frame_, curr_frame_) :
      motion_estimator_->EstimateInitialRotation(prev_frame_, curr_frame_);

  // get previous to referene transformation
  initial_motion_estimate_ = ref_to_prev_trans_.inverse();
  initial_motion_estimate_.rotate(init_rotation_est);
  initial_motion_cov_.setIdentity();

  // estimate motion: current frame to reference frame
  motion_estimator_->EstimateMotion(ref_frame_,
                                    curr_frame_,
                                    depth_.get(),
                                    initial_motion_estimate_,
                                    initial_motion_cov_);

  if (motion_estimator_->IsMotionEstimateValid()) {
    Eigen::Isometry3d to_reference = motion_estimator_->GetMotionEstimate();
    curr_to_ref_trans_ = to_reference;
    curr_to_prev_trans_ = ref_to_prev_trans_ * to_reference;
    curr_to_ref_cov_ = motion_estimator_->GetMotionEstimateCov();
    ref_to_prev_trans_ = to_reference.inverse();
    pose_ = pose_ * curr_to_prev_trans_;
  } else if (!ref_changed) {
    // motion estimate is not valid
    // reference frame is not changed
    initial_motion_estimate_.setIdentity();
    initial_motion_estimate_.rotate(init_rotation_est);
    initial_motion_cov_.setIdentity();

    // Re-estimate
    motion_estimator_->EstimateMotion(prev_frame_,
                               curr_frame_,
                               depth_.get(),
                               initial_motion_estimate_,
                               initial_motion_cov_);

    if (motion_estimator_->IsMotionEstimateValid()) {
      curr_to_prev_trans_ = motion_estimator_->GetMotionEstimate();
      motion_estimate_cov_ = motion_estimator_->GetMotionEstimateCov();
      pose_ = pose_ * curr_to_prev_trans_;
      reference_frame_changed_ = true;
      curr_to_ref_trans_ = ref_to_prev_trans_.inverse() * curr_to_prev_trans_;
    }
  }

  if (!motion_estimator_->IsMotionEstimateValid()) {
    cerr << "Cannot get valid motion estimation, exit." << endl;
    exit(-1);
    // reference_frame_changed_ = true;
  }

  if (motion_estimator_->GetNumInliers() <
      reference_frame_change_threshold_) {
    reference_frame_changed_ = true;
  }

}

}
