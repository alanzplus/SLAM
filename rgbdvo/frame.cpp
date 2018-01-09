#include <cstdio>
#include <iostream>
#include <emmintrin.h>

#include "depth.hpp"
#include "frame.hpp"
#include "camera.hpp"
#include "helper.hpp"
using namespace std;
namespace rgbdvo {
Frame::Frame(const Camera* camera,
             const Options& options) {
  const auto& input_camera = camera->GetInputCameraParameters();
  camera_ = camera;
  width_ = input_camera.width;
  height_ = input_camera.height;
  Option::GetInt(options, "num-pyramid-levels", &num_levels_);
  Option::GetInt(options, "feature-window-size", &feature_window_size_);
  for (int32_t level_num = 0; level_num < num_levels_; level_num++) {
    int32_t level_width =  width_ >> level_num;
    int32_t level_height =  height_ >> level_num;
    PyramidLevel* level = new PyramidLevel(level_width, level_height,
                                           level_num, feature_window_size_,
                                           options);
    levels_.push_back(level);
  }
}

Frame::~Frame()
{
  for(auto level : levels_) {
    delete level;
  }
  num_levels_ = 0;
}

void
Frame::PrepareFrame(const uint8_t* raw_gray,
                    int32_t fast_threshold,
                    Depth* depth) {

  assert(num_levels_);

  auto first_level = levels_[0];

  // copy raw image to first pyramid level
  for (int32_t row = 0; row < height_; row++) {
    memcpy(first_level->raw_gray_ + row * first_level->raw_gray_stride_,
           raw_gray + row * width_, width_);
  }

  // compute image pyramid and detect initial features
  for (int32_t level_num = 0; level_num < num_levels_; level_num++) {
    auto level = levels_[level_num];

    if (level_num > 0) {
      // resize the image from the previous level
      auto prev_level = levels_[level_num-1];
      int32_t prev_width = prev_level->GetWidth();
      int32_t prev_height = prev_level->GetHeight();
      GaussPyramid::BuildNextLevel(
          prev_level->raw_gray_, prev_level->raw_gray_stride_,
          prev_width, prev_height,
          level->raw_gray_, level->raw_gray_stride_,
          prev_level->pyrbuf_.get());
    }

    level->initial_keypoints_.clear();

    FAST(level->raw_gray_,
         level->width_,
         level->height_,
         level->raw_gray_stride_,
         &level->initial_keypoints_, fast_threshold, 1);

    level->num_detected_keypoints_ =
      static_cast<int32_t>(level->initial_keypoints_.size());

    level->grid_filter_.Filter(&level->initial_keypoints_);

    level->num_keypoints_ = 0;
    // cout << "Level:" << level_num << ","
    //      << "after grid filtering:" << level->initial_keypoints_.size() << ",";

    int32_t num_kp_candidates = level->initial_keypoints_.size();

    // increase buffer size if needed
    if (num_kp_candidates > level->keypoints_capacity_) {
      level->IncreaseCapacity(static_cast<int32_t>(num_kp_candidates*1.2));
    }

    int32_t min_dist_from_edge = (feature_window_size_ - 1) / 2 + 1;
    int32_t min_x = min_dist_from_edge;
    int32_t min_y = min_dist_from_edge;
    int32_t max_x = level->width_ - (min_dist_from_edge + 1);
    int32_t max_y = level->height_ - (min_dist_from_edge + 1);

    for (auto& kp_cand : level->initial_keypoints_) {

      // ignore features too close to border
      if(kp_cand.u < min_x || kp_cand.u > max_x || kp_cand.v < min_y ||
         kp_cand.v > max_y)
        continue;

      KeypointData kpdata;
      kpdata.kp = kp_cand;
      kpdata.base_uv(0) = kp_cand.u * (1 << level_num);
      kpdata.base_uv(1) = kp_cand.v * (1 << level_num);
      kpdata.pyramid_level = level_num;

      assert(kpdata.base_uv(0) >= 0);
      assert(kpdata.base_uv(1) < width_);
      assert(kpdata.base_uv(0) >= 0);
      assert(kpdata.base_uv(1) < height_);

      // lookup rectified pixel coordinates
      int32_t idx =
        static_cast<int32_t>(kpdata.base_uv(1) * width_ + kpdata.base_uv(0));
      camera_->LookupByIndex(idx, &kpdata.rect_base_uv);

      // Ignore the points that fall
      // outside the original image region when undistorted.
      if (kpdata.rect_base_uv(0) < 0 || kpdata.rect_base_uv(0) >= width_ ||
          kpdata.rect_base_uv(1) < 0 || kpdata.rect_base_uv(1) >= height_) {
        continue;
      }

      // ignore features with unknown depth
      int32_t du = static_cast<int32_t>(kpdata.rect_base_uv(0)+0.5);
      int32_t dv = static_cast<int32_t>(kpdata.rect_base_uv(1)+0.5);
      if (!depth->HaveXYZ(du, dv)) { continue; }

      // We will calculate depth of all the keypoints later
      kpdata.xyzw = Eigen::Vector4d(NAN, NAN, NAN, NAN);
      kpdata.has_depth = false;
      kpdata.keypoint_index = level->num_keypoints_;

      kpdata.track_id = -1; //hasn't been associated with a track yet

      level->keypoints_[level->num_keypoints_] = kpdata;
      level->num_keypoints_++;
    }

    // extract features
    level->ExtractAlignedDescriptors(level->keypoints_.get(),
                                     level->num_keypoints_,
                                     level->descriptors_);

    // cout << "after depth filter:" << level->num_keypoints_ << endl;
  }
  // populate 3D position for descriptors. Depth calculation may fail for some
  // of these keypoints.
  depth->GetXYZ(this);

  // Get rid of keypoints with no depth.
  FilterBadKeypoints();

}

void
Frame::FilterBadKeypoints() {
  for (auto level : levels_) {
    int32_t desc_stride = level->GetDescriptorStride();
    // Loop Invariant: kp_data_end and descriptors_end point at end
    // of arrays with 'good' keypoints and descriptors respectively
    KeypointData* kp_data_end = &(level->keypoints_[0]);
    uint8_t* descriptors_end = &(level->descriptors_[0]);
    for (int32_t kp_ind = 0; kp_ind < level->num_keypoints_; ++kp_ind) {
      KeypointData * kp_data = &(level->keypoints_[kp_ind]);
      uint8_t *desc = level->descriptors_ + kp_ind * desc_stride;
      if (kp_data->has_depth) {
        // Keep this keypoint - copy it to end of 'good' keypoint array
        // Avoid redundant copying
        if (kp_data_end != kp_data) {
          *kp_data_end = *kp_data;
          memcpy(descriptors_end, desc, desc_stride);
        }
        ++kp_data_end;
        descriptors_end += desc_stride;
      }
    }

    int32_t new_num_keypoints = kp_data_end - &(level->keypoints_[0]);
    level->num_keypoints_ = new_num_keypoints;
    // Re-index keypoints to keep things consistent
    for (int32_t kp_ind=0; kp_ind < level->num_keypoints_; ++kp_ind) {
      KeypointData& kp_data(level->keypoints_[kp_ind]);
      assert (kp_data.has_depth);
      kp_data.keypoint_index = kp_ind;
    }
  }
}
}