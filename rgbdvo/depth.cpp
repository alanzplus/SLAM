#include "depth.hpp"
#include "pyramid.hpp"
#include "frame.hpp"
#include "feature.hpp"

namespace rgbdvo {

Depth::Depth(const IntrinsicParameters& camera_params) {
  rgb_width_ = camera_params.width;
  rgb_height_ = camera_params.height;
  depth_width_ = camera_params.depth_width;
  depth_height_ = camera_params.depth_height;

  x_scale_ = (float)depth_width_ / (float)rgb_width_;
  y_scale_ = (float)depth_height_ / (float)rgb_height_;

  int32_t num_depth_pixels = depth_width_ * depth_height_;
  depth_data_.reset(new float[num_depth_pixels]);

  int32_t num_rgb_pixels = rgb_width_ * rgb_height_;
  rays_.reset(new Eigen::Matrix<double, 3, Eigen::Dynamic>(3, num_rgb_pixels));

  fx_inv_ = 1.0 / camera_params.fx;
  fy_inv_ = 1.0 / camera_params.fy;
  neg_cx_div_fx_ = - camera_params.cx * fx_inv_;
  neg_cy_div_fy_ = - camera_params.cy * fy_inv_;

  // precompute RGB rays
  int32_t rindex = 0;
  for(int32_t v = 0; v < rgb_height_; v++) {
    for(int32_t u = 0; u < rgb_width_; u++) {
      (*rays_)(0, rindex) = u * fx_inv_ + neg_cx_div_fx_;
      (*rays_)(1, rindex) = v * fy_inv_ + neg_cy_div_fy_;
      (*rays_)(2, rindex) = 1;
      rindex++;
    }
  }
}

Depth::~Depth() {}

void
Depth::SetDepth(const float* depth_image) {
  int32_t num_depth_pixels = depth_width_ * depth_height_;
  memcpy(depth_data_.get(), depth_image, num_depth_pixels * sizeof(float));
}

void
Depth::GetXYZ(Frame* frame)
{
  int32_t num_levels = frame->GetNumLevels();
  for (int32_t level_num=0; level_num < num_levels; ++level_num) {
    auto* level = frame->GetLevel(level_num);

    int32_t num_kp = level->GetNumKeypoints();
    for (int32_t kp_ind = 0; kp_ind < num_kp; ++kp_ind) {

      auto* kpdata(level->GetKeypointData(kp_ind));

      int32_t u = (int32_t)(kpdata->rect_base_uv(0)+0.5);
      int32_t v = (int32_t)(kpdata->rect_base_uv(1)+0.5);

      // kpdata->disparity = NAN;

      float z = depth_data_[RGB2DepthIdx(u, v)];
      if (isnan(z)) {
        kpdata->has_depth = false;
        kpdata->xyzw = Eigen::Vector4d(NAN, NAN, NAN, NAN);
        kpdata->xyz = Eigen::Vector3d(NAN, NAN, NAN);
      } else {
        kpdata->has_depth = true;
        kpdata->xyz = z * rays_->col(v * rgb_width_ + u);
        kpdata->xyzw.head<3>() = kpdata->xyz;
        kpdata->xyzw.w() = 1;
      }
    }
  }
}

void
Depth::RefineXYZ(FeatureMatch * matches,
                 int32_t num_matches,
                 Frame* frame) {
  for (int32_t m_ind = 0; m_ind < num_matches; m_ind++) {
    FeatureMatch& match = matches[m_ind];
    if (match.status == MATCH_NEEDS_DEPTH_REFINEMENT) {
      if (GetXYZInterp(&match.refined_target_keypoint)) {
        match.status = MATCH_OK;
      } else {
        match.status = MATCH_REFINEMENT_FAILED;
        match.inlier = false;
      }
    }
  }
}

bool
Depth::GetXYZInterp(KeypointData* kpdata) {
  float u_f = kpdata->rect_base_uv(0);
  float v_f = kpdata->rect_base_uv(1);
  float v_f_d = v_f * y_scale_;
  float u_f_d = u_f * x_scale_;
  int32_t v = (int32_t)v_f_d;
  int32_t u = (int32_t)u_f_d;
  float wright  = (u_f_d - u);
  float wbottom = (v_f_d - v);

  // can't handle borders
  assert(u >= 0 && v >= 0 && u < depth_width_ - 1 && v < depth_height_ - 1);

  float w[4] = {
    (1 - wright) * (1 - wbottom),
    wright * (1 - wbottom),
    (1 - wright) * wbottom,
    wright * wbottom
  };

  int32_t depth_index = v * depth_width_ + u;
  double depths[4] = {
    depth_data_[depth_index + 0],
    depth_data_[depth_index + 1],
    depth_data_[depth_index + depth_width_],
    depth_data_[depth_index + depth_width_ + 1]
  };

  // missing any depth data for surrounding pixels?
  int32_t num_missing_data = 0;
  for(int32_t i = 0; i<4; i++)
    if(isnan(depths[i]))
      num_missing_data++;

  if(num_missing_data == 4) {
    // missing all surrounding depth data.
    return false;
  }

  if(num_missing_data) {
    // if any of the surrounding depth data is missing, just clamp to the
    // nearest pixel.  interpolation gets messy if we try to do otherwise
    float wmax = -1;
    double z = NAN;
    for(int32_t i=0; i<4; i++) {
      if(isnan(depths[i]) && w[i] > wmax) {
        z = depths[i];
        wmax = w[i];
      }
    }
    kpdata->xyz.x() = z * (u_f * fx_inv_ + neg_cx_div_fx_);
    kpdata->xyz.y() = z * (v_f * fy_inv_ + neg_cy_div_fy_);
    kpdata->xyz.z() = z;
  } else {
    double z = 0;
    for(int32_t i=0; i<4; i++)
      z += depths[i] * w[i];
    kpdata->xyz.x() = z * (u_f * fx_inv_ + neg_cx_div_fx_);
    kpdata->xyz.y() = z * (v_f * fy_inv_ + neg_cy_div_fy_);
    kpdata->xyz.z() = z;
  }
  kpdata->xyzw.head<3>() = kpdata->xyz;
  kpdata->xyzw.w() = 1;
  return true;
}

}