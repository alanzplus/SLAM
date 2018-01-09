#ifndef __RGBD_VO_DEPTH__
#define __RGBD_VO_DEPTH__

#include <cinttypes>
#include <memory>

#include "camera.hpp"
namespace rgbdvo {
class Frame;
class KeypointData;
class FeatureMatch;

class Depth {
  public:
    Depth() {}
    explicit Depth(const IntrinsicParameters& camera_params);
    ~Depth();

    // setDepthImage -> SetDepth
    void SetDepth(const float* depth_data);

    // haveXyz -> HaveXYZ
    bool HaveXYZ(int32_t u, int32_t v) {
      return !isnan(depth_data_[RGB2DepthIdx(u, v)]);
    }
    // getXyz -> GetXYZ
    void GetXYZ(Frame * frame);

    // refineXyz -> RefineXYZ
    void RefineXYZ(FeatureMatch * matches,
                   int32_t num_matches,
                   Frame* frame);

    double GetBaseline() const { return 0; }

    Depth(const Depth& rhs) = delete;
    Depth& operator=(const Depth& rhs) = delete;

  private:

    int32_t RGB2DepthIdx(double u, double v) const {
      return (int32_t)(v * y_scale_) * depth_width_ + (int32_t)(u * x_scale_);
    }

    bool GetXYZInterp(KeypointData* kpdata);

    int32_t rgb_width_, rgb_height_;
    int32_t depth_width_, depth_height_;
    float x_scale_, y_scale_;

    std::unique_ptr<Eigen::Matrix<double, 3, Eigen::Dynamic>> rays_;
    std::unique_ptr<float[]> depth_data_;

    double fx_inv_, fy_inv_;
    double neg_cx_div_fx_, neg_cy_div_fy_;
};

}

#endif
