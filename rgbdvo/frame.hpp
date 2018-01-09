#ifndef __RGBD_VO_FRAME__
#define __RGBD_VO_FRAME__
#include <cstdint>
#include <cassert>
#include <vector>
#include <Eigen/Geometry>

#include "feature.hpp"
#include "pyramid.hpp"
// #include "helper.hpp"
#include "settings.hpp"
namespace rgbdvo {
class Camera;
class CameraInstrinsics;
class Rectification;
class Depth;

class Frame {
  public:
     Frame(const Camera* camera,
           const Options& options);

    ~ Frame();

    void PrepareFrame(const uint8_t* raw_gray,
                      int32_t fast_threshold,
                      Depth* depth);

    int32_t GetNumKeypoints() const {
      int32_t ret = 0;
      for(int32_t i=0; i < num_levels_; i++)
        ret += levels_[i]->GetNumKeypoints();
      return ret;
    }

    int32_t GetNumDetectedKeypoints() const {
      int32_t ret = 0;
      for(int32_t i=0; i < num_levels_; i++)
        ret += levels_[i]->GetNumDetectedKeypoints();
      return ret;
    }

    int32_t GetNumLevels() const {
      return num_levels_;
    }

    const PyramidLevel* GetLevel(int32_t i) const {
      return levels_[i];
    }

    PyramidLevel* GetLevel(int32_t i) {
      return levels_[i];
    }

    int32_t GetFeatureWindowSize() const { return feature_window_size_; }

  private:

    void FilterBadKeypoints();

    int32_t width_, height_;

    int32_t num_levels_;
    int32_t feature_window_size_;

    const Camera* camera_;

    std::vector<PyramidLevel*> levels_;
};

}
#endif