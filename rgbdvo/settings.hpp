#ifndef __RGBD_VO_SETTINGS__
#define __RGBD_VO_SETTINGS__
#include <string>
#include <map>

namespace rgbdvo {

typedef std::map<std::string, std::string> Options;

inline
Options GetRGBDVODefaultOptions() {
  return Options {
    {"reference-frame-change-threshold", "250"},
    {"feature-window-size", "9"},
    {"num-pyramid-levels", "3"},
    {"fast-init-threshold", "20"},
    {"fast-threshold-min", "5"},
    {"fast-threshold-max", "70"},
    {"fast-adaptive-gain", "0.005"},
    {"target-pixels-per-feature", "250"},
    {"grid-width", "80"},
    {"grid-height", "80"},
    {"max-keypoints-per-grid", "25"},
    {"inlier-max-reprojection-error", "1.5"},
    {"clique-inlier-threshold", "0.1"},
    {"min-feature-for-estimate", "10"},
    {"max-mean-reprojection-error", "10.0"},
    {"feature-search-window", "25"}
  };
}

inline
Options GetFrameMatcherDefaultOptions() {
  return Options {
    {"reference-frame-change-threshold", "150"},
    {"feature-window-size", "9"},
    {"num-pyramid-levels", "3"},
    {"fast-init-threshold", "20"},
    {"fast-threshold-min", "5"},
    {"fast-threshold-max", "70"},
    {"fast-adaptive-gain", "0.005"},
    {"target-pixels-per-feature", "250"},
    {"grid-width", "80"},
    {"grid-height", "80"},
    {"max-keypoints-per-grid", "25"},
    {"inlier-max-reprojection-error", "1.5"},
    {"clique-inlier-threshold", "0.1"},
    {"min-feature-for-estimate", "10"},
    {"max-mean-reprojection-error", "10.0"},
    {"feature-search-window", "25"}
  };
}

} // namespace rgbdvo

#endif