#ifndef __RGBD_VO_PYRAMID__
#define __RGBD_VO_PYRAMID__
#include <vector>
#include <Eigen/Core>
#include "feature.hpp"
#include "settings.hpp"

namespace rgbdvo {

class GridKeyPointFilter {
public:
  GridKeyPointFilter() {}
  GridKeyPointFilter (int32_t img_width, int32_t img_height,
                      int32_t grid_width, int32_t grid_height,
                      int32_t max_keypoints_per_bucket) :
      img_width_(img_width), img_height_(img_height),
      grid_width_(grid_width), grid_height_(grid_height),
      max_keypoints_per_grid_(max_keypoints_per_bucket),
      grid_rows_(ceil((float)img_height/grid_height)),
      grid_cols_(ceil((float)img_width/grid_width)),
      grids_(grid_rows_ * grid_cols_) {}

  void Filter(std::vector<KeyPoint>* keypoints);

private:
  int32_t img_width_, img_height_;
  int32_t grid_width_, grid_height_;
  int32_t max_keypoints_per_grid_;
  int32_t grid_rows_, grid_cols_;
  std::vector<std::vector<KeyPoint>> grids_;

};

class GaussPyramid {
  public:
    GaussPyramid() = delete;
    ~GaussPyramid() = delete;

    static int32_t GetNextLevelSize(int32_t width, int32_t height) {
      return (width * height) / 2;
    }

    static int32_t BuildNextLevel(const uint8_t* src, int32_t src_stride,
                                  int32_t src_width, int32_t src_height,
                                  uint8_t* dest, int32_t dst_stride,
                                  uint8_t* buffer);
  private:
    static void HorizontalGaussFilter(const uint8_t* src, int32_t src_stride,
                                      int32_t src_width, int32_t src_height,
                                      uint8_t* dest, int32_t dest_stride);
};


class PyramidLevel {
  public:
    PyramidLevel(int32_t width, int32_t height,
                 int32_t level_num,
                 int32_t feature_window_size,
                 const Options& options);
    ~PyramidLevel();

    const uint8_t* GetGrayscaleImage() const {
      return raw_gray_;
    }

    int32_t GetGrayscaleImageStride() const {
      return raw_gray_stride_;
    }

    const uint8_t* GetDescriptor(int32_t i) const {
      return descriptors_ + i * GetDescriptorStride();
    }


    int32_t GetDescriptorStride() const {
      return descriptor_extractor_->GetDescriptorStride();
    }

    int32_t GetNumKeypoints() const {
      return num_keypoints_;
    }

    int32_t GetDescriptorLength() const {
      return descriptor_extractor_->GetDescriptorLength();
    }

    const KeyPoint& GetKeypoint(int32_t i) const {
      return keypoints_[i].kp;
    }

    const Eigen::Vector4d& GetKeypointXYZW(int32_t i) const {
      return keypoints_[i].xyzw;
    }

    const float GetKeypointRectBaseU(int32_t kp_index) const {
      return keypoints_[kp_index].rect_base_uv(0);
    }

    const float GetKeypointRectBaseV(int32_t kp_index) const {
      return keypoints_[kp_index].rect_base_uv(1);
    }

    const Eigen::Vector2d& GetKeypointRectBaseUV(int32_t kp_index) const {
      return keypoints_[kp_index].rect_base_uv;
    }

    const KeypointData* GetKeypointData(int32_t i) const {
        return &keypoints_[i];
    }

    KeypointData* GetKeypointData(int32_t i) {
        return &keypoints_[i];
    }

    int32_t GetLevelNum() const { return level_num_; }

    // populateDescriptorInterp -> ExtractInterpDescriptor
    void ExtractInterpDescriptor(float x, float y, uint8_t* descriptor) const;

    // populateDescriptorAligned -> ExtractAlignedDescriptor
    void ExtractAlignedDescriptor(int32_t x, int32_t y, uint8_t* descriptor) const;

    // populateDescriptorsInterp -> ExtractInterpDescriptors
    void ExtractInterpDescriptors(const KeypointData* keypoints,
                                    int32_t num_keypoints,
                                    uint8_t* descriptors) const;

    // populateDescriptorsAligned -> ExtractAlignedDescriptors
    void ExtractAlignedDescriptors(const KeypointData* keypoints,
                                    int32_t num_keypoints,
                                    uint8_t* descriptors) const;

    int32_t GetWidth() const { return width_; }
    int32_t GetHeight() const { return height_; }

    bool isLegalKeypointCoordinate(float x, float y) const {
      return x >= keypoint_min_x_ && x <= keypoint_max_x_ &&
             y >= keypoint_min_y_ && y <= keypoint_max_y_;
    }

    const int32_t* GetDescriptorIndexOffsets() const {
      return descriptor_extractor_->GetDescriptorIndexOffsets();
    }

    const std::vector<KeyPoint>& GetInitialFeatures() const {
      return initial_keypoints_;
    }

    int32_t GetNumDetectedKeypoints() const {
      return num_detected_keypoints_;
    }

  private:
    friend class Frame;

    void IncreaseCapacity(int32_t new_capacity);

    uint8_t* raw_gray_;
    int32_t raw_gray_stride_;

    std::vector<KeyPoint> initial_keypoints_;

    // record the initial number of kps after applying FAST
    int32_t num_detected_keypoints_;

    GridKeyPointFilter grid_filter_;

    std::unique_ptr<KeypointData[]> keypoints_;

    int32_t num_keypoints_;
    int32_t keypoints_capacity_;

    uint8_t* descriptors_;

    int32_t keypoint_min_x_, keypoint_min_y_;
    int32_t keypoint_max_x_, keypoint_max_y_;

    int32_t width_, height_;

    int32_t level_num_;

    std::unique_ptr<uint8_t[]> pyrbuf_;
    std::unique_ptr<Descriptor> descriptor_extractor_;
};

}
#endif