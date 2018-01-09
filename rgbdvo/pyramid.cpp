#include "pyramid.hpp"

#include <cassert>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <iostream>

#include "helper.hpp"

static const size_t ALIGNMENT = 16;

namespace rgbdvo {

void
GridKeyPointFilter::Filter(std::vector<KeyPoint>* keypoints) {
  // clear grid buckets - only makes sense if this object is reused
  for (auto& grid : grids_) {
    grid.clear();
  }

  // insert keypoints in corresponding bucket
  for (auto& kp : (*keypoints)) {
    int32_t grid_iy = kp.v / grid_height_;
    int32_t grid_ix = kp.u / grid_width_;

    assert(grid_ix >= 0 && grid_ix < grid_cols_);
    assert(grid_iy >= 0 && grid_iy < grid_rows_);

    int32_t grid_idx = grid_iy * grid_cols_ + grid_ix;
    grids_[grid_idx].push_back(kp);
  }

  // clear original keypoints before refilling it with keypoints to keep
  keypoints->clear();

  // select strongest from each bucket
  for (auto& grid_kps : grids_) {
    std::vector<KeyPoint>::iterator new_end_itr;
    if (grid_kps.size() > static_cast<size_t>(max_keypoints_per_grid_)) {
      new_end_itr = grid_kps.begin() + max_keypoints_per_grid_;
      std::nth_element(grid_kps.begin(), new_end_itr, grid_kps.end(),
        [](const KeyPoint& lhs, const KeyPoint& rhs) {
          return lhs.score > rhs.score;
        });
    } else {
      new_end_itr = grid_kps.end();
    }
    std::copy(grid_kps.begin(), new_end_itr, std::back_inserter(*keypoints));
  }
}

int32_t
GaussPyramid::BuildNextLevel(const uint8_t* src, int32_t src_stride,
                             int32_t src_width, int32_t src_height,
                             uint8_t* dest, int32_t dst_stride,
                             uint8_t* buffer) {
  HorizontalGaussFilter(src, src_stride,
                        src_width, src_height,
                        buffer, src_height);

  HorizontalGaussFilter(buffer, src_height, src_height,
                        src_width / 2, dest, dst_stride);
  return 0;
}

void GaussPyramid::
HorizontalGaussFilter(const uint8_t* src, int32_t src_stride,
                      int32_t src_width, int32_t src_height,
                      uint8_t* dest, int32_t dest_stride) {
  int32_t dst_max_row = src_width/2 - 1;
  int32_t dst_col;

  for(dst_col = 0; dst_col < src_height; dst_col++) {
    const uint8_t* s = &src[dst_col * src_stride];
    uint16_t sum = 0;
    int32_t dst_row;

    // left border
    sum = s[0] + 4 * s[1] + 3 * s[2];
    dest[dst_col] = sum >> 3;

    // middle
    for(dst_row=1; dst_row < dst_max_row; dst_row++) {
      sum = s[0] + 4 * s[1] + 6 * s[2] + 4 * s[3] + s[4];
      dest[dst_row * dest_stride + dst_col] = sum / 16;
      s+=2;
    }

    // right border
    if(src_width & 0x1) {
      sum = 3 * s[0] + 4 * s[1] + s[2];
      dest[dst_max_row * dest_stride + dst_col] = sum >> 3;
    } else {
      sum = s[0] + 4 * s[1] + 7 * s[2] + 4 * s[3];
      dest[dst_max_row * dest_stride + dst_col] = sum >> 4;
    }
  }
}

PyramidLevel::PyramidLevel(int32_t width, int32_t height, int32_t level_num,
                           int32_t feature_window_size,
                           const Options& options) {

  int32_t grid_width, grid_height, max_keypoints_per_grid;
  Option::GetInt(options, "grid-width", &grid_width);
  Option::GetInt(options, "grid-height", &grid_height);
  Option::GetInt(options, "max-keypoints-per-grid", &max_keypoints_per_grid);
  grid_filter_ = GridKeyPointFilter(
    width, height, grid_width, grid_height, max_keypoints_per_grid);

  width_ = width;
  height_ = height;
  raw_gray_stride_ = round_up_to_multiple(width_, ALIGNMENT);
  descriptor_extractor_.reset(
    new Descriptor(raw_gray_stride_, feature_window_size));

  assert(0 == posix_memalign((void**)&raw_gray_,
         ALIGNMENT, raw_gray_stride_ * height_));

  memset(raw_gray_, 0, raw_gray_stride_ * height_);

  keypoint_min_x_ = feature_window_size;
  keypoint_min_y_ = feature_window_size;
  keypoint_max_x_ = width_ - feature_window_size - 2;
  keypoint_max_y_ = height_ - feature_window_size - 2;

  // allocate workspace for computing the next pyramid level
  int32_t pyrbuf_size = GaussPyramid::GetNextLevelSize(width_, height_);
  pyrbuf_.reset(new uint8_t[pyrbuf_size]);

  level_num_ = level_num;

  num_keypoints_ = 0;
  keypoints_capacity_ = 1500;
  keypoints_.reset(new KeypointData[keypoints_capacity_]);

  initial_keypoints_.reserve(2000);

  descriptors_ = NULL;

  // allocate descriptor buffers
  int desc_buf_size = keypoints_capacity_ * descriptor_extractor_->GetDescriptorStride();
  if(0 != posix_memalign((void**)&descriptors_, ALIGNMENT, desc_buf_size)) {
    fprintf(stderr, "error allocating descriptor memory\n");
  }
}

void
PyramidLevel::IncreaseCapacity(int32_t new_capacity) {
  keypoints_capacity_ = new_capacity;
  keypoints_.reset(new KeypointData[keypoints_capacity_]);

  // allocate descriptor buffers
  int32_t descriptor_buf_size = keypoints_capacity_ * GetDescriptorStride();
  free(descriptors_);
  int32_t status = posix_memalign((void**)&descriptors_, ALIGNMENT,
      descriptor_buf_size);
  if(0 != status) {
    fprintf(stderr, "error allocating descriptor memory\n");
  }
}

PyramidLevel::~PyramidLevel() {
  free(raw_gray_);
  free(descriptors_);
  raw_gray_ = NULL;
  descriptors_ = NULL;
  keypoints_capacity_ = 0;
}

void
PyramidLevel::
ExtractInterpDescriptor(float x, float y, uint8_t* descriptor) const {
  descriptor_extractor_->ExtractInterpDescriptor(raw_gray_, x, y, descriptor);
}

void
PyramidLevel::
ExtractAlignedDescriptor(int32_t x, int32_t y, uint8_t* descriptor) const {
  descriptor_extractor_->ExtractAlignedDescriptor(raw_gray_, x, y, descriptor);
}

void
PyramidLevel::ExtractInterpDescriptors(const KeypointData* keypoints,
                                        int32_t num_keypoints,
                                        uint8_t* descriptors) const {
  descriptor_extractor_->ExtractInterpDescriptors(raw_gray_, keypoints,
                                                  num_keypoints, descriptors);
}

void
PyramidLevel::ExtractAlignedDescriptors(const KeypointData* keypoints,
                                        int32_t num_keypoints,
                                        uint8_t* descriptors) const {
  descriptor_extractor_->ExtractAlignedDescriptors(raw_gray_, keypoints,
                                                   num_keypoints, descriptors);
}

}