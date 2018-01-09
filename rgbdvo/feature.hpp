#ifndef __RGBD_VO_UTILITY__
#define __RGBD_VO_UTILITY__
#include <cstdint>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <algorithm>
#include <numeric>


#include <Eigen/Core>

namespace rgbdvo {
struct KeyPoint;
class KeypointData;
class PyramidLevel;

void FAST(const uint8_t* img, int32_t width, int32_t height, int32_t row_stride,
    std::vector<KeyPoint>* keypoints,
    int32_t threshold,
    bool nonmax_suppression);

struct KeyPoint {
  // u <-> X, u <-> Y
  float u, v;
  float score;

  KeyPoint() : u(0), v(0), score(0) {}
  KeyPoint(float v_u, float v_v, float v_score)
    : u(v_u), v(v_v), score(v_score) {}
};

// Argumented Keypoint Structure
class KeypointData {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    // Keypoint in its pyramid level
    KeyPoint kp;

    // Homogeneous coordinates of the keypoint
    Eigen::Vector4d xyzw;

    bool has_depth;

    // Inhomogeneous coordinate of the keypoint
    Eigen::Vector3d xyz;

    // Keypoint pixel coordinates in base pyramid level
    Eigen::Vector2d base_uv;

    // rectified keypoint pixel coordinates in the base pyramid level
    Eigen::Vector2d rect_base_uv;

    /**
     * Pixel disparity, if applicable.  If not applicable (e.g., the
     * depth source is not a stereo camera), this value is NAN
     *
     * Note that the meaning of this value depends on the DepthSource used to
     * compute it.  For StereoDepth, this corresponds directly to stereo
     * disparity.  For PrimeSenseDepth, this corresponds to the "disparity"
     * values reported by the PrimeSense sensor, which are not the same as
     * stereo disparity.  See the PrimeSenseDepth class documentation for more
     * details.
     */
    // float disparity;

    uint8_t pyramid_level;
    int32_t keypoint_index;

    /**
     * to identify unique feature track externally
     */
    int32_t track_id;

    KeypointData() :
      kp(0, 0, 0),
      xyzw(0, 0, 0, 1),
      has_depth (false),
      xyz(0, 0, 0),
      base_uv(0, 0),
      rect_base_uv(0, 0),
      // disparity(NAN),
      pyramid_level(0),
      keypoint_index(0),
      track_id(0)
    {}

    void copyFrom(const KeypointData& src) {
      kp.u = src.kp.u;
      kp.v = src.kp.v;
      kp.score = src.kp.score;
      xyzw = src.xyzw;
      xyz = src.xyz;
      has_depth = src.has_depth;
      base_uv = src.base_uv;
      rect_base_uv = src.rect_base_uv;
      // disparity = NAN;
      pyramid_level = src.pyramid_level;
      keypoint_index = src.keypoint_index;
      track_id = src.track_id;
    }

};


class Descriptor {
 public:
  // raw_gray_strid : width of the input image
  // feature_window_size : square pixel patch width
  Descriptor(int32_t raw_gray_stride, int32_t feature_window_size)
  : raw_gray_stride_(raw_gray_stride),
    feature_window_size_(feature_window_size) {
    Init();
  }

  ~Descriptor () {
    free(descriptor_brightness_offset_);
  }

  // Compute a single descriptor using bilinear interpolation.
  // populatedDescriptorInterp -> ExtractInterpDescriptor
  void ExtractInterpDescriptor(uint8_t *image,
                               float x, float y,
                               uint8_t* descriptor) const;

  // Compute a single aligned descriptor.
  // populateDescriptorAligned -> ExtractAlignedDescriptor
  void ExtractAlignedDescriptor(uint8_t *image,
                                int32_t x, int32_t y,
                                uint8_t* descriptor) const;

  // Compute num_keypoints' descriptors using bilinear interpolation.
  // populatedDescriptorsInterp -> ExtractInterpDescriptors
  void ExtractInterpDescriptors(uint8_t* image,
                                const KeypointData* keypoints,
                                int32_t num_keypoints,
                                uint8_t* descriptors) const;

  // Compute num_keypoints' aligned descriptors.
  // populateDescriptorsAligend -> ExtractAlignedDescriptors
  void ExtractAlignedDescriptors(uint8_t* image,
                                 const KeypointData* keypoints,
                                 int32_t num_keypoints,
                                 uint8_t* descriptors) const;

  // Return the number of bytes that should seperate descriptors in a vector
  // of keypoint descriptors.
  int32_t GetDescriptorStride() const {
    return descriptor_stride_;
  }

  // Return an array of offset indices indicatin the image offset of each
  // descriptor pixel from the descriptor center pixel.
  const int32_t* GetDescriptorIndexOffsets() const {
    return descriptor_index_offsets_.get();
  }

  // Return the number of useful bytes in each descriptor.
  int32_t GetDescriptorLength() const {
    return descriptor_len_;
  }

  Descriptor(const Descriptor& other) = delete;
  Descriptor& operator=(const Descriptor& other) = delete;

 private:

  void Init();
  void Normalized(uint8_t* desc) const;

  int32_t raw_gray_stride_;

  int32_t num_descriptor_pad_bytes_;
  uint8_t* descriptor_brightness_offset_;
  int32_t brightess_offset_num_sse_ops_;

  int32_t descriptor_len_;
  int32_t feature_window_size_;
  int32_t descriptor_stride_;
  std::unique_ptr<int32_t[]> descriptor_index_offsets_;
};

/**
 * Status of a feature match.
 */
enum MatchStatusCode {
  /**
   * match is ok, but needs depth refinement.
   */
  MATCH_NEEDS_DEPTH_REFINEMENT,
  /**
   * match should be rejected.
   */
  MATCH_REFINEMENT_FAILED,
  /**
   * match is ok.
   */
  MATCH_OK
};

/**
 * \ingroup FovisCore
 * \brief Represents a single image feature matched between two camera images
 * taken at different times.
 *
 * The two frames are referred to as the reference and target frames.
 */
class FeatureMatch
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  public:
    /**
     * Initializes a NULL (and useless) feature match.
     */
    FeatureMatch() :
      target_keypoint(NULL),
      ref_keypoint(NULL),
      compatibility_degree(0),
      in_maximal_clique(false),
      inlier(false),
      reprojection_error(0),
      track_id(-1)
    {
    }

    /**
     * Initializes a feature match from a target and reference keypoint.
     */
    FeatureMatch(KeypointData* target_keypoint, KeypointData* ref_keypoint) :
      target_keypoint(target_keypoint),
      ref_keypoint(ref_keypoint),
      compatibility_degree(0),
      in_maximal_clique(false),
      inlier(false),
      reprojection_error(0),
      track_id(-1)
    {
      refined_target_keypoint.copyFrom(*target_keypoint);
    }

    /**
     * The target keypoint.
     */
    KeypointData* target_keypoint;

    /**
     * The reference keypoint.
     */
    KeypointData* ref_keypoint;

    /**
     * The target keypoint, after subpixel refinement of the feature match in
     * image space.
     */
    KeypointData refined_target_keypoint;

    /**
     * binary vector, one entry for every feature match.  Each entry is 1 if
     * the motion according to this match is compatible with the motion
     * according to the other match.
     */
    std::vector<int> consistency_vec;

    /**
     * number of 1s in consistency_vec
     */
    int compatibility_degree;

    /**
     * Is this feature match in the maximal consistency clique
     */
    bool in_maximal_clique;

    /**
     * Is this feature an inlier, used for motion estimation
     */
    bool inlier;

    /**
     * Identifies the match during outlier rejection.
     */
    int id;

    /**
     * The image-space distance between the reference keypoint and the target
     * keypoint reprojected onto the reference image.
     */
    double reprojection_error;

    /**
     * Identifies the feature track externally.  If a feature in a new frame is
     * matched to a feature in the previous frame, then the corresponding
     * FeatureMatch object takes on the track_id value of the FeatureMatch
     * object for the previous frame feature.  If the previous frame feature is
     * new (i.e., wasn't matched to a feature in the previous-previous frame),
     * then the track_id is set to a new value.
     */
    int track_id;

    /**
     * status of the feature match.  Value is one of: \p MATCH_NEEDS_DEPTH_REFINEMENT, \p MATCH_OK, or \p MATCH_REFINEMENT_FAILED.
     */
    MatchStatusCode status;
};


class FeatureMatcher {
public:
  FeatureMatcher ();
  ~FeatureMatcher ();

  /**
   * Feature matching using sum of absolute differences (SAD).
   *
   * \param ref_level features in the reference image.
   * \param target_level features in the target image.
   * \param candidates identifies potential match candidates for each feature
   * in the reference image.  For every reference feature, there is a vector of
   * target feature indices that is a potential match.
   * \param matches output array of matches.  This should be pre-allocated and
   * of size at least min(num features in \p ref_level, num features in \p
   * target_level)
   * \param num_matches output parameter, is set to the number of features
   * matched.
   */
  void matchFeatures(PyramidLevel* ref_level,
                     PyramidLevel* target_level,
                     const std::vector<std::vector<int> >& candidates,
                     FeatureMatch* matches,
                     int* num_matches);

private:
  FeatureMatcher (const FeatureMatcher& other);
  FeatureMatcher& operator=(const FeatureMatcher& other);

  // how many features can be referenced in the temporary workspace buffers
  int _ref_feature_capacity;
  int _target_feature_capacity;

  // temporary workspace buffers for feature matching
  int32_t* _ref_to_target_indices;
  int32_t* _ref_to_target_scores;
  int32_t* _target_to_ref_indices;
  int32_t* _target_to_ref_scores;
};


class SAD {
public:
  SAD(int descriptor_len) :
      _descriptor_len(descriptor_len),
      _nsad_ops(descriptor_len / 16 + ((descriptor_len % 16) ? 1 : 0))
      { }

  /**
   * Calculate SAD score between ref_desc and target_desc. ref_desc
   * and target_desc must be padded with zero-filled pad bytes up to
   * multiple of 16.
   */
  int32_t score(const uint8_t *ref_desc, const uint8_t *target_desc) {
    // compute sum of absolute differences (fast)
    const uint8_t * pp = ref_desc;
    const uint8_t * cp = target_desc;
    __m128i d = _mm_setzero_si128();
    for (int i = 0; i < _nsad_ops; i++) {
      __m128i c = _mm_sad_epu8(*(__m128i *) pp, *(__m128i *) cp);
      d = _mm_add_epi16(c, d);
      pp += 16;
      cp += 16;
    }

    __m128i e = _mm_srli_si128(d, 8);
    __m128i f = _mm_add_epi32(d, e);

    int32_t score = _mm_cvtsi128_si32(f);
    return score;
  }

  int getWorstScore() const {
    return _descriptor_len * 255;
  }

private:
  int _descriptor_len;
  int _nsad_ops;
};

}

#endif

