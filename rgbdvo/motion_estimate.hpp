#ifndef __RGBD_VO_MOTION_ESTIMATE__
#define __RGBD_VO_MOTION_ESTIMATE__
#include <cstdint>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "feature.hpp"
// #include "helper.hpp"
#include "settings.hpp"
namespace rgbdvo {


enum MotionEstimateStatusCode
{
  NO_DATA,
  SUCCESS,
  INSUFFICIENT_INLIERS,
  OPTIMIZATION_FAILURE,
  REPROJECTION_ERROR
};

extern const char* MotionEstimateStatusCodeStrings[];

class Frame;
class Depth;
class Camera;
class PyramidLevel;

class MotionEstimator {
  public:
    MotionEstimator(const Camera* camera, const Options& options);
    ~MotionEstimator();

    void EstimateMotion(Frame* reference_frame,
                        Frame* target_frame,
                        Depth* depth_source,
                        const Eigen::Isometry3d &init_motion_est,
                        const Eigen::MatrixXd &init_motion_cov);

    bool IsMotionEstimateValid() const {
      return _estimate_status == SUCCESS;
    }

    MotionEstimateStatusCode GetMotionEstimateStatus() const {
      return _estimate_status;
    }

    const Eigen::Isometry3d& GetMotionEstimate() const {
      return *_motion_estimate;
    }

    const Eigen::MatrixXd& GetMotionEstimateCov() const {
      return *_motion_estimate_covariance;
    }

    const FeatureMatch* GetMatches() const {
      return _matches;
    }

    int32_t GetNumMatches() const {
      return _num_matches;
    }

    int32_t GetNumInliers() const {
      return _num_inliers;
    }

    int32_t GetNumReprojectionFailures() const {
      return _num_reprojection_failures;
    }

    double GetMeanInlierReprojectionError() const {
      return _mean_reprojection_error;
    }

    void sanityCheck() const;

    Eigen::Quaterniond
    EstimateInitialRotation(
      const Frame* prev, const Frame* cur,
      const Eigen::Isometry3d&
      init_motion_estimate = Eigen::Isometry3d::Identity());

  private:
    void matchFeatures(PyramidLevel* ref_level, PyramidLevel* target_level);
    void computeMaximallyConsistentClique();
    void estimateRigidBodyTransform();
    void refineMotionEstimate();
    void computeReprojectionError();
    void refineFeatureMatch(PyramidLevel* ref_level,
                            PyramidLevel* target_level,
                            Eigen::Vector2d ref_uv,
                            Eigen::Vector2d init_target_uv,
                            Eigen::Vector2d * final_target_uv,
                            float *delta_sse);

    // Minimize reprojection error.
    Eigen::Isometry3d
    refineMotionEstimate(
        const Eigen::Matrix<double, 4, Eigen::Dynamic>& points,
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_projections,
        double fx, double cx, double cy,
        const Eigen::Isometry3d& initial_estimate,
        int32_t max_iterations);

    // Minimize bidirectional reprojection error.
    void refineMotionEstimateBidirectional(
        const Eigen::Matrix<double, 4, Eigen::Dynamic>& ref_points,
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_projections,
        const Eigen::Matrix<double, 4, Eigen::Dynamic>& target_points,
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& target_projections,
        double fx, double cx, double cy,
        const Eigen::Isometry3d& initial_estimate,
        int32_t max_iterations,
        Eigen::Isometry3d* result,
        Eigen::MatrixXd* result_covariance);

    void
    computeReprojectionError(
        const Eigen::Matrix<double, 4, Eigen::Dynamic>& points,
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_projections,
        const Eigen::Matrix<double, 3, 4>& K,
        const Eigen::Isometry3d& motion,
        Eigen::VectorXd* err,
        int32_t err_offset) ;

    void
    computeProjectionJacobian(
        const Eigen::Matrix<double, 6, 1>& params,
        double fx, double px, double py,
        const Eigen::Matrix<double, 4, Eigen::Dynamic>& points,
        Eigen::Matrix<double, Eigen::Dynamic, 6> *result,
        int32_t result_row_offset);

    void
    computeReverseProjectionJacobian(
        const Eigen::Matrix<double, 6, 1>& params,
        double fx, double px, double py,
        const Eigen::Matrix<double, 4, Eigen::Dynamic>& points,
        Eigen::Matrix<double, Eigen::Dynamic, 6>* result,
        int32_t result_row_offset);

 private:
    Depth* _depth_source;
    FeatureMatcher _matcher;
    FeatureMatch* _matches;
    int32_t _num_matches;
    int32_t _matches_capacity;
    int32_t _num_tracks;
    int32_t _num_frames;
    int32_t _num_inliers;
    double _mean_reprojection_error;
    int32_t _num_reprojection_failures;
    const Camera* camera_;
    Frame* _ref_frame;
    Frame* _target_frame;
    Eigen::Isometry3d* _motion_estimate;
    Eigen::MatrixXd* _motion_estimate_covariance;
    double _max_mean_reprojection_error;
    double _inlier_max_reprojection_error;
    double _clique_inlier_threshold;
    int32_t _min_features_for_valid_motion_estimate;
    double _max_feature_motion;
    int32_t _use_subpixel_refinement;
    bool _update_target_features_with_refined;
    MotionEstimateStatusCode _estimate_status;
};

// Estimates a rough 2D homography registering two images.
class InitialHomographyEstimator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:

  void SetTemplateImage(const uint8_t * grayData,
                        int32_t width,
                        int32_t height,
                        int32_t stride,
                        int32_t downsampleFactor);

  void SetTestImage(const uint8_t * grayData,
                    int32_t width,
                    int32_t height,
                    int32_t stride,
                    int32_t downsampleFactor);

  Eigen::Matrix3f Track(const Eigen::Matrix3f &init_H,
                        int32_t nIters, double *finalRMS);

private:
  int32_t template_rows_, template_cols_;
  Eigen::MatrixXf templateImage_, testImage_;
  Eigen::MatrixXf warpedTestImage_;
  Eigen::MatrixXf errorIm_;
  Eigen::ArrayXf templateDxRow_, templateDyRow_;
  Eigen::MatrixXf templatePoints_;
  Eigen::ArrayXf xx_, yy_;

  static void computeGradient(const Eigen::MatrixXf &image,
                              Eigen::MatrixXf * dxp,
                              Eigen::MatrixXf *dyp);
  static double computeError(const Eigen::MatrixXf &error);
  Eigen::MatrixXf computeJacobian(const Eigen::ArrayXf &dx,
                                  const Eigen::ArrayXf &dy) const;
  Eigen::Matrix3f lieToH(const Eigen::VectorXf &lie) const;
  Eigen::MatrixXf
  constructWarpedImage(const Eigen::MatrixXf &srcImage,
                       const Eigen::MatrixXf &warpedPoints) const;
  static Eigen::ArrayXf flattenMatrix(Eigen::MatrixXf &m);

};

}

#endif