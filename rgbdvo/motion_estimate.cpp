#include "motion_estimate.hpp"
#include "frame.hpp"
#include "camera.hpp"
#include "depth.hpp"
#include "helper.hpp"

#include <emmintrin.h>
#include <Eigen/SVD>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
namespace rgbdvo {

#define USE_HORN_ABSOLUTE_ORIENTATION
#define USE_ROBUST_STEREO_COMPATIBILITY
#define USE_BIDIRECTIONAL_REFINEMENT

const char* MotionEstimateStatusCodeStrings[] = {
  "NO_DATA",
  "SUCCESS",
  "INSUFFICIENT_INLIERS",
  "OPTIMIZATION_FAILURE",
  "REPROJECTION_ERROR"
};

template <typename DerivedA, typename DerivedB>
int absolute_orientation_horn(const Eigen::MatrixBase<DerivedA>& P1,
        const Eigen::MatrixBase<DerivedB>& P2,
        Eigen::Isometry3d* result)
{
    int num_points = P1.cols();
    assert(P1.cols() == P2.cols());
    assert(P1.rows() == 3 && P2.rows() == 3);
    if(num_points < 3)
        return -1;

    // compute centroids of point sets
    Eigen::Vector3d P1_centroid = P1.rowwise().sum() / num_points;
    Eigen::Vector3d P2_centroid = P2.rowwise().sum() / num_points;

    Eigen::MatrixXd R1 = P1;
    R1.colwise() -= P1_centroid;
    Eigen::MatrixXd R2 = P2;
    R2.colwise() -= P2_centroid;

    // compute matrix M
    double Sxx = R1.row(0).dot(R2.row(0));
    double Sxy = R1.row(0).dot(R2.row(1));
    double Sxz = R1.row(0).dot(R2.row(2));
    double Syx = R1.row(1).dot(R2.row(0));
    double Syy = R1.row(1).dot(R2.row(1));
    double Syz = R1.row(1).dot(R2.row(2));
    double Szx = R1.row(2).dot(R2.row(0));
    double Szy = R1.row(2).dot(R2.row(1));
    double Szz = R1.row(2).dot(R2.row(2));

    double A00 = Sxx + Syy + Szz;
    double A01 = Syz - Szy;
    double A02 = Szx - Sxz;
    double A03 = Sxy - Syx;
    double A11 = Sxx - Syy - Szz;
    double A12 = Sxy + Syx;
    double A13 = Szx + Sxz;
    double A22 = -Sxx + Syy - Szz;
    double A23 = Syz + Szy;
    double A33 = -Sxx - Syy + Szz;

    // prepare matrix for eigen analysis
    Eigen::Matrix4d N;
    N << A00, A01, A02, A03,
        A01, A11, A12, A13,
        A02, A12, A22, A23,
        A03, A13, A23, A33;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(N);

    // rotation quaternion is the eigenvector with greatest eigenvalue
    Eigen::Vector4d eigvals = eigensolver.eigenvalues();
    int max_eigen_ind = 0;
    double max_eigen_val = eigvals(0);
    for(int i=1; i<4; i++) {
        if(eigvals(i) > max_eigen_val) {
            max_eigen_val = eigvals(i);
            max_eigen_ind = i;
        }
    }
    Eigen::Vector4d quat = eigensolver.eigenvectors().col(max_eigen_ind);
    Eigen::Quaterniond rotation(quat[0], quat[1], quat[2], quat[3]);

    // now compute the resulting isometry
    result->setIdentity();
    result->translate(P2_centroid - rotation * P1_centroid);
    result->rotate(rotation);

    return 0;
}

MotionEstimator::
MotionEstimator(const Camera* camera,
                const Options& options) {
  camera_ = camera;

  _ref_frame = NULL;
  _target_frame = NULL;

  _num_inliers = 0;
  _motion_estimate = new Eigen::Isometry3d();
  _motion_estimate_covariance = new Eigen::MatrixXd();
  _motion_estimate->setIdentity();
  _estimate_status = NO_DATA;
  _motion_estimate_covariance->setIdentity(6,6);

  _matches = NULL;
  _num_matches = 0;
  _matches_capacity = 0;
  _num_tracks = 0;
  _num_frames = 0;

  assert(Option::GetDouble(
    options, "inlier-max-reprojection-error", &_inlier_max_reprojection_error));
  assert(Option::GetDouble(
    options, "clique-inlier-threshold", &_clique_inlier_threshold));
  assert(Option::GetInt(
    options, "min-feature-for-estimate", &_min_features_for_valid_motion_estimate));
  assert(Option::GetDouble(
    options, "max-mean-reprojection-error", &_max_mean_reprojection_error));
  assert(Option::GetDouble(
    options, "feature-search-window", &_max_feature_motion));

  _use_subpixel_refinement = true;
  _update_target_features_with_refined = false;
}

MotionEstimator::~MotionEstimator()
{
  _ref_frame = NULL;
  _target_frame = NULL;
  delete[] _matches;
  _num_matches = 0;
  _matches_capacity = 0;
  _num_inliers = 0;
  delete _motion_estimate_covariance;
  delete _motion_estimate;
  _motion_estimate_covariance = NULL;
  _motion_estimate = NULL;
  _estimate_status = NO_DATA;
}

void
MotionEstimator::EstimateMotion(Frame* ref_frame,
                                Frame* target_frame,
                                Depth* depth_source,
                                const Eigen::Isometry3d &init_motion_est,
                                const Eigen::MatrixXd &init_motion_cov) {

  _depth_source = depth_source;

  _ref_frame = ref_frame;
  _target_frame = target_frame;
  *_motion_estimate = init_motion_est;
  *_motion_estimate_covariance = init_motion_cov;

  _num_matches = 0;
  int max_num_matches = std::min(_ref_frame->GetNumKeypoints(), _target_frame->GetNumKeypoints());
  if (max_num_matches > _matches_capacity) {
    delete[] _matches;
    _matches_capacity = static_cast<int>(max_num_matches * 1.2);
    _matches = new FeatureMatch[_matches_capacity];
  }

  int num_levels = _ref_frame->GetNumLevels();
  for (int level_ind = 0; level_ind < num_levels; level_ind++) {
    PyramidLevel* ref_level = _ref_frame->GetLevel(level_ind);
    PyramidLevel* target_level = _target_frame->GetLevel(level_ind);
    matchFeatures(ref_level, target_level);
  }

  if (_use_subpixel_refinement) {
    depth_source->RefineXYZ(_matches, _num_matches, target_frame);
  }

  // assign a 'local' match id for use in inlier detection
  for (int i=0; i < _num_matches; ++i) {
    _matches[i].id = i;
  }

  computeMaximallyConsistentClique();

// ???????????
#ifdef USE_ROBUST_STEREO_COMPATIBILITY
  // Horn/SVD doesn't do too well when using robust stereo metric
  _estimate_status = SUCCESS;
#else
  estimateRigidBodyTransform();
#endif

  // refine motion estimate by minimizing reprojection error
  refineMotionEstimate();

  // compute inlier reprojection error
  computeReprojectionError();

  // remove features with a high reprojection error from the inlier set
  _num_reprojection_failures = 0;
  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    FeatureMatch& match = _matches[m_ind];
    if (!match.inlier)
      continue;
    if (match.reprojection_error > _inlier_max_reprojection_error) {
      match.inlier = false;
      _num_inliers--;
      _num_reprojection_failures++;
    }
  }

  // prevent propagation of track id's through outlier matches.
  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    FeatureMatch& match = _matches[m_ind];
    if (!match.inlier) {
      match.target_keypoint->track_id = -1;
    }
  }

  // second motion estimate refinement
  refineMotionEstimate();

  // compute new reprojection error
  computeReprojectionError();

  if (_mean_reprojection_error > _max_mean_reprojection_error) {
    _estimate_status = REPROJECTION_ERROR;
  }

  if (_update_target_features_with_refined) {
    for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
      FeatureMatch& match = _matches[m_ind];
      match.target_keypoint->copyFrom(match.refined_target_keypoint);
    }
  }

  _num_frames++;

}

void MotionEstimator::sanityCheck() const {
#ifndef NDEBUG
  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    const FeatureMatch& match = _matches[m_ind];

    const KeypointData* ref_kp = match.ref_keypoint;
    const KeypointData* target_kp = match.target_keypoint;
    assert(ref_kp->pyramid_level >= 0 &&
           ref_kp->pyramid_level < _ref_frame->GetNumLevels());
    assert(target_kp->pyramid_level >= 0 &&
           target_kp->pyramid_level < _target_frame->GetNumLevels());
    const PyramidLevel * ref_level = _ref_frame->GetLevel(ref_kp->pyramid_level);
    const PyramidLevel * target_level = _target_frame->GetLevel(target_kp->pyramid_level);
    assert(ref_kp->kp.u >= 0 && ref_kp->kp.v >= 0 &&
           ref_kp->kp.u < ref_level->GetWidth() &&
           ref_kp->kp.v < ref_level->GetHeight());
    assert(target_kp->kp.u >= 0 && target_kp->kp.v >= 0 &&
           target_kp->kp.u < target_level->GetWidth() &&
           target_kp->kp.v < target_level->GetHeight());
  }
#endif
}

Eigen::Quaterniond MotionEstimator::
EstimateInitialRotation(const Frame* prev, const Frame* cur,
                        const Eigen::Isometry3d& init_motion_estimate) {
  int32_t initial_rotation_pyramid_level = 4;
  int num_pyr_levels = prev->GetNumLevels();
  int pyrLevel = min(num_pyr_levels-1,initial_rotation_pyramid_level);
  const PyramidLevel * ref_level = prev->GetLevel(pyrLevel);
  const PyramidLevel * target_level = cur->GetLevel(pyrLevel);

  InitialHomographyEstimator rotation_estimator;
  rotation_estimator.SetTemplateImage(ref_level->GetGrayscaleImage(),
      ref_level->GetWidth(), ref_level->GetHeight(),
      ref_level->GetGrayscaleImageStride(),
      initial_rotation_pyramid_level - pyrLevel);

  rotation_estimator.SetTestImage(target_level->GetGrayscaleImage(),
      target_level->GetWidth(), target_level->GetHeight(),
      target_level->GetGrayscaleImageStride(),
      initial_rotation_pyramid_level - pyrLevel);

  Eigen::Matrix3f H = Eigen::Matrix3f::Identity();
  double finalRMS = 0;
  H = rotation_estimator.Track(H,8, &finalRMS);
  double scale_factor = 1 << initial_rotation_pyramid_level;
  Eigen::Matrix3f S = Eigen::Matrix3f::Identity() * scale_factor;
  S(2, 2) = 1;
  //scale H up to the full size image
  H = S * H * S.inverse();
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  const IntrinsicParameters& input_camera = camera_->GetInputCameraParameters();
  rpy(0) = asin(H(1, 2) / input_camera.fx);
  rpy(1) = -asin(H(0, 2) / input_camera.fx);
  rpy(2) = -atan2(H(1, 0), H(0, 0));

  Eigen::Quaterniond q = RPYToQuat(rpy);
  return q;
}


void MotionEstimator::matchFeatures(PyramidLevel* ref_level, PyramidLevel* target_level) {
  // get the camera projection matrix
  Eigen::Matrix<double, 3, 4> xyz_c_to_uvw_c =
    camera_->GetRectifiedCameraParameters().toProjectionMatrix();
  // get the ref_to_target isometry cuz of order of loops
  Eigen::Isometry3d ref_to_target = _motion_estimate->inverse();
  Eigen::Matrix<double, 3, 4> reproj_mat = xyz_c_to_uvw_c * ref_to_target.matrix();
  int num_ref_features = ref_level->GetNumKeypoints();
  int num_target_features = target_level->GetNumKeypoints();

  std::vector<std::vector<int> > candidates(num_ref_features);
  for (int ref_ind = 0; ref_ind < num_ref_features; ref_ind++) {
    // constrain the matching to a search-region based on the
    // current motion estimate
    const Eigen::Vector4d& ref_xyzw = ref_level->GetKeypointXYZW(ref_ind);
    assert(!isnan(ref_xyzw(0)) && !isnan(ref_xyzw(1)) &&
           !isnan(ref_xyzw(2)) && !isnan(ref_xyzw(3)));
    Eigen::Vector3d reproj_uv1 = reproj_mat * ref_xyzw;
    reproj_uv1 /= reproj_uv1(2);
    Eigen::Vector2d ref_uv(ref_level->GetKeypointRectBaseU(ref_ind),
                           ref_level->GetKeypointRectBaseV(ref_ind));
    std::vector<int>& ref_candidates(candidates[ref_ind]);
    for (int target_ind = 0; target_ind < num_target_features; target_ind++) {
      Eigen::Vector2d target_uv(target_level->GetKeypointRectBaseU(target_ind),
                                target_level->GetKeypointRectBaseV(target_ind));
      //TODO: Should adapt based on covariance instead of constant sized window!
      //Eigen::Vector2d err = target_uv - ref_uv; //ignore motion est
      Eigen::Vector2d err = target_uv - reproj_uv1.head<2>();
      if (err.norm() < _max_feature_motion) {
        ref_candidates.push_back(target_ind);
      }
    }
  }

  int inserted_matches = 0;
  _matcher.matchFeatures(ref_level, target_level, candidates,
                         &(_matches[_num_matches]), &inserted_matches);
  int old_num_matches = _num_matches;
  _num_matches = old_num_matches + inserted_matches;

  if (_use_subpixel_refinement) {
    for (int n=old_num_matches; n < _num_matches; ++n) {
      //std::cerr << "n = " << n << std::endl;
      FeatureMatch& match(_matches[n]);
      const KeypointData* ref_kpdata(match.ref_keypoint);
      const KeypointData* target_kpdata(match.target_keypoint);
      Eigen::Vector2d ref_uv(ref_kpdata->kp.u, ref_kpdata->kp.v);
      Eigen::Vector2d init_target_uv(target_kpdata->kp.u, target_kpdata->kp.v);
      Eigen::Vector2d final_target_uv;

      float delta_sse = -1;
      refineFeatureMatch(ref_level, target_level, ref_uv, init_target_uv,
                         &final_target_uv, &delta_sse);
      double ds = (init_target_uv - final_target_uv).norm();
      if (ds < 1e-9) {
        match.refined_target_keypoint.copyFrom(*match.target_keypoint);
        match.status = MATCH_OK;
      } else if (ds > 1.5) {
        // TODO make threshold a parameter. Also, reject or keep?
        match.refined_target_keypoint.copyFrom(*match.target_keypoint);
        match.status = MATCH_OK;
      } else {
        match.refined_target_keypoint.kp.u = final_target_uv(0);
        match.refined_target_keypoint.kp.v = final_target_uv(1);
        match.refined_target_keypoint.base_uv =
          final_target_uv * (1 << target_kpdata->pyramid_level);
        camera_->BilinearLookup(match.refined_target_keypoint.base_uv,
            &match.refined_target_keypoint.rect_base_uv);
        match.status = MATCH_NEEDS_DEPTH_REFINEMENT;
      }
    }
  }

  // label matches with their track_id
  for (int n=old_num_matches; n < _num_matches; ++n) {
    FeatureMatch& match(_matches[n]);
    KeypointData* ref_kpdata(match.ref_keypoint);
    KeypointData* target_kpdata(match.target_keypoint);
    if (ref_kpdata->track_id < 0) {
      //ref wasn't already part of a track
      ref_kpdata->track_id = _num_tracks++;
    }
    target_kpdata->track_id = ref_kpdata->track_id;
    match.track_id = ref_kpdata->track_id;
  }

}

// used for sorting feature matches.
static bool consistencyCompare(const FeatureMatch &ca, const FeatureMatch& cb)
{
  return ca.compatibility_degree > cb.compatibility_degree;
}

#ifdef USE_ROBUST_STEREO_COMPATIBILITY
// Robust Stereo Compatibility Check from:
// Heiko Hirschmuller, Peter R. Innocent and Jon M. Garibaldi
// "Fast, Unconstrained Camera Motion Estimation from Stereo without Tracking
// and Robust Statistics"
// Paper recommends a De (compatibility thresh) of ~0.2 pixels

static inline double sqr(double x) { return x * x; }

static inline
double robustStereoCompatibility_computeDL(double L,
                                           const  Eigen::Vector3d & p1,
                                           const  Eigen::Vector3d & p2,
                                           double t, double f, double De) {
  double A = sqr((p1(0) - p2(0)) * (t - p1(0)) - (p1(1) - p2(1)) * p1(1) - (p1(2) - p2(2)) * p1(2));
  double B = sqr((p1(0) - p2(0)) * p1(0) + (p1(1) - p2(1)) * p1(1) + (p1(2) - p2(2)) * p1(2));
  double C = 0.5 * sqr(t * (p1(1) - p2(1)));
  double D = sqr((p1(0) - p2(0)) * (t - p2(0)) - (p1(1) - p2(1)) * p2(1) - (p1(2) - p2(2)) * p2(2));
  double E = sqr((p1(0) - p2(0)) * p2(0) + (p1(1) - p2(1)) * p2(1) + (p1(2) - p2(2)) * p2(2));
  double F = 0.5 * sqr(t * (p1(1) - p2(1)));
  return De / (L * f * t) * sqrt(sqr(p1(2)) * (A + B + C) + sqr(p2(2)) * (D + E + F));
}

static inline bool
robustStereoCompatibility(const Eigen::Vector3d & C1,
                          const Eigen::Vector3d & C2,
                          const Eigen::Vector3d & P1,
                          const Eigen::Vector3d & P2,
                          double baseline,
                          double focal_length,
                          double De) {
  //compute the L quantities (ie dist between pairs of points)
  double L1 = (C2-C1).norm();
  double L2 = (P2-P1).norm();
  //compute the DLs (delta L)
  double DL1 = robustStereoCompatibility_computeDL(L1, C1, C2, baseline, focal_length, De);
  double DL2 = robustStereoCompatibility_computeDL(L2, P1, P2, baseline, focal_length, De);
  return (fabs(L1-L2) <= 3*sqrt(sqr(DL1)+sqr(DL2)));
}
#endif

void MotionEstimator::computeMaximallyConsistentClique()
{
  if (!_num_matches)
    return;

  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    FeatureMatch& match = _matches[m_ind];
    match.consistency_vec.resize(_num_matches);
  }

#ifdef USE_ROBUST_STEREO_COMPATIBILITY
  double baseline = _depth_source->GetBaseline();
  bool have_baseline = baseline > 0;
  // XXX this is not actually correct for kinect/primesense
  const IntrinsicParameters& rparams = camera_->GetRectifiedCameraParameters();
  double stereo_focal_length = rparams.fx;
#endif

  // For each pair of matches, compute the distance between features in the
  // reference frame, and the distance between features in the target frame.
  // Rigid body transformations (isometries) preserve distance, so the distance
  // should not change significantly if the two feature matches are
  // "compatible".
  //
  // If the depth comes from a stereo camera, then apply a consistency metric that
  // allows for disparity error resulting from the stereo baseline.

  // FIXME using both homogeneous and cartesian coordinates is a bit gross here...
  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    FeatureMatch& match = _matches[m_ind];

    const Eigen::Vector4d& ref_xyzw_1 = match.ref_keypoint->xyzw;
    const Eigen::Vector4d& target_xyzw_1 = match.refined_target_keypoint.xyzw;
    const Eigen::Vector3d& ref_xyz_1 = match.ref_keypoint->xyz;
    const Eigen::Vector3d& target_xyz_1 = match.refined_target_keypoint.xyz;
    assert(match.id == m_ind);

    // are the features points at infinity?
    bool ref_infinity = ref_xyzw_1.w() < 1e-9;
    bool target_infinity = target_xyzw_1.w() < 1e-9;

    for (int m_ind2 = m_ind + 1; m_ind2 < _num_matches; m_ind2++) {
      FeatureMatch& match2 = _matches[m_ind2];
      assert(match2.id == m_ind2);
      const Eigen::Vector4d& ref_xyzw_2 = match2.ref_keypoint->xyzw;
      const Eigen::Vector4d& target_xyzw_2 = match2.refined_target_keypoint.xyzw;
      const Eigen::Vector3d& ref_xyz_2 = match2.ref_keypoint->xyz;
      const Eigen::Vector3d& target_xyz_2 = match2.refined_target_keypoint.xyz;

      bool consistent;
      // special case:  if either of the features are points at infinity, then
      // we can't compare their distances.
      if((ref_infinity && ref_xyzw_2.w() < 1e-9) ||
         (target_infinity && target_xyzw_2.w() < 1e-9)) {
        consistent = true;
      } else {
#ifdef USE_ROBUST_STEREO_COMPATIBILITY
        if (have_baseline) { // will not go into this part
          consistent = robustStereoCompatibility(ref_xyz_1, ref_xyz_2,
                                                 target_xyz_1 ,target_xyz_2,
                                                 baseline, stereo_focal_length,
                                                 _clique_inlier_threshold);
        } else {
          double ref_dist = (ref_xyz_2 - ref_xyz_1).norm();
          double target_dist = (target_xyz_2 - target_xyz_1).norm();
          consistent = fabs(ref_dist - target_dist) < _clique_inlier_threshold;
        }
#else
        double ref_dist = (ref_xyz_2 - ref_xyz_1).norm();
        double target_dist = (target_xyz_2 - target_xyz_1).norm();
        consistent = fabs(ref_dist - target_dist) < _clique_inlier_threshold;
#endif
      }

      if (consistent) {
        match.consistency_vec[match2.id] = 1;
        match.compatibility_degree++;
        match2.consistency_vec[match.id] = 1;
        match2.compatibility_degree++;
      }
    }
  }

  // sort the features based on their consistency with other features
  std::sort(_matches, _matches+_num_matches, consistencyCompare);

  // pick the best feature and mark it as an inlier
  FeatureMatch &best_candidate = _matches[0];
  best_candidate.in_maximal_clique = true;
  best_candidate.inlier = true;
  _num_inliers = 1;

  // start a list of quick-reject features (features that are known to be
  // inconsistent with any of the existing inliers)
  int reject[_num_matches];
  std::fill(reject, reject+_num_matches, 0);
  for (int m_ind = 1; m_ind < _num_matches; m_ind++) {
    int other_id = _matches[m_ind].id;
    if (!best_candidate.consistency_vec[other_id])
      reject[other_id] = 1;
  }

  // now start adding inliers that are consistent with all existing
  // inliers
  for (int m_ind = 1; m_ind < _num_matches; m_ind++) {
    FeatureMatch& cand = _matches[m_ind];

    // if this candidate is consistent with fewer than the existing number
    // of inliers, then immediately stop iterating since no more features can
    // be inliers
    if (cand.compatibility_degree < _num_inliers)
      break;

    // skip if it's a quick reject
    if (reject[cand.id])
      continue;

    cand.in_maximal_clique = true;
    cand.inlier = true;
    _num_inliers++;

    // mark some more features for rejection
    for (int r_ind = m_ind + 1; r_ind < _num_matches; r_ind++) {
      int other_id = _matches[r_ind].id;
      if (!reject[other_id] && !cand.consistency_vec[other_id])
        reject[other_id] = 1;
    }
  }
}

void MotionEstimator::estimateRigidBodyTransform() {
  _motion_estimate->setIdentity();

  if (_num_inliers < _min_features_for_valid_motion_estimate) {
    _estimate_status = INSUFFICIENT_INLIERS;
    return;
  }

  // gather all the inliers into two big matrices
  Eigen::MatrixXd target_xyz(3, _num_inliers);
  Eigen::MatrixXd ref_xyz(3, _num_inliers);

  int i = 0;
  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    FeatureMatch& match = _matches[m_ind];
    if (!match.inlier)
      continue;
    // FIXME so that this works with points at infinity
    target_xyz.col(i) = match.refined_target_keypoint.xyzw.head<3>() /
        match.refined_target_keypoint.xyzw(3) ;
    ref_xyz.col(i) = match.ref_keypoint->xyzw.head<3>() /
       match.ref_keypoint->xyzw(3);
    i++;
  }

#ifdef USE_HORN_ABSOLUTE_ORIENTATION
  if (0 != absolute_orientation_horn(target_xyz, ref_xyz, _motion_estimate)) {
    _estimate_status = OPTIMIZATION_FAILURE;
    return;
  }
#else
  Eigen::Matrix4d ume_estimate = Eigen::umeyama(target_xyz, ref_xyz);
  *_motion_estimate = Eigen::Isometry3d(ume_estimate);
#endif

  _estimate_status = SUCCESS;
}

void MotionEstimator::refineMotionEstimate()
{

  if (_num_inliers < _min_features_for_valid_motion_estimate) {
    _estimate_status = INSUFFICIENT_INLIERS;
    return;
  }

#ifdef USE_BIDIRECTIONAL_REFINEMENT
  // gather all the inliers into matrices
  Eigen::MatrixXd target_xyz(4,_num_inliers);
  Eigen::MatrixXd target_projections(2,_num_inliers);
  Eigen::MatrixXd ref_xyz(4,_num_inliers);
  Eigen::MatrixXd ref_projections(2,_num_inliers);
  int i = 0;
  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    FeatureMatch& match = _matches[m_ind];
    if (!match.inlier)
      continue;
    target_xyz.col(i) = match.refined_target_keypoint.xyzw;
    target_projections.col(i) = match.refined_target_keypoint.rect_base_uv;

    ref_xyz.col(i) = match.ref_keypoint->xyzw;
    ref_projections.col(i) = match.ref_keypoint->rect_base_uv;
    i++;
  }

  const IntrinsicParameters& rparams = camera_->GetRectifiedCameraParameters();

  // refine motion estimate by minimizing bidirectional reprojection error.
  // bidirectional reprojection error is the error of the target features
  // projected into the reference image, along with the reference features
  // projected into the target image
  refineMotionEstimateBidirectional(ref_xyz,
          ref_projections,
          target_xyz,
          target_projections,
          rparams.fx,
          rparams.cx,
          rparams.cy,
          *_motion_estimate,
          6,
          _motion_estimate,
          _motion_estimate_covariance);

  // TODO regularize the motion estimate covariance.
#else
  // gather all the inliers into matrices
  Eigen::MatrixXd target_xyz(4,_num_inliers);
  Eigen::MatrixXd ref_projections(2,_num_inliers);
  int i = 0;
  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    FeatureMatch& match = _matches[m_ind];
    if (!match.inlier)
      continue;
    target_xyz.col(i) = match.refined_target_keypoint.xyzw;

    ref_projections.col(i) = match.ref_keypoint->rect_base_uv;
    i++;
  }

  const CameraIntrinsicsParameters& rparams = camera_->GetRectifiedCameraParameters();
  // refine motion estimate by minimizing reprojection error of the
  // target features projected into the reference image.
  *_motion_estimate = refineMotionEstimate(target_xyz,
          ref_projections,
          rparams->fx,
          rparams->cx,
          rparams->cy,
          *_motion_estimate,
          6);
#endif
}

void MotionEstimator::computeReprojectionError()
{
  if (_estimate_status != SUCCESS)
    return;

  Eigen::Matrix<double, 3, 4> proj_matrix =
    camera_->GetRectifiedCameraParameters().toProjectionMatrix();
  Eigen::Matrix<double, 3, 4> reproj_matrix =
      proj_matrix * _motion_estimate->matrix();

  _mean_reprojection_error = 0;

  for (int m_ind = 0; m_ind < _num_matches; m_ind++) {
    FeatureMatch& match = _matches[m_ind];
    if (!match.inlier) {
      continue;
    }

    Eigen::Vector3d reproj_homogeneous =
        reproj_matrix * match.refined_target_keypoint.xyzw;

    Eigen::Vector2d reproj(reproj_homogeneous(0) / reproj_homogeneous(2),
        reproj_homogeneous(1) / reproj_homogeneous(2));

    Eigen::Vector2d err = match.ref_keypoint->rect_base_uv - reproj;

    match.reprojection_error = err.norm();

    //    printf("%3d:  %6.1f, %6.1f, %6.1f -> %6.1f, %6.1f -> %6.2f\n", m_ind,
    //        transformed_xyzw(0), transformed_xyzw(1), transformed_xyzw(2),
    //        reprojected_x, reprojected_y,
    //        match.reprojection_error
    //        );
    _mean_reprojection_error += match.reprojection_error;
  }
  _mean_reprojection_error /= _num_inliers;
}


static inline
int dot_int16_aligned(const int16_t* a, const int16_t* b, int num_taps) {
  assert(IS_ALIGNED16(a) && IS_ALIGNED16(b));
  const int16_t* ap = a;
  const int16_t* bp = b;
  __m128i tmp;
  __m128i ma = _mm_load_si128((const __m128i *) a);
  __m128i mb = _mm_load_si128((const __m128i *) b);
  __m128i hi = _mm_mulhi_epi16(ma, mb);
  __m128i lo = _mm_mullo_epi16(ma, mb);
  ma = _mm_unpackhi_epi16(lo, hi);
  mb = _mm_unpacklo_epi16(lo, hi);
  __m128i sums = _mm_add_epi32(ma, mb);
  ap += 8;
  bp += 8;
  for (int i = 1; i < num_taps; i++) {
    ma = _mm_load_si128((const __m128i *) ap);
    mb = _mm_load_si128((const __m128i *) bp);
    hi = _mm_mulhi_epi16(ma, mb);
    lo = _mm_mullo_epi16(ma, mb);
    ma = _mm_unpackhi_epi16(lo, hi);
    mb = _mm_unpacklo_epi16(lo, hi);
    sums = _mm_add_epi32(sums, ma);
    sums = _mm_add_epi32(sums, mb);
    ap += 8;
    bp += 8;
  }
  tmp = _mm_shuffle_epi32(sums, _MM_SHUFFLE(0, 1, 2, 3));
  sums = _mm_add_epi32(sums, tmp);
  tmp = _mm_shuffle_epi32(sums, _MM_SHUFFLE(1, 0, 0, 1));
  sums = _mm_add_epi32(sums, tmp);
  return _mm_cvtsi128_si32(sums);
}

void MotionEstimator::
refineFeatureMatch(PyramidLevel* ref_level,
                   PyramidLevel* target_level,
                   Eigen::Vector2d ref_uv,
                   Eigen::Vector2d init_target_uv,
                   Eigen::Vector2d * final_target_uv,
                   float *delta_sse) {
  // get the reference descriptor
  const uint8_t* ref_gray = ref_level->GetGrayscaleImage();
  int ref_gray_stride = ref_level->GetGrayscaleImageStride();

  int desc_len = ref_level->GetDescriptorLength();
  int desc_stride = ref_level->GetDescriptorStride();

  uint8_t ref_descriptor[desc_stride] __attribute__ ((aligned (16)));
  // ref_level->populateDescriptorInterp(ref_uv.x(), ref_uv.y(), ref_descriptor);
  ref_level->ExtractInterpDescriptor(ref_uv.x(), ref_uv.y(), ref_descriptor);

  int buf_num_bytes = round_up_to_multiple(desc_len * sizeof(int16_t), 16);
  int buf_num_elements = buf_num_bytes / sizeof(int16_t);
  int buf_num_pad_bytes = (buf_num_elements - desc_len) * sizeof(int16_t);

  // how many SSE operations does it take to compute a dot product?  Each
  // operation works on 8 elements at a time.
  int dot_prod_num_taps = buf_num_elements / 8;

  // initialize the target descriptor
  uint8_t orig_target_desc[desc_stride] __attribute__ ((aligned (16)));
  // target_level->populateDescriptorInterp(init_target_uv.x(), init_target_uv.y(),
  //                                        orig_target_desc);
  target_level->ExtractInterpDescriptor(init_target_uv.x(), init_target_uv.y(),
                                        orig_target_desc);
  uint8_t tgt_desc[desc_stride] __attribute__ ((aligned (16)));
  memcpy(tgt_desc, orig_target_desc, desc_stride);

  float tx = init_target_uv.x();
  float ty = init_target_uv.y();

  const uint8_t* target_gray = target_level->GetGrayscaleImage();
  int tgt_gray_stride = target_level->GetGrayscaleImageStride();

  int16_t pix_errs[buf_num_elements] __attribute__ ((aligned (16)));
  memset(pix_errs + desc_len, 0, buf_num_pad_bytes);

  // compute an initial error
  for (int i = 0; i < desc_len; i++) {
    pix_errs[i] = tgt_desc[i] - ref_descriptor[i];
  }
  int initial_sse = dot_int16_aligned(pix_errs, pix_errs, dot_prod_num_taps);

  int32_t final_sse = initial_sse;

  // Minimization using Efficient Second-order Minimization (ESM) method
  // described in:
  //   Selim Benhimane and Ezio Malis, "Real-time image-based tracking of
  //   planes using Efficient Second-order Minimization", IROS 2004

  // compute reference image gradients
  int16_t ref_desc_dx[buf_num_elements] __attribute__ ((aligned (16)));
  int16_t ref_desc_dy[buf_num_elements] __attribute__ ((aligned (16)));
  memset(ref_desc_dx + desc_len, 0, buf_num_pad_bytes);
  memset(ref_desc_dy + desc_len, 0, buf_num_pad_bytes);
  int rdesc_offset = ref_uv.y() * ref_gray_stride + ref_uv.x();
  const int* ref_desc_offsets = ref_level->GetDescriptorIndexOffsets();
  for (int i = 0; i < desc_len; i++) {
    int k = rdesc_offset + ref_desc_offsets[i];
    ref_desc_dx[i] = ref_gray[k + 1] - ref_gray[k];
    ref_desc_dy[i] = ref_gray[k + ref_gray_stride] - ref_gray[k];
    assert(k + ref_gray_stride < ref_level->GetHeight() * ref_gray_stride);
  }

  int16_t Mx[buf_num_elements] __attribute__ ((aligned (16)));
  int16_t My[buf_num_elements] __attribute__ ((aligned (16)));
  memset(Mx + desc_len, 0, buf_num_pad_bytes);
  memset(My + desc_len, 0, buf_num_pad_bytes);

  int max_iterations = 6;
  const int* tgt_desc_offsets = target_level->GetDescriptorIndexOffsets();
  for (int iter_num = 0; iter_num < max_iterations; iter_num++) {
    // compute target image gradients at current position and
    // M = sum of Jacobians at reference and current positions
    int tdesc_center = ((int) ty) * tgt_gray_stride + (int) tx;
    for (int i = 0; i < desc_len; i++) {
      int k = tdesc_center + tgt_desc_offsets[i];
      Mx[i] = ref_desc_dx[i] + (target_gray[k + 1] - target_gray[k]);
      My[i] = ref_desc_dy[i] + (target_gray[k + tgt_gray_stride] - target_gray[k]);
    }

    // S = M'*M
    double S_00 = dot_int16_aligned(Mx, Mx, dot_prod_num_taps);
    double S_01 = dot_int16_aligned(Mx, My, dot_prod_num_taps);
    double S_11 = dot_int16_aligned(My, My, dot_prod_num_taps);

    // S^{-1}
    double det = S_00 * S_11 - S_01 * S_01;
    if (det <= 1e-9)
      break;
    double Sinv_00 = S_11 / det;
    double Sinv_01 = -S_01 / det;
    double Sinv_11 = S_00 / det;

    // M'*err
    double gx = dot_int16_aligned(Mx, pix_errs, dot_prod_num_taps);
    double gy = dot_int16_aligned(My, pix_errs, dot_prod_num_taps);

    // M^{+} = pseudoinverse of M
    // delta = -2 * M^{+} * err
    double delta_x = -2 * (Sinv_00 * gx + Sinv_01 * gy);
    double delta_y = -2 * (Sinv_01 * gx + Sinv_11 * gy);

    float next_tx = tx + delta_x;
    float next_ty = ty + delta_y;

    // stop if the keypoint is about to go out of bounds
    if (!target_level->isLegalKeypointCoordinate(next_tx, next_ty))
      break;

    // compute shifted target descriptor and error
    // target_level->populateDescriptorInterp(next_tx, next_ty, tgt_desc);
    target_level->ExtractInterpDescriptor(next_tx, next_ty, tgt_desc);

    for (int i = 0; i < desc_len; i++)
      pix_errs[i] = tgt_desc[i] - ref_descriptor[i];
    int sse = dot_int16_aligned(pix_errs, pix_errs, dot_prod_num_taps);

    if (sse < final_sse) {
      // only step if the error decreases.
      tx = next_tx;
      ty = next_ty;
      final_sse = sse;
    } else {
      // if stepping would increase the error, then just give up.
      // TODO modify damping parameters instead?  Might not be worth it
      break;
    }

    // stop if we're not moving that much
    if (fabs(delta_x) < 0.1 && fabs(delta_y) < 0.1)
      break;
  }

  *final_target_uv = Eigen::Vector2d(tx, ty);
  *delta_sse = final_sse - initial_sse;
}


static inline Eigen::Isometry3d
isometryFromParams(const Eigen::Matrix<double, 6, 1>& params)
{
  Eigen::Isometry3d result;

  double roll = params(3), pitch = params(4), yaw = params(5);
  double halfroll = roll / 2;
  double halfpitch = pitch / 2;
  double halfyaw = yaw / 2;
  double sin_r2 = sin(halfroll);
  double sin_p2 = sin(halfpitch);
  double sin_y2 = sin(halfyaw);
  double cos_r2 = cos(halfroll);
  double cos_p2 = cos(halfpitch);
  double cos_y2 = cos(halfyaw);

  Eigen::Quaterniond quat(
    cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2,
    sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2,
    cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2,
    cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2);

  result.setIdentity();
  result.translate(params.head<3>());
  result.rotate(quat);

  return result;
}

static inline Eigen::Matrix<double, 6, 1>
isometryToParams(const Eigen::Isometry3d& M)
{
  Eigen::Quaterniond q(M.rotation());
  double roll_a = 2 * (q.w()*q.x() + q.y()*q.z());
  double roll_b = 1 - 2 * (q.x()*q.x() + q.y()*q.y());
  double pitch_sin = 2 * (q.w()*q.y() - q.z()*q.x());
  double yaw_a = 2 * (q.w()*q.z() + q.x()*q.y());
  double yaw_b = 1 - 2 * (q.y()*q.y() + q.z()*q.z());

  Eigen::Matrix<double, 6, 1> result;
  result.head<3>() = M.translation();
  result(3) = atan2(roll_a, roll_b);
  result(4) = asin(pitch_sin);
  result(5) = atan2(yaw_a, yaw_b);
  return result;
}

void MotionEstimator::
computeReprojectionError(
    const Eigen::Matrix<double, 4, Eigen::Dynamic>& points,
    const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_projections,
    const Eigen::Matrix<double, 3, 4>& K,
    const Eigen::Isometry3d& motion,
    Eigen::VectorXd* err,
    int err_offset) {
  int num_points = points.cols();
  assert(((err->size() == num_points * 2) && (err_offset == 0)) ||
         ((err->size() == num_points * 4) && ((err_offset == 0) || (err_offset == num_points*2))));
  assert(num_points == ref_projections.cols());

  Eigen::Matrix<double, 3, 4> P = K * motion.matrix();

  for(int pind=0; pind < num_points; pind++) {
    Eigen::Vector3d uvw = P * points.col(pind);
    double u = uvw(0) / uvw(2);
    double v = uvw(1) / uvw(2);
    (*err)(err_offset + pind*2 + 0) = u - ref_projections(0, pind);
    (*err)(err_offset + pind*2 + 1) = v - ref_projections(1, pind);
  }
}


Eigen::Isometry3d MotionEstimator::
refineMotionEstimate(
        const Eigen::Matrix<double, 4, Eigen::Dynamic>& points,
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_projections,
        double fx, double px, double py,
        const Eigen::Isometry3d& initial_estimate,
        int max_iterations) {
  Eigen::Matrix<double, 3, 4> xyz_c_to_uvw_c;
  xyz_c_to_uvw_c << fx, 0, px, 0,
                 0, fx, py, 0,
                 0, 0, 1, 0;

  Eigen::Isometry3d estimate = initial_estimate;
  Eigen::Matrix<double, 6, 1> estimate_vec = isometryToParams(estimate);

  int num_points = points.cols();

  // compute initial reprojection error
  Eigen::VectorXd err(num_points*2);
  computeReprojectionError(
    points, ref_projections, xyz_c_to_uvw_c, estimate,
    &err, 0);

  double initial_sse = err.cwiseProduct(err).sum();
  double final_sse = initial_sse;

  // TODO use a non-zero lambda value for actual levenberg-marquadt refinement
  // double lambda = 0;
  Eigen::Matrix<double, Eigen::Dynamic, 6> M(num_points*2, 6);

  for(int iter_num=0; iter_num<max_iterations; iter_num++) {
    computeProjectionJacobian(estimate_vec, fx, px, py, points, &M, 0);

    // Gauss-Newton:
    //    delta = -(pseudoinverse of M) * error
    //          = - inv(M'*M) * M' * error
    // Levenberg-Marquadt
    //    delta = - inv(M'*M + lambda * diag(M'*M)) * M' * error

    Eigen::Matrix<double, 6, 1> g = M.transpose() * err;
    Eigen::Matrix<double, 6, 6> MtM = M.transpose() * M;

//    Eigen::Matrix<double, 6, 6> diag(MtM.diagonal().asDiagonal());
//    MtM += lambda * diag;

    Eigen::Matrix<double, 6, 1> delta = - MtM.inverse() * g;

    // compute new isometry estimate
    Eigen::Matrix<double, 6, 1> next_params = estimate_vec + delta;
    Eigen::Isometry3d next_estimate = isometryFromParams(next_params);

    // compute reprojection error at new estimate
    computeReprojectionError(
      points, ref_projections, xyz_c_to_uvw_c, next_estimate, &err, 0);
    double sse = err.cwiseProduct(err).sum();

    // if stepping would increase the error, then just give up.
    if(sse > final_sse)
      break;

    // update estimate parameters and error values
    estimate_vec = next_params;
    estimate = next_estimate;
    final_sse = sse;

    // stop if we're not moving that much
    if(fabs(delta(0)) < 0.0001 &&
       fabs(delta(1)) < 0.0001 &&
       fabs(delta(2)) < 0.0001 &&
       fabs(delta(3)) < (0.01 * M_PI/180) &&
       fabs(delta(4)) < (0.01 * M_PI/180) &&
       fabs(delta(5)) < (0.01 * M_PI/180))
        break;
  }

  return estimate;
}

void
MotionEstimator::
computeProjectionJacobian(
    const Eigen::Matrix<double, 6, 1>& params,
    double fx, double px, double py,
    const Eigen::Matrix<double, 4, Eigen::Dynamic>& points,
    Eigen::Matrix<double, Eigen::Dynamic, 6> *result,
    int result_row_offset) {
  double tx    = params(0);
  double ty    = params(1);
  double tz    = params(2);
  double roll  = params(3);
  double pitch = params(4);
  double yaw   = params(5);
  double sr   = sin(roll);
  double cr   = cos(roll);
  double sp   = sin(pitch);
  double cp   = cos(pitch);
  double sy   = sin(yaw);
  double cy   = cos(yaw);

  // projection matrix
  Eigen::Matrix<double, 3, 4> P;
  P <<
    fx*cp*cy - px*sp, px*cp*sr - fx*(cr*sy - cy*sp*sr), fx*(sr*sy + cr*cy*sp) + px*cp*cr, fx*tx + px*tz,
    fx*cp*sy - py*sp, fx*(cr*cy + sp*sr*sy) + py*cp*sr, py*cp*cr - fx*(cy*sr - cr*sp*sy), fx*ty + py*tz,
    -sp, cp*sr, cp*cr, tz;

  // Jacobian matrices for homogeneous coordinates of point projections
  // thank you matlab
  Eigen::Matrix<double, 6, 4> Ju, Jv, Jw;
  Ju << 0, 0, 0, fx,
     0, 0, 0, 0,
     0, 0, 0, px,
     0, fx*(sr*sy + cr*cy*sp) + px*cp*cr,   fx*(cr*sy - cy*sp*sr) - px*cp*sr, 0,
     - px*cp - fx*cy*sp, -sr*(px*sp - fx*cp*cy), -cr*(px*sp - fx*cp*cy), 0,
     -fx*cp*sy, - fx*cr*cy - fx*sp*sr*sy, fx*cy*sr - fx*cr*sp*sy, 0;
  Jv << 0, 0, 0, 0,
     0, 0, 0, fx,
     0, 0, 0, py,
     0, py*cp*cr - fx*(cy*sr - cr*sp*sy), - fx*(cr*cy + sp*sr*sy) - py*cp*sr, 0,
     - py*cp - fx*sp*sy, -sr*(py*sp - fx*cp*sy), -cr*(py*sp - fx*cp*sy), 0,
     fx*cp*cy,   fx*cy*sp*sr - fx*cr*sy, fx*sr*sy + fx*cr*cy*sp, 0;
  Jw << 0, 0, 0, 0,
     0, 0, 0, 0,
     0, 0, 0, 1,
     0, cp*cr, -cp*sr, 0,
     -cp, -sp*sr, -cr*sp, 0,
     0, 0, 0, 0;

  int num_points = points.cols();
//  Eigen::Matrix<double, Eigen::Dynamic, 6> result(num_points*2, 6);
  assert(result->rows() == num_points*4 || result->rows() == num_points*2);
  assert(result->cols() == 6);
  assert(result_row_offset == 0);
  int row = 0;
  for(int i=0; i<num_points; i++) {
    const Eigen::Vector4d& point = points.col(i);
    Eigen::Vector3d uvw = P * point;
    Eigen::Matrix<double, 6, 1> du = Ju * point;
    Eigen::Matrix<double, 6, 1> dv = Jv * point;
    Eigen::Matrix<double, 6, 1> dw = Jw * point;

    double w_inv_sq = 1 / (uvw(2) * uvw(2));

    result->row(result_row_offset + row) = (du * uvw(2) - dw * uvw(0)) * w_inv_sq;
    result->row(result_row_offset + row+1) = (dv * uvw(2) - dw * uvw(1)) * w_inv_sq;

    row += 2;
  }
}

void
MotionEstimator::
computeReverseProjectionJacobian(
    const Eigen::Matrix<double, 6, 1>& params,
    double fx, double px, double py,
    const Eigen::Matrix<double, 4, Eigen::Dynamic>& points,
    Eigen::Matrix<double, Eigen::Dynamic, 6>* result,
    int result_row_offset) {
  double tx    = params(0);
  double ty    = params(1);
  double tz    = params(2);
  double roll  = params(3);
  double pitch = params(4);
  double yaw   = params(5);
  double sr   = sin(roll);
  double cr   = cos(roll);
  double sp   = sin(pitch);
  double cp   = cos(pitch);
  double sy   = sin(yaw);
  double cy   = cos(yaw);

  // Reverse projection matrix.
  Eigen::Matrix<double, 3, 4> P;
  P << px*(sr*sy + cr*cy*sp) + fx*cp*cy,
       fx*cp*sy - px*(cy*sr - cr*sp*sy),
       px*cp*cr - fx*sp,
       - px*(tx*(sr*sy + cr*cy*sp) - ty*(cy*sr - cr*sp*sy) + tz*cp*cr) - fx*(tx*cp*cy - tz*sp + ty*cp*sy),

       py*(sr*sy + cr*cy*sp) - fx*(cr*sy - cy*sp*sr),
       fx*(cr*cy + sp*sr*sy) - py*(cy*sr - cr*sp*sy),
       cp*(py*cr + fx*sr),
       - py*(tx*(sr*sy + cr*cy*sp) - ty*(cy*sr - cr*sp*sy) + tz*cp*cr) - fx*(ty*(cr*cy + sp*sr*sy) - tx*(cr*sy - cy*sp*sr) + tz*cp*sr),

       sr*sy + cr*cy*sp,
       cr*sp*sy - cy*sr,
       cp*cr,
       ty*(cy*sr - cr*sp*sy) - tx*(sr*sy + cr*cy*sp) - tz*cp*cr;

  // Jacobian matrices for homogeneous coordinates of reverse point projections
  Eigen::Matrix<double, 6, 4> Ju, Jv, Jw;
  Ju << 0, 0, 0, - px*(sr*sy + cr*cy*sp) - fx*cp*cy,
        0, 0, 0, px*(cy*sr - cr*sp*sy) - fx*cp*sy,
        0, 0, 0, fx*sp - px*cp*cr,
        px*cr*sy - px*cy*sp*sr, - px*cr*cy - px*sp*sr*sy, -px*cp*sr, px*ty*(cr*cy + sp*sr*sy) - px*tx*(cr*sy - cy*sp*sr) + px*tz*cp*sr,
        -cy*(fx*sp - px*cp*cr), -sy*(fx*sp - px*cp*cr), - fx*cp - px*cr*sp, fx*(tz*cp + tx*cy*sp + ty*sp*sy) - px*(tx*cp*cr*cy - tz*cr*sp + ty*cp*cr*sy),
        px*(cy*sr - cr*sp*sy) - fx*cp*sy, px*(sr*sy + cr*cy*sp) + fx*cp*cy, 0, - fx*(ty*cp*cy - tx*cp*sy) - px*(tx*(cy*sr - cr*sp*sy) + ty*(sr*sy + cr*cy*sp));

 Jv << 0, 0, 0, fx*(cr*sy - cy*sp*sr) - py*(sr*sy + cr*cy*sp),
       0, 0, 0, py*(cy*sr - cr*sp*sy) - fx*(cr*cy + sp*sr*sy),
       0, 0, 0, -cp*(py*cr + fx*sr),
       fx*(sr*sy + cr*cy*sp) + py*(cr*sy - cy*sp*sr), - fx*(cy*sr - cr*sp*sy) - py*(cr*cy + sp*sr*sy), cp*(fx*cr - py*sr), py*(ty*(cr*cy + sp*sr*sy) - tx*(cr*sy - cy*sp*sr) + tz*cp*sr) - fx*(tx*(sr*sy + cr*cy*sp) - ty*(cy*sr - cr*sp*sy) + tz*cp*cr),
       cp*cy*(py*cr + fx*sr), cp*sy*(py*cr + fx*sr), -sp*(py*cr + fx*sr), -(py*cr + fx*sr)*(tx*cp*cy - tz*sp + ty*cp*sy),
       py*(cy*sr - cr*sp*sy) - fx*(cr*cy + sp*sr*sy), py*(sr*sy + cr*cy*sp) - fx*(cr*sy - cy*sp*sr), 0, fx*(tx*(cr*cy + sp*sr*sy) + ty*(cr*sy - cy*sp*sr)) - py*(tx*(cy*sr - cr*sp*sy) + ty*(sr*sy + cr*cy*sp));

  Jw << 0, 0, 0, - sr*sy - cr*cy*sp,
        0, 0, 0, cy*sr - cr*sp*sy,
        0, 0, 0, -cp*cr,
        cr*sy - cy*sp*sr, - cr*cy - sp*sr*sy, -cp*sr, ty*(cr*cy + sp*sr*sy) - tx*(cr*sy - cy*sp*sr) + tz*cp*sr,
        cp*cr*cy, cp*cr*sy, -cr*sp, -cr*(tx*cp*cy - tz*sp + ty*cp*sy),
        cy*sr - cr*sp*sy, sr*sy + cr*cy*sp, 0, - tx*(cy*sr - cr*sp*sy) - ty*(sr*sy + cr*cy*sp);

  int num_points = points.cols();
  assert(result->rows() == num_points*4 || result->rows() == num_points*2);
  assert(result->cols() == 6);
  assert(result_row_offset == 0 || result_row_offset == (result->rows() / 2));
//  Eigen::Matrix<double, Eigen::Dynamic, 6> result(num_points*2, 6);
  int row = 0;
  for(int i=0; i<num_points; i++) {
    const Eigen::Vector4d& point = points.col(i);
    Eigen::Vector3d uvw = P * point;
    Eigen::Matrix<double, 6, 1> du = Ju * point;
    Eigen::Matrix<double, 6, 1> dv = Jv * point;
    Eigen::Matrix<double, 6, 1> dw = Jw * point;

    double w_inv_sq = 1 / (uvw(2) * uvw(2));

    result->row(result_row_offset + row) = (du * uvw(2) - dw * uvw(0)) * w_inv_sq;
    result->row(result_row_offset + row+1) = (dv * uvw(2) - dw * uvw(1)) * w_inv_sq;

    row += 2;
  }
}

void MotionEstimator::
refineMotionEstimateBidirectional(
        const Eigen::Matrix<double, 4, Eigen::Dynamic>& ref_points,
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& ref_projections,
        const Eigen::Matrix<double, 4, Eigen::Dynamic>& target_points,
        const Eigen::Matrix<double, 2, Eigen::Dynamic>& target_projections,
        double fx,
        double px, double py,
        const Eigen::Isometry3d& initial_estimate,
        int max_iterations,
        Eigen::Isometry3d* result,
        Eigen::MatrixXd* result_covariance) {
  Eigen::Matrix<double, 3, 4> xyz_c_to_uvw_c;
  xyz_c_to_uvw_c << fx, 0, px, 0,
                 0, fx, py, 0,
                 0, 0, 1, 0;

  Eigen::Matrix<double, 6, 1> estimate_vec = isometryToParams(initial_estimate);

  int num_points = target_points.cols();

  // allocate space for reprojection error vector
  Eigen::VectorXd err(num_points*4);

  // reprojection error from target point cloud to reference image
  computeReprojectionError(
    target_points, ref_projections, xyz_c_to_uvw_c,
    initial_estimate, &err, 0);

  // reprojection error from reference point cloud to target image
  computeReprojectionError(
      ref_points, target_projections, xyz_c_to_uvw_c,
      initial_estimate.inverse(), &err, num_points*2);

#ifdef USE_ESM
  // is this the jacobian at true minimum?
  Eigen::Matrix<double, Eigen::Dynamic, 6> M_0(num_points*4, 6);
  Eigen::Matrix<double, 6, 1> zero_vec;
  zero_vec.setZero();
  computeProjectionJacobian(zero_vec, fx, px, py, ref_points, &M_0, 0);
  computeReverseProjectionJacobian(zero_vec, fx, px, py, target_points, &M_0, num_points*2);
#endif

  double initial_sse = err.cwiseProduct(err).sum();
  double final_sse = initial_sse;

  Eigen::Matrix<double, Eigen::Dynamic, 6> M(num_points * 4, 6);

  for(int iter_num=0; iter_num<max_iterations; iter_num++) {
    // jacobian of target point cloud projected on to reference image
    computeProjectionJacobian(estimate_vec, fx, px, py, target_points, &M, 0);

    // jacobian of reference point cloud projected on to target image
    computeReverseProjectionJacobian(estimate_vec, fx, px, py, ref_points, &M, num_points*2);

#ifdef USE_ESM
    // ESM:
    //    delta = - 2 * pseudoinverse(M + M_0) * error
    M += M_0;
    Eigen::Matrix<double, 6, 1> g = M.transpose() * err;
    Eigen::Matrix<double, 6, 6> MtM = M.transpose() * M;
    Eigen::Matrix<double, 6, 1> delta = - 2 * MtM.inverse() * g;
#else
    // Gauss-Newton:
    //    delta = -(pseudoinverse of M) * error
    //          = - inv(M'*M) * M' * error
    Eigen::Matrix<double, 6, 1> g = M.transpose() * err;
    Eigen::Matrix<double, 6, 6> MtM = M.transpose() * M;
    Eigen::Matrix<double, 6, 1> delta = - MtM.inverse() * g;
#endif

    // compute new isometry estimate
    Eigen::Matrix<double, 6, 1> next_params = estimate_vec + delta;
    Eigen::Isometry3d next_estimate = isometryFromParams(next_params);

    // compute reprojection error at new estimate
    computeReprojectionError(
        target_points, ref_projections, xyz_c_to_uvw_c, next_estimate,
        &err, 0);
    computeReprojectionError(
        ref_points, target_projections, xyz_c_to_uvw_c, next_estimate.inverse(),
        &err, num_points*2);
    double sse = err.cwiseProduct(err).sum();
    //std::cerr << iter_num << ": " << next_params.transpose() << " : " << sse << std::endl;

    // if stepping would increase the error, then just give up.
    if(sse > final_sse)
      break;

    // update estimate parameters and error values
    estimate_vec = next_params;
    final_sse = sse;
    *result = next_estimate;

    // stop if we're not moving that much
    if(fabs(delta(0)) < 0.0001 &&
       fabs(delta(1)) < 0.0001 &&
       fabs(delta(2)) < 0.0001 &&
       fabs(delta(3)) < (0.01 * M_PI/180) &&
       fabs(delta(4)) < (0.01 * M_PI/180) &&
       fabs(delta(5)) < (0.01 * M_PI/180))
        break;
  }

  // compute the motion estimate covariance.
  //
  // XXX: this assumes that the covariance of the target feature locations is
  // identity.  In the future, we should allow the user to pass in a covariance
  // matrix on the feature locations (or at least specify a covariance for each
  // feature), which would then factor into this covariance matrix computation
  // here.
  if(result_covariance) {
    computeProjectionJacobian(estimate_vec, fx, px, py, target_points, &M, 0);
    computeReverseProjectionJacobian(estimate_vec, fx, px, py, ref_points, &M, num_points*2);
#ifdef USE_ESM
    M += M_0;
    Eigen::Matrix<double, 6, 6> MtM_inv = (M.transpose() * M).inverse();
    *result_covariance = 4 * MtM_inv;
#else
    Eigen::Matrix<double, 6, 6> MtM_inv = (M.transpose() * M).inverse();
    *result_covariance = MtM_inv;
#endif
  }
}

Eigen::ArrayXf InitialHomographyEstimator::flattenMatrix(Eigen::MatrixXf &m)
{
  return Eigen::Map<Eigen::ArrayXf>(m.data(), m.rows() * m.cols());
}

static void
grayToEigen(const uint8_t * grayData, int width, int height, int stride,
    int downsampleFactor, Eigen::MatrixXf* result)
{
  Eigen::MatrixXf& eig_imf = *result;
  if (downsampleFactor > 0) {
    int cols = width >> downsampleFactor;
    int rows = height >> downsampleFactor;
    eig_imf = Eigen::MatrixXf::Zero(rows, cols);
    for (int y = 0; y < height; y++) {
      int ey = y >> downsampleFactor;
      for (int x = 0; x < width; x++) {
        int ex = x >> downsampleFactor;
        eig_imf(ey, ex) += grayData[y * stride + x];
      }
    }
    double pixelFactor = (1 << downsampleFactor);
    eig_imf /= pixelFactor * pixelFactor;
  }
  else {
    eig_imf.resize(height, width);
    const uint8_t* row_start = grayData;
    for(int row=0; row<height; row++) {
      for(int col=0; col<width; col++) {
        eig_imf(row, col) = row_start[col];
      }
      row_start += stride;
    }
  }
}

void InitialHomographyEstimator::
SetTestImage(const uint8_t * grayData,
             int32_t width,
             int32_t height,
             int32_t stride,
             int32_t downsampleFactor) {
  grayToEigen(grayData, width, height, stride, downsampleFactor, &testImage_);
}

void InitialHomographyEstimator::
SetTemplateImage(const uint8_t * grayData,
                 int32_t width,
                 int32_t height,
                 int32_t stride,
                 int32_t downsampleFactor) {
  grayToEigen(grayData, width, height, stride, downsampleFactor, &templateImage_);
  template_rows_ = templateImage_.rows();
  template_cols_ = templateImage_.cols();

  //compute template gradients
  Eigen::MatrixXf templateDx, templateDy;
  computeGradient(templateImage_, &templateDx, &templateDy);

  templateDxRow_ = flattenMatrix(templateDx);
  templateDyRow_ = flattenMatrix(templateDy);

  //setup the utility matrices
  Eigen::MatrixXf x = Eigen::VectorXf::LinSpaced(template_cols_, 0, template_cols_ - 1).transpose().replicate(template_rows_, 1);
  Eigen::MatrixXf y = Eigen::VectorXf::LinSpaced(template_rows_, 0, template_rows_ - 1).replicate(1, template_cols_);
  xx_ = flattenMatrix(x);
  yy_ = flattenMatrix(y);

  templatePoints_.resize(3, xx_.rows());
  templatePoints_.row(0) = xx_;
  templatePoints_.row(1) = yy_;
  templatePoints_.row(2).setOnes();

  xx_ += 1;
  yy_ += 1;

}

double InitialHomographyEstimator::computeError(const Eigen::MatrixXf &error)
{
  return error.norm() / sqrt((double) error.rows() * error.cols());
}

Eigen::Matrix3f InitialHomographyEstimator::Track(const Eigen::Matrix3f & initH, int nIters, double * finalRMS)
{

  double minError = INFINITY;
  Eigen::Matrix3f H = initH;
  Eigen::Matrix3f bestH = H;
  int bestIter = 0;
  int lastImproved = 0;
  double lastRMS = INFINITY;

  for (int iter = 0; iter < nIters; iter++) {
    Eigen::MatrixXf warpedHomogeneousPoints;
    warpedHomogeneousPoints = H * templatePoints_;

    warpedTestImage_ = constructWarpedImage(testImage_, warpedHomogeneousPoints);

    errorIm_ = warpedTestImage_ - templateImage_;
    Eigen::VectorXf errorRow;
    errorRow = flattenMatrix(errorIm_);

    double rmsError = computeError(errorRow);

    if (rmsError < minError) {
      minError = rmsError;
      bestH = H;
      bestIter = iter;
    }

    Eigen::MatrixXf warpedTestImageDx, warpedTestImageDy;
    computeGradient(warpedTestImage_, &warpedTestImageDx, &warpedTestImageDy);

    Eigen::ArrayXf warpedTestImageDxRow, warpedTestImageDyRow;
    warpedTestImageDxRow = flattenMatrix(warpedTestImageDx);
    warpedTestImageDyRow = flattenMatrix(warpedTestImageDy);

    Eigen::MatrixXf Jt = computeJacobian(templateDxRow_ + warpedTestImageDxRow, templateDyRow_ + warpedTestImageDyRow);

    //compute the psuedo-inverse
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(Jt, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXf sigma = svd.singularValues();
    Eigen::MatrixXf U = svd.matrixU();
    Eigen::MatrixXf V = svd.matrixV();
    int r = 0;
    for (r = 0; r < sigma.rows(); r++) { //singular values are in decreasing order
      if (sigma(r) < 1e-7) //TODO:better way to get the tolerance?
        break;
      else
        sigma(r) = 1.0 / sigma(r);
    }
    Eigen::MatrixXf Jt_plus;
    if (r == 0)
      Jt_plus = Eigen::MatrixXf::Zero(Jt.cols(), Jt.rows());
    else {
      Jt_plus = V.block(0, 0, V.rows(), r) * sigma.head(r).asDiagonal() * U.block(0, 0, U.rows(), r).transpose();
    }

    Eigen::VectorXf lie_d = -2 * Jt_plus * errorRow;

    H = H * lieToH(lie_d);

    if (rmsError < lastRMS)
      lastImproved = iter;


    if (lie_d.norm() < 1e-6 || (rmsError - minError > 3 && iter - bestIter > 2) || iter - bestIter > 4 || iter
        - lastImproved > 2) {
      break;
    }

    lastRMS = rmsError;

  }
  if (finalRMS != NULL)
    *finalRMS = minError;
  return bestH;

}

void InitialHomographyEstimator::computeGradient(const Eigen::MatrixXf &image, Eigen::MatrixXf *dxp, Eigen::MatrixXf *dyp)
{
  Eigen::MatrixXf & dx = *dxp;
  Eigen::MatrixXf & dy = *dyp;
  dx = Eigen::MatrixXf::Zero(image.rows(), image.cols());
  dy = Eigen::MatrixXf::Zero(image.rows(), image.cols());

  dx.block(0, 1, dx.rows(), dx.cols() - 2) = image.block(0, 2, dx.rows(), dx.cols() - 2) - image.block(0, 0, dx.rows(),
      dx.cols() - 2);
  //handle border

  dy.block(1, 0, dy.rows() - 2, dy.cols()) = image.block(2, 0, dy.rows() - 2, dy.cols()) - image.block(0, 0, dy.rows()
      - 2, dy.cols());
  //normalize
  dx /= 2.0;
  dy /= 2.0;

  //handle borders
  dx.col(0) = image.col(1) - image.col(0);
  dx.col(image.cols() - 1) = image.col(image.cols() - 1) - image.col(image.cols() - 2);
  dy.row(0) = image.row(1) - image.row(0);
  dy.row(image.rows() - 1) = image.row(image.rows() - 1) - image.row(image.rows() - 2);

}

Eigen::MatrixXf InitialHomographyEstimator::computeJacobian(const Eigen::ArrayXf &dx, const Eigen::ArrayXf &dy) const
{
  Eigen::MatrixXf Jt(dx.rows(), 3);
  Jt.col(0) = dx;
  Jt.col(1) = dy;
  Jt.col(2) = dx * yy_ - dy * xx_;
  return Jt;
}

Eigen::Matrix3f InitialHomographyEstimator::lieToH(const Eigen::VectorXf &lie) const
{
  //TODO: support more parameters?
  Eigen::Matrix3f M;
  M << 0, lie(2), lie(0),
      -lie(2), 0, lie(1),
       0, 0, 0;
  return M.exp();
}

Eigen::MatrixXf InitialHomographyEstimator::constructWarpedImage(const Eigen::MatrixXf &srcImage,
    const Eigen::MatrixXf &warpedPoints) const
{
  Eigen::MatrixXf warped = Eigen::MatrixXf(template_rows_, template_cols_);

  const double defaultValue = 128;
  //Bilinear interpolation
  for (int i = 0; i < warpedPoints.cols(); i++) {
    double val;
    Eigen::Vector2f pt = warpedPoints.col(i).head(2) / warpedPoints(2, i);
    Eigen::Vector2i fipt(floor(pt(0)), floor(pt(1)));
    Eigen::Vector2i cipt(ceil(pt(0)), ceil(pt(1)));
    if (0 <= pt(0) && pt(0) < srcImage.cols() - 1 && 0 <= pt(1) && pt(1) < srcImage.rows() - 1) {
      double x1 = pt(0) - fipt(0);
      double y1 = pt(1) - fipt(1);
      double x2 = 1 - x1;
      double y2 = 1 - y1;
      val = x2 * y2 * srcImage(fipt(1), fipt(0)) + x1 * y2 * srcImage(fipt(1), fipt(0) + 1) + x2 * y1 * srcImage(
          fipt(1) + 1, fipt(0)) + x1 * y1 * srcImage(fipt(1) + 1, fipt(0) + 1);

    }
    else if (0 <= fipt(0) && fipt(0) < srcImage.cols() && 0 <= fipt(1) && fipt(1) < srcImage.rows()) {
      val = srcImage(fipt(1), fipt(0));
    }
    else if (0 <= cipt(0) && cipt(0) < srcImage.cols() && 0 <= cipt(1) && cipt(1) < srcImage.rows()) {
      val = srcImage(cipt(1), cipt(0));
    }
    else
      val = defaultValue; //templateImage_(i / template_cols_, i % template_cols_); //default to the same as template, so error is 0

    warped(i % template_rows_, i / template_rows_) = val; //Eigen is Column-major
  }
  return warped;
}






}