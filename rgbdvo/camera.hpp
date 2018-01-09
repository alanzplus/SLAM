#ifndef __RGBD_VO_CAMERA__
#define __RGBD_VO_CAMERA__
#include <cstdint>
#include <cinttypes>
#include <memory>

#include <Eigen/Geometry>

namespace rgbdvo {

struct IntrinsicParameters {
  IntrinsicParameters() :
    width(0), height(0), fx(0), fy(0), cx(0), cy(0),
    k1(0), k2(0), k3(0), p1(0), p2(0),
    depth_height(0), depth_width(0),
    h_fov(0.0), v_fov(0.0) {}

  // 3x4 Projection matrix
  // [ fx  0  cx  0]
  // [ 0  fy  cy  0]
  // [ 0   0   1  0]
  Eigen::Matrix<double, 3, 4> toProjectionMatrix() const {
    Eigen::Matrix<double, 3, 4> ret;
    ret <<
      fx,  0, cx, 0,
       0, fy, cy, 0,
       0,  0,  1, 0;
    return ret;
  }

  // image width and height
  int32_t width, height;
  int32_t depth_width, depth_height;

  // focal length along the X axis and the Y axis.
  double fx, fy;

  // X/Y-coordinate of the camera center of projection / principal point.
  double cx, cy;

  // distortion coefficients
  double k1, k2, k3;

  // tangential distortion coefficent
  double p1, p2;

  // FOV
  double h_fov, v_fov;
};


class Camera {
  public:
    explicit Camera(const IntrinsicParameters& camera_params);
    ~Camera();

    const IntrinsicParameters& GetInputCameraParameters() const {
      return input_cam_params_;
    }

    const IntrinsicParameters& GetRectifiedCameraParameters() const {
      return rectified_cam_params_;
    }

    // rectifyLookup -> Lookup
    void Lookup(int32_t u, int32_t v,
                Eigen::Vector2d* out_uv) const {
      assert(u >= 0 && v >= 0 &&
             u < input_cam_params_.width &&
             v < input_cam_params_.height);
      int32_t pixel_index = v * input_cam_params_.width + u;
      out_uv->x() = Xmap_[pixel_index];
      out_uv->y() = Ymap_[pixel_index];
    }

    // rectifyLookupByIndex -> LookupByIndex
    void LookupByIndex(int32_t idx,
                       Eigen::Vector2d* out_uv) const
    {
      out_uv->x() = Xmap_[idx];
      out_uv->y() = Ymap_[idx];
    }

    // rectifyBilinearLookup -> BilinearLookup
    void BilinearLookup(const Eigen::Vector2d& uv,
                        Eigen::Vector2d* out_uv) const {
      int32_t u = (int32_t)uv.x();
      int32_t v = (int32_t)uv.y();

      assert(u >= 0 && v >= 0 && u < input_cam_params_.width &&
             v < input_cam_params_.height);

      // weights
      float wright  = (uv.x() - u);
      float wbottom = (uv.y() - v);
      float w[4] = {
        (1 - wright) * (1 - wbottom),
        wright * (1 - wbottom),
        (1 - wright) * wbottom,
        wright * wbottom
      };

      int32_t ra_index = v * input_cam_params_.width + u;
      uint32_t neighbor_indices[4] = {
        static_cast<uint32_t>(ra_index),
        static_cast<uint32_t>(ra_index + 1),
        static_cast<uint32_t>(ra_index + input_cam_params_.width),
        static_cast<uint32_t>(ra_index + input_cam_params_.width + 1)
      };

      out_uv->x() = 0;
      out_uv->y() = 0;
      for(int32_t i = 0; i < 4; ++i) {
        out_uv->x() += w[i] * Xmap_[neighbor_indices[i]];
        out_uv->y() += w[i] * Ymap_[neighbor_indices[i]];
      }
    }

    Camera* MakeCopy() const;

    Camera(const Camera& rhs) = delete;
    Camera& operator=(const Camera& rhs) = delete;

  private:
    Camera() {}
    void MakeXYMap();
    IntrinsicParameters input_cam_params_;
    IntrinsicParameters rectified_cam_params_;
    std::unique_ptr<float[]> Xmap_;
    std::unique_ptr<float[]> Ymap_;
};

}
#endif