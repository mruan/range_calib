
#ifndef ERROR_MODELS_H
#define ERROR_MODELS_H

// Templated range sensor model for use with Ceres.
// The camera is parameterized using 7 parameters: 4 for rotation+ 3 for translation
// i.e. only the extrinsics are being optimized.
struct Euclidean3DError
{
  Euclidean3DError(double ox, double oy, double oz)
    :_ox(ox), _oy(oy), _oz(oz){}

  template <typename T>
  bool operator()(const T* const cam_quat, const T* const cam_tran,
		  const T* const point, T* residuals) const
  {
    // Use a quaternion rotation that doesn't assume the quaternion is
    // normalized, since one of the ways to run the bundler is to let Ceres
    // optimize all 4 quaternion parameters unconstrained.    
    T p[3];
    ceres::QuaternionRotatePoint(cam_quat, point, p);

    p[0] += cam_tran[0];
    p[1] += cam_tran[1];
    p[2] += cam_tran[2];

    residual[0] = p[0] - T(_ox);
    residual[1] = p[1] - T(_oy);
    residual[2] = p[2] - T(_oz);
    return true;
  }

  double _ox, _oy, _oz; // observed (x, y, z)
};

#endif
