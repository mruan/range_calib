
#include <ceres/ceres.h>

#include "error_models.hpp"

class RangeCalibProblem
{
public:

  void BuildProblem()
  {
    for(int i=0; i< _num_observations; ++i)
      {
	// 3 output: euclidean error
	// 4 rotation + 3 translation + 3 coordinates
	CostFunction* cf = new AutoDiffCostFunction<Euclidean3DError, 3, 4, 3, 3>(
			       new Euclidean3DError(_P[3*i], _P[3*i+1], _P[3*i+2]));
      }

    // If enabled use Huber's loss function.
    LossFunction* lf = flag_robustify ? new HuberLoss(1.0) : NULL;

    AddResidualBlock(cf, lf, camera, camera+4, P);

    if (FLAGS_use_quaternions && FLAGS_use_local_parameterization) {
      LocalParameterization* quaternion_parameterization =
	new QuaternionParameterization;
      for (int i = 0; i < bal_problem->num_cameras(); ++i) {
	problem->SetParameterization(cameras + camera_block_size * i,
				     quaternion_parameterization);
    }
  }
  }
};
