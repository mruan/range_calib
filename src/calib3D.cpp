
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <stdio.h>

#include "error_models.hpp"

// A complete program consists 3 parts:
// 1. Data    -> observations and initial guesses
// 2. Problem -> Cost functions, loss functions
// 3. Solver  -> all the options

#define POINT_ND    3
#define CAMERA_ND   7 // 4 quat + 3 tran

using namespace ceres;

class RangeSensorCalibration
{
public:
  bool LoadData(std::string file)
  {
    FILE* fd = fopen(file.c_str(), "r");
    if (!fd) 
      { 
	printf("Error: unable to open file %s\n", file.c_str()); 
	return false;
      }

    // First line contains: Num_of_Cameras Num_of_Points Num_of_Observations
    fscanf(fd, " %d %d %d ", &num_cameras_, &num_points_, &num_observations_);

    point_index_  = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[POINT_ND * num_observations_];

    num_parameters_ = CAMERA_ND * num_cameras_ + POINT_ND * num_points_;
    parameters_ = new double[num_parameters_];

    for (int i = 0; i < num_observations_; ++i) 
      {
	fscanf(fd, " %d %d ", camera_index_ + i, point_index_ + i);
	double* offset = observations_ + POINT_ND*i;
	fscanf(fd, " %lf %lf %lf ", offset, offset+1, offset+2);
	// potentially add confidence
      }

    // Assume the rotations are parameterized by quaternions
    for (int i=0; i< num_parameters_; ++i)
      fscanf(fd, " %lf ", parameters_+i);

    fclose(fd);
  }

  void BuildProblem()
  {
    double* pPts = parameters_ + CAMERA_ND * num_cameras_;
    double* pCam = parameters_;

    for(int i=0; i< num_observations_; ++i)
      {
	// 3 euclidean error, 4 quaternion, 3 translation, 3 coordinates
	CostFunction* cf = new AutoDiffCostFunction<Euclidean3DError, 3, 4, 3, 3>( new Euclidean3DError(observations_[POINT_ND*i], observations_[POINT_ND*i+1], observations_[POINT_ND*i+2]));

	// If enabled use Huber's loss function.
	LossFunction* lf = new HuberLoss(1.0);// NULL;
	
	double* which_cam = pCam + camera_index_[i]*CAMERA_ND;
	double* which_pt  = pPts + point_index_[i]*POINT_ND;

	_problem.AddResidualBlock(cf, lf, which_cam, which_cam+4, which_pt);
      }
    
    // Set local parameterization for quaternion
    LocalParameterization* quat_param = new QuaternionParameterization;
    for (int i = 0; i < num_cameras_; ++i) 
      {
	_problem.SetParameterization(pCam + i*CAMERA_ND, quat_param);
      }
  }
  
  void SetOptions()
  {
    // Minimizer options
    _options.max_num_iterations = 25;
    _options.minimizer_progress_to_stdout = true;
    _options.num_threads = 1; // TODO: for safety now, use just one
    
    // Linear solver options
    _options.linear_solver_type = ceres::DENSE_SCHUR;
    _options.num_linear_solver_threads = 1;

    // Ordering options:
    double* pPts = parameters_ + CAMERA_ND * num_cameras_;
    double* pCam = parameters_;
    ParameterBlockOrdering* ordering = new ParameterBlockOrdering;
    // The points come before the cameras.
    for (int i = 0; i < num_points_; ++i) 
      {
	ordering->AddElementToGroup(pPts + POINT_ND * i, 0);
      }

    for (int i = 0; i < num_cameras_; ++i) 
      {
	// When using axis-angle, there is a single parameter block for
	// the entire camera.
	ordering->AddElementToGroup(pCam + CAMERA_ND * i, 1);
	// If quaternions are used, there are two blocks, so add the
	// second block to the ordering.
	ordering->AddElementToGroup(pCam + CAMERA_ND * i + 4, 1);
      }
    _options.linear_solver_ordering = ordering;
  }

  void RunOptimizer()
  {
    Solver::Summary summary;
    Solve(_options, &_problem, &summary);
    std::cout << summary.FullReport() << std::endl;
  }

  ~RangeSensorCalibration()
  {
    delete[] point_index_;
    delete[] camera_index_;
    delete[] observations_;
    delete[] parameters_;
  }

private:
  
  Problem _problem;
  Solver::Options _options;

  int num_cameras_;
  int num_points_;
  int num_observations_;
  int num_parameters_;

  int* point_index_;
  int* camera_index_;
  double* observations_;
  // The parameter vector is laid out as follows
  // [camera_1, ..., camera_n, point_1, ..., point_m]
  double* parameters_;
};

int main(int argc, char** argv)
{
  if(argc < 2)
    {
      printf("Usage: %s <problem_config_file>\n", argv[0]);
      return -1;
    }

  RangeSensorCalibration rsc;
  if(!rsc.LoadData(argv[1]))
    return -1;
  
  rsc.BuildProblem();
  rsc.SetOptions();

  rsc.RunOptimizer();

  return 0;
}
