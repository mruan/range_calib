/*
  Author: Ming Ruan
  Date:   June 21
  Description:  A simple script to test the calibration
 */
#include <stdio.h>
#include <iostream>
#include <fstream>      // std::ifstream

#include <ceres/ceres.h>

#include "resource_manager.hpp"
#include "linearTF_solver.hpp"

int main(int argc, char** argv)
{
  if (argc < 2)
    {
      printf("Usage: %s <problem_config_file>\n", argv[0]);
      return -1;
    }

  std::ifstream ifs(argv[1], std::ifstream::in);
  if(!ifs.is_open())
    {
      std::cerr<< "Cannot open file: "<< std::string(argv[1])<< std::endl;
      return -1;
    }

  ResourceManager rm;
  ceres::Problem problem;
  ceres::Solver::Options options;

  if(!rm.ReadfromStream(ifs))
    return -1;
  ifs.close();

  LinearTfSolver ls;
  //  ls.SetSrcPoints(rm.GetLandmarks());
  // for each sensor in rm, solve a linear rigid transform
  for (int i=0; i< 3; i++)
    {
      //    ls.SetTgtPoints(rm.GetSensorObservations(i));
      double t[3], q[4];
      ls.EstimateTfSVD(t, q);
      rm.SetSensorTf(i, t, q);
    }

  rm.BuildCeresProblem(problem);

  // Minimizer options
  options.max_num_iterations = 25;
  options.minimizer_progress_to_stdout = true;
  //  options.num_threads = 1;
    
  // Linear solver options
  options.linear_solver_type = ceres::DENSE_SCHUR;
  //  options.num_linear_solver_threads = 1;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;
  

  std::ofstream ofs("out.txt", std::ofstream::out);
  rm.WriteToStream(ofs);
  ofs.close();

  return 0;
}
