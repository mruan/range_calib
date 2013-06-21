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

int main(int argc, char** argv)
{
  if (argc < 2)
    {
      printf("Usage: %s <problem_config_file>\n", argv[0]);
      return -1;
    }

  std::ifstream fs(argv[1], std::ifstream::in);
  if(!fs.is_open())
    {
      std::cerr<< "Cannot open file: "<< std::string(argv[1])<< std::endl;
      return -1;
    }

  ResourceManager rm;
  ceres::Problem problem;
  ceres::Solver::Options options;

  if(!rm.ReadfromFile(fs))
    return -1;

  rm.BuildProblem(problem);

  // Minimizer options
  options.max_num_iterations = 25;
  options.minimizer_progress_to_stdout = true;
  options.num_threads = 1;
    
  // Linear solver options
  //  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.num_linear_solver_threads = 1;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << std::endl;

  return 0;
}
