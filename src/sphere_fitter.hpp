/*
  Class to fit a sphere to a blob of points. 
  It is critical to have a good initial guess
 */


#pragma once

#include <stdio.h>
#include <limits> // for +inf

#include <ceres/ceres.h>
#include <ceres/types.h>

using namespace ceres;

// This global variable must be declared once somewhere in main() function!
extern const double g_Radius; // global target ball's radius

// The (Directional) Ray cost function from [Marek09]
// It's robust for biased point cloud
class RayCostFunction:public SizedCostFunction<1, 3>
{
public:
  RayCostFunction(double xj, double yj, double zj)
  {
    r  = std::sqrt(xj*xj+yj*yj+zj*zj);// + 1e-5
    x  = xj / r;
    y  = yj / r;
    z  = zj / r;
    //    printf("r= %lf x=%lf y=%lf z=%lf\n", r, x, y, z);
  }

  virtual ~RayCostFunction(){}

  virtual bool Evaluate(double const* const* params,
			double* res, double** jac) const
  {
    const double& X = params[0][0];
    const double& Y = params[0][1];
    const double& Z = params[0][2];

    double p   = X*x + Y*y + Z*z;
    double q2  = pow(y*Z-z*Y,2)+pow(z*X-x*Z,2)+pow(x*Y-y*X,2);
    double q   = std::sqrt(q2);

    if (q < g_Radius)
      res[0] = p - std::sqrt(g_Radius*g_Radius - q2) - r;
    else
      res[0] = std::sqrt((p-r)*(p-r) + (q-g_Radius)*(q-g_Radius));

    if (jac != NULL && jac[0] != NULL)
      {
	if (q < g_Radius)
	  {
	    double denom = std::sqrt(g_Radius*g_Radius - q2);
	    jac[0][0] = x + (X - x*p) /denom;
	    jac[0][1] = y + (Y - y*p) /denom;
	    jac[0][2] = z + (Z - z*p) /denom;
	  }
	else
	  {
	    double pr = p - r;
	    double qR = 1 - g_Radius/q;
	    jac[0][0] = (pr * x + qR *(X - x*p))/res[0];
	    jac[0][1] = (pr * y + qR *(Y - y*p))/res[0];
	    jac[0][2] = (pr * z + qR *(Z - z*p))/res[0];
	  }
      }
    return true;
  }

private:
  double r;
  double x, y, z;// x, y, z are normalized
};

// Class Wrapper for cost function
class SphereFitter
{
public:
  SphereFitter(Point3d& init_guess)
  {
    // Set initial guess
    _center = init_guess;

    _options.max_num_iterations = 50;
    _options.linear_solver_type = ceres::DENSE_QR;
    //    _options.minimizer_progress_to_stdout = true;

    /*
    _options.num_threads = 1; // TODO: for safety now, use just one
    _options.num_linear_solver_threads = 1;

    */
  }

  typedef pcl::PointXYZ T;
  // Overloaded functions to add point clouds:
  // 1. Add the entire point cloud
  // 2. Add point cloud with mask
  // 3. Add point cloud with inlier list
  void SetInputCloud(pcl::PointCloud<T>::Ptr cloud)
  {
    for (int i=0; i < cloud->points.size(); i++)
      AddResidualBlock(cloud->points[i]);
  }

  void SetInputCloud(pcl::PointCloud<T>::Ptr cloud, const std::vector<bool>& mask)
  {
    for (int i=0; i < cloud->points.size(); i++)
      {
	if (mask[i])
	  AddResidualBlock(cloud->points[i]);
      }
  }

  void SetInputCloud(pcl::PointCloud<T>::Ptr cloud, const std::vector<int>& inlier)
  {
    for (int i=0; i < inlier.size(); i++)
      AddResidualBlock( cloud->points[inlier[i]]);
  }

  // Actually fit the sphere, also return the final cost
  bool FitSphere(Point3d& p)
  {
    std::cout << "Initial guess at["<<_center.x<<" ";
    std::cout << _center.y<<" "<< _center.z<<"]\n";

    Solver::Summary summary;
    Solve(_options, &_problem, &summary);

    // copy the results
    p = _center;

    std::cout << summary.BriefReport() << std::endl;

    if (summary.termination_type == FUNCTION_TOLERANCE ||
	summary.termination_type == GRADIENT_TOLERANCE ||
	summary.termination_type == PARAMETER_TOLERANCE)
      {
	std::cout << "Center found at ["<<p.x<<" "<<p.y<<" "<<p.z<<"]\n";
	return summary.final_cost;
      }
    else
      {
	std::cout << "Return the initial guess"<<std::endl;
	return std::numeric_limits<double>::max();
      }
  }

private:
  // helper function to add the residual block
  void AddResidualBlock(T& p)
  {
    // Robust loss function to deal with outliers
    LossFunction* lf = new HuberLoss(1.0);
    _problem.AddResidualBlock(new RayCostFunction(p.x,p.y,p.z),lf,&(_center.x));
  }

  Point3d _center;
  Solver::Options _options;
  Problem _problem;
};
