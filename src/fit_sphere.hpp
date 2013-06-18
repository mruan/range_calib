
#include <ceres/ceres.h>
#include <unistd.h>

using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class RayCostFunction:public SizedCostFunction<1, 3>
{
public:
  RayCostFunction(double xj, double yj, double zj)
  {
    r  = sqrt(xj*xj+yj*yj+zj*zj);// + 1e-5
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
    double q   = sqrt(q2);

    if (q < R)
      res[0] = p - sqrt(R*R - q2) - r;
    else
      res[0] = sqrt((p-r)*(p-r) + (q-R)*(q-R));

    if (jac != NULL && jac[0] != NULL)
      {
	if (q < R)
	  {
	    double denom = sqrt(R*R - q2);
	    jac[0][0] = x + (X - x*p) /denom;
	    jac[0][1] = y + (Y - y*p) /denom;
	    jac[0][2] = z + (Z - z*p) /denom;
	  }
	else
	  {
	    double pr = p - r;
	    double qR = 1 - R/q;
	    jac[0][0] = (pr * x + qR *(X - x*p))/res[0];
	    jac[0][1] = (pr * y + qR *(Y - y*p))/res[0];
	    jac[0][2] = (pr * z + qR *(Z - z*p))/res[0];
	  }
      }

    // TODO: This printf() has to be there in order to prevent numerical failure
    // It's a strange bug that might need a semaphore.
    printf(" ");
    //    printf("%lf %lf %lf, %lf %lf, %lf\n", X, Y,Z, p, q, res[0]);
  }

  static void SetRadius(double radius) { R = radius;}
  static double R;
private:
  double r;
  double x, y, z;// x, y, z are normalized
};

class SphereFitter
{
public:
  SphereFitter(double x=0, double y=0, double z=0)
  {
    // Initial guesses
    _center[0] = x;
    _center[1] = y;
    _center[2] = z;

    _options.max_num_iterations = 25;
    _options.linear_solver_type = ceres::DENSE_QR;
    _options.minimizer_progress_to_stdout = true;
  }

  typedef pcl::PointXYZ T;
  void SetInputCloud(pcl::PointCloud<T>::Ptr cloud)
  {
    for (int i=0; i < cloud->points.size(); i++)
      {
	T& p = cloud->points[i];
	_problem.AddResidualBlock(new RayCostFunction(p.x, p.y, p.z), 
				  NULL, &_center[0]);
      }
  }

  //  template<typename T>
  void SetInputCloud(pcl::PointCloud<T>::Ptr cloud, const std::vector<bool>& mask)
  {
    for (int i=0; i < cloud->points.size(); i++)
      {
	if (!mask[i])
	  continue;

	T& p = cloud->points[i];
	_problem.AddResidualBlock(new RayCostFunction(p.x, p.y, p.z), 
				  NULL, &_center[0]);
      }
    //    _problem.AddParameterBlock(_center, 3);
  }

  bool FitSphere(double& x, double& y, double& z)
  {
    Solve(_options, &_problem, &_summary);

    std::cout << _summary.BriefReport() << std::endl;
    x = _center[0];
    y = _center[1];
    z = _center[2];
    return true;
  }

private:
  double _center[3];
  Solver::Options _options;
  Solver::Summary _summary;
  Problem _problem;
};
