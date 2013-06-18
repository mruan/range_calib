
#include <ceres/ceres.h>

using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

class DirectionalCostFunction
  :public SizedCostFunction<1, 3>
{
public:
  RayCostFunction(double x, double y, double z)
    :_x(x), _y(y), _z(z){}

  virtual ~RayCostFunction(){}

  virtual bool Evaluate(double const* const* params,
			double* res, double** jac) const
  {
    double& cx = params[0][0];
    double& cy = params[0][1];
    double& cz = params[0][2];

    double nr  = sqrt(_x*_x+_y*_y+_z*_z);
    double ux  = _x / nrm;
    double uy  = _y / nrm;
    double uz  = _z / nrm;

    double p   = cx * ux + cy * uy + cz * uz;
    double q2  = pow(uy*cz-uz*cy,2)+pow(uz*cx-ux-cz,2)+pow(ux*cy-uy*cx,2);
    double q   = sqrt(q);

    if (q < s_radius)
      res[0] = p - sqrt(s_radius*s_radius - q2) - nr;
    else
      res[0] = sqrt((p-nr)*(p-nr) + (q-s_radius)*(q-s_radius));

    if (jac != NULL && jac[0] != NULL)
      {
	if (q < s_radius)
	  {
	    double denom = sqrt(s_radius*s_radius-q2);
	    jac[0][0] = ux + (cx-ux*p)/denom;
	    jac[0][1] = uy + (cy-uy*p)/denom;
	    jac[0][2] = uz + (cz-uz*p)/denom;
	  }
	else
	  {
	    double pr = p-nr;
	    double qR = 1 - s_radius /q;
	    jac[0][0] = (pr*ux+ qR*(cx-ux*p))/res[0];
	    jac[0][1] = (pr*uy+ qR*(cy-uy*p))/res[0];
	    jac[0][2] = (pr*uz+ qR*(cz-uz*p))/res[0];
	  }
      }
  }

  static double s_radius;

private:
  const double _x;
  const double _y;
  const double _z;
};

class SphereFitter
{
public:
  SphereFitter(double r = 0.1275)
  {
    _options.max_num_iterations = 25;
    _options.linear_solver_type = ceres::DENSE_QR;
    _options.minimizer_progress_to_stdout = true;

    DirectionalCostFunction::s_radius = r;
  }

  template<typename PointT>
  void SetPointCloud(pcl::PointCloud<PointT>::Ptr cloud)
  {
    for (int i=0; i < cloud->points.size(); i++)
      {
	PointT& p = cloud->points[i];
	_problem.AddResidualBlock(new DirectionalCostFunction(p.x, p.y, p.z), 
				  NULL, &cx, &cy, &cz);
      }
  }

  bool FitSphere(double x, double y, double z)
  {
    Solve(options, &problem, &summary);

    x = cx;
    y = cy;
    z = cz;
    return true;
  }

private:
  double cx, cy, cz;
  Solver::Options _option;
  Solver::Summary _summary;
  Problem _problem;
};
