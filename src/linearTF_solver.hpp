/*

  Solves for a Rigid Transformation using the SVD method
  Roughly based on the PCL implementation with some small tweaks (ie float -> double)
 
  This linear solution is used for further nonlinear optimization in the coming steps
*/

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include "types.hpp"

class LinearTfSolver
{
public:
  void SetRefPoints(std::vector<Point3d>& in_cloud)
  {
    compute3DCentroid(in_cloud, centroid_ref);
    
    demeanPointCloud(in_cloud, centroid_ref, cloud_ref_demean);
  }

  void SetTgtPoints(std::vector<Point3d>& out_cloud)
  {
    compute3DCentroid(out_cloud, centroid_tgt);

    demeanPointCloud(out_cloud, centroid_tgt, cloud_tgt_demean);
  }

  // Assume t = double[3], q = double[4]
  void EstimateTfSVD(double* t, double* q)
  {
    // Assemble the correlation matrix H = target * reference'
    Eigen::Matrix3d H = (cloud_tgt_demean * cloud_ref_demean.transpose ()).topLeftCorner<3, 3>();

    // Compute the Singular Value Decomposition
    Eigen::JacobiSVD<Eigen::Matrix3d> svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d u = svd.matrixU ();
    Eigen::Matrix3d v = svd.matrixV ();

    // Compute R = V * U'
    if (u.determinant () * v.determinant () < 0)
      {
	for (int i = 0; i < 3; ++i)
	  v (i, 2) *= -1;
      }

    //    std::cout<< "centroid_src: "<<centroid_src(0) <<" "<< centroid_src(1) <<" "<< centroid_src(2) << " "<< centroid_src(3)<<std::endl;
    //    std::cout<< "centroid_tgt: "<<centroid_tgt(0) <<" "<< centroid_tgt(1) <<" "<< centroid_tgt(2) << " "<< centroid_tgt(3)<<std::endl;
    
    Eigen::Matrix3d R = v * u.transpose ();

    const Eigen::Vector3d Rc (R * centroid_tgt.head<3> ());
    Eigen::Vector3d T = centroid_ref.head<3> () - Rc;

    // Make sure these memory locations are valid
    assert(t != NULL && q!=NULL);
    Eigen::Quaterniond Q(R);
    t[0] = T(0);  t[1] = T(1);  t[2] = T(2);
    q[0] = Q.w(); q[1] = Q.x(); q[2] = Q.y(); q[3] = Q.z();
  }

private:
  bool compute3DCentroid(std::vector<Point3d>& cloud, Eigen::Vector4d& centroid)
  {
    if (cloud.empty())
      return false;

    // Initialize to all 0
    centroid.setZero();

    for(int i=0; i< cloud.size(); ++i)
      {
	centroid[0] += cloud[i].x;
	centroid[1] += cloud[i].y;
	centroid[2] += cloud[i].z;
	//	printf("Point[%d]: %lf %lf %lf\n", cloud[i].x, cloud[i].y, cloud[i].z);
      }
    centroid[3] = 0.0;
    centroid /= static_cast<double>(cloud.size());
    return true;
  }

  bool demeanPointCloud(std::vector<Point3d>& cloud, 
			Eigen::Vector4d& centroid, Eigen::MatrixXd &cloud_out)
  {
    size_t npts = cloud.size();

    cloud_out = Eigen::MatrixXd::Zero(4, npts); // keep the data aligned

    for(size_t i=0; i< npts; ++i)
      {
	Eigen::Vector4d pt;
	pt << cloud[i].x, cloud[i].y, cloud[i].z, 0.0;
	cloud_out.block<4,1>(0, i) = pt - centroid;

	// Make sure zero out the 4th dimension (1 row, N cloumns)
	cloud_out.block(3, 0, 1, npts).setZero();
      }
  }

  Eigen::Vector4d centroid_ref;
  Eigen::Vector4d centroid_tgt;
  Eigen::MatrixXd cloud_ref_demean;
  Eigen::MatrixXd cloud_tgt_demean;
};
