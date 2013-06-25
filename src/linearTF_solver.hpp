/*

  Solves for a Rigid Transformation using the SVD method
  Roughly based on the PCL implementation with some small tweaks (ie float -> double)
 
  This linear solution is used for further nonlinear optimization in the coming steps
*/

class LinearTfSovler
{
public:
  void SetSrcPoints(std::vector<double[3]>& in_cloud)
  {
    compute3DCentroid(in_cloud, centroid_src);
    
    demeanPointCloud(in_cloud, centroid_src, cloud_src_demean);
  }

  void SetTgtPoints(std::vector<double[3]>& out_cloud)
  {
    compute3DCentroid(out_cloud, centroid_src);

    demeanPointCloud(out_cloud, centroid_tgt, cloud_tgt_demean);
  }

  void EstimateTfSVD(double t[3], double q[4])
  {
    // Assemble the correlation matrix H = source * target'
    Eigen::Matrix3d H = (cloud_src_demean * cloud_tgt_demean.transpose ()).topLeftCorner<3, 3>();

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

    Eigen::Matrix3d R = v * u.transpose ();
    Eigen::Quaternion Q(R);
    q[0] = Q.w(); q[1] = Q.x(); q[2] = Q.y(); q[3] = Q.z();

    const Eigen::Vector3d Rc (R * centroid_src.head<3> ());
    Eigen::Vector3d T = centroid_tgt.head<3> () - Rc;
    t[0] = T(0);  t[1] = T(1);  t[2] = T(2);
  }

private:
  bool compute3DCentroid(std::vector<double[3]>& cloud, Eigen::Vector4d& centroid)
  {
    if (cloud.empty())
      return false;

    // Initialize to all 0
    centroid.setZero();

    for(int i=0; i< cloud.size(); ++i)
      {
	centroid[0] += cloud[i][0];
	centroid[1] += cloud[i][1];
	centroid[2] += cloud[i][2];
      }
    centroid[3] = 0.0;
    centroid /= static_cast<double>(cloud.size());
    return true;
  }

  bool demeanPointCloud(std::vector<double[3]>& cloud, Eigen::Vector4d& centroid, Eigen::MatrixXd &cloud_out)
  {
    size_t npts = cloud.size();

    cloud_out = Eigen::MatrixXf::Zero(4, npts); // keep the data aligned

    for(size_t i=0; i< npts; ++i)
      {
	Eigen::Vector4d pt;
	pt << cloud[i][0] << cloud[i][1] << cloud[i][2] << 0;
	cloud_out.block<4,1>(0, i) = pt - centroid;

	// Make sure zero out the 4th dimension (1 row, N cloumns)
	cloud_out.block(3, 0, 1, npts).setZero();
      }
  }

  Eigen::Vector4d centroid_src;
  Eigen::Vector4d centroid_tgt;
  Eigen::MatrixXf cloud_src_demean;
  Eigen::MatrixXf cloud_tgt_demean;
};
