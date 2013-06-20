/*
  Author: Ming Ruan
  Date: June 17

  Filter out the sphere with a given radius in the point cloud
  It works much better (and faster) with background-subtracted data

 */

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class BallSelector
{
public:
  BallSelector(double r, double err)
    :_radius(r), _err(err){}

  void SetInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  { _cloud = cloud; _w = cloud->width; _h = cloud->height; _flt_mask.resize(_w*_h); }

  void GetCenter(double& x, double& y, double &z)
  { x = _center[0];  y = _center[1]; z = _center[2]; }

  const std::vector<bool>& GetMask()
  { return _flt_mask; }

  // This is still a rather stupid implementation of the idea
  // which should have exploited neighbors in image space
  float ComputeScore()
  {
    double cx, cy, cz; // center of sphere
    double dx, dy, dz; // difference
    double this_score = 0;
    double best_score = 0;//MIN_COUNT;// need at least this many to be a good fit
    int best_index = -1;
    // for every point
    for(int i=0; i< _w*_h; ++i)
      {
	// TODO: skip if _pre_mask is false;
	/*
	  if (!_pre_mask[i])
	  continue;
	*/
	pcl::PointXYZ& p = _cloud->points[i];

	// Assume the center is at this pixel
	// but pushed back by a radius r;
	cz = p.z + _radius;  cx = p.x;	cy = p.y;
      
	// reset the count;
	int this_count = 0;

	// count how many points potentially belong to
	// such a sphere
	// TODO: this is stupid, only need to search the neighborhood,
	for (int j=0; j < _w*_h; ++j)
	  {
	    pcl::PointXYZ& q = _cloud->points[j];
	    dx = q.x - cx;  dy = q.y - cy;  dz = q.z - cz;

	    if (isInlier(dx, dy, dz))
	      this_count++;
	  }

	// Percentage of inliers.
	this_score = this_count / (float) (_w*_h);

	if (this_score > best_score)
	  {
	    best_score = this_score;
	    best_index = i;
	  }
      }
    
    // At least half of the points are inliers
    if (best_index < 0.5)
      return -0.5; // negative value

    //  std::cout << "Found " << best_count << " inliers" << std::endl;

    // Build the mask:
    pcl::PointXYZ& p = _cloud->points[best_index];
  
    // Assume the center is at this pixel
    // but pushed back by a radius r;
    _center[2] = cz = p.z + _radius;
    _center[0] = cx = p.x;
    _center[1] = cy = p.y;  

    // TODO: Again this is stupid, only search the neighborhood
    for (int j=0; j < _w*_h; ++j)
      {
	pcl::PointXYZ& q = _cloud->points[j];
	dx = q.x - cx;	dy = q.y - cy;	dz = q.z - cz;
   
	if (isInlier(dx, dy, dz))
	  _flt_mask[j] = true;
      }
    return best_score;
  }

private:

  inline bool isInlier(double dx, double dy, double dz)
  {
    // |(dl/R)^2 - 1| < Error 
    return fabs((dx*dx+dy*dy+dz*dz)/(_radius*_radius)-1.0) < _err;
    //    return fabs(sqrt(x*x+y*y+z*z) - _radius) < _err;
  }
  
  double _radius;
  double _err; // TODO: _err can be a function of depth
  int _w, _h;
  std::vector<bool> _flt_mask; // filter mask
  // std::vector<bool> _pre_mask;
  double _center[3];
  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
};
