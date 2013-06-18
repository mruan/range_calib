#include "ball_filter.hpp"

BallFilter::BallFilter(double r, double err, int width, int height)
  :_radius(r), _err(err), _w(width), _h(height), _flt_mask(_w*_h, 0)
{
}

void BallFilter::SetInputPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  _cloud = cloud;
  //  printf("W = %d, H= %d\n", cloud->width, cloud->height);
}

const std::vector<bool>& BallFilter::GetMask()
{
  return _flt_mask;
}

inline double getDiff(double x, double y, double z, double r)
{
  return sqrt(dx*dx+dy*dy+dz*dz) - _radius;
  //  return dx*dx + dy* dy + dz* dz - _radius * _radius;
}

bool BallFilter::ApplyFilter()
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
      cz = p.z + _radius;
      cx = p.x;
      cy = p.y;
      
      // reset the count;
      int this_count = 0;

      // count how many points potentially belong to
      // such a sphere
      // TODO: this is stupid, only need to search the neighborhood,
      for (int j=0; j < _w*_h; ++j)
	{
	  pcl::PointXYZ& q = _cloud->points[j];
	  dx = q.x - cx;
	  dy = q.y - cy;
	  dz = q.z - cz;

	  double diff = getDiff(dx, dy, dz, _radius);
	  if (fabs(diff) < _err)
	    this_count++;
	}

      this_score = this_count * cz * cz;

      if (this_score > best_score)
	{
	  best_score = this_score;
	  best_index = i;
	}

      printf("%lf ", this_score);
    }

  if (best_index < 0)
    return false;

  //  std::cout << "Found " << best_count << " inliers" << std::endl;

  // Build the mask:
  pcl::PointXYZ& p = _cloud->points[best_index];
  
  // Assume the center is at this pixel
  // but pushed back by a radius r;
  cz = p.z + _radius;
  cx = p.x;
  cy = p.y;  

  // TODO: Again this is stupid, only search the neighborhood
  for (int j=0; j < _w*_h; ++j)
    {
      pcl::PointXYZ& q = _cloud->points[j];
      dx = q.x - cx;
      dy = q.y - cy;
      dz = q.z - cz;
      
      double diff = getDiff(dx, dy, dz, _radius);
	//sqrt(dx*dx+dy*dy+dz*dz) - _radius;
      if (fabs(diff) < _err)
	_flt_mask[j] = true;
    }
  return true;
}
