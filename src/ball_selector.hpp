/*
  Author: Ming Ruan
  Date: June 17

  Filter out the sphere with a given radius in the point cloud
  It works much better (and faster) with background-subtracted data

 */

#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "types.hpp"// custom-defined points

// This global variable must be declared once somewhere in main() function!
extern const double g_Radius; // global target ball's radius

class BallSelector
{
public:
  BallSelector(double err)
    :_err(err){}

  void SetInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int mask_v=1)
  { 
    _cloud = cloud; _w = cloud->width; _h = cloud->height; 
    _mask.resize(_w*_h);

    // 1. Set mask to all true if you don't know any prior
    // 2. Otherwise set it all false and set individual pixel later
    std::fill(_mask.begin(), _mask.end(), mask_v);//TODO: inefficient
  }

  Point3d GetCenter()
  { return _center; }

  // This function should be called after the input cloud has been set
  void SetPreMask(const Point3d& p, double f, int cx=-1, int cy=-1)
  {
    assert(_mask.size()>0);
    if (cx==-1 && cy==-1)
      {
	cx = _cloud->width/2;
	cy = _cloud->height/2;
      }

    int u = p.x/p.z*f + cx;
    int v = p.y/p.z*f + cy;
    int r = g_Radius/p.z*f; // TODO: use std::ceil?

    for(int du=0; du <= r; ++du)
      {
	for(int dv = 0; dv<=r- du; ++dv)
	  {//TODO: exception will be thrown here if it goes out of bound
	    _mask.at((u-du) * _cloud->width + (v-dv)) = 1;
	    _mask.at((u+du) * _cloud->width + (v-dv)) = 1;
	    _mask.at((u-du) * _cloud->width + (v+dv)) = 1;
	    _mask.at((u+du) * _cloud->width + (v+dv)) = 1;
	  }
      }
  }

  const std::vector<bool>& GetMask()
  { return _mask; }

  // This is still a rather stupid implementation of the idea
  // which should have exploited neighbors in image space
  // TODO: it might be sufficient to test only at the geometric center
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
	if (!_mask[i])
	  continue;

	pcl::PointXYZ& p = _cloud->points[i];

	// Assume the center is at this pixel
	// but pushed back by a radius r;
	cz = p.z + g_Radius;  cx = p.x;	cy = p.y;
      
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
    _center.z = cz = p.z + g_Radius;
    _center.x = cx = p.x;
    _center.y = cy = p.y;  

    // TODO: Again this is stupid, only search the neighborhood
    for (int j=0; j < _w*_h; ++j)
      {
	pcl::PointXYZ& q = _cloud->points[j];
	dx = q.x - cx;	dy = q.y - cy;	dz = q.z - cz;
   
	if (isInlier(dx, dy, dz))
	  _mask[j] = true;
      }
    return best_score;
  }

private:

  inline bool isInlier(double dx, double dy, double dz)
  {
    // |(dl/R)^2 - 1| < Error 
    return fabs((dx*dx+dy*dy+dz*dz)/(g_Radius*g_Radius)-1.0) < _err;
    //    return fabs(sqrt(x*x+y*y+z*z) - g_Radius) < _err;
  }
  
  double _err; // TODO: _err can be a function of depth
  int _w, _h;
  std::vector<bool> _mask; // filter mask
  // std::vector<bool> _pre_mask;
  Point3d _center;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
};
