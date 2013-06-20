/*
  Author: Ming Ruan
  Date: June 11, 2012
  
  Version 2:
  This program tries to extract a sphere from a point cloud
  Pre-condition:
  1. The shape of the sphere is given (known radius)

  Find sphere center with increasing distance from the camera, see how
  well it works

  Procedures:
  1. Load the PCD file, which has point type: PointXYZ
  2. Extract the parametric sphere from the point cloud
     2.1
  3. Colorize points based on inlier/outlier
  4. Display the point cloud along with the parametric sphere
*/
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>

// PCL stuff
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

// For background subtraction
#include "bg_sub.hpp"

using namespace std;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;
/*
CloudType::Ptr OctreeFilter(CloudType::Ptr bg, CloudType::Ptr raw)
{
  CloudType::Ptr fg;
  // Build octree with 50cm resolution
  BGS::OctreeFilter bgs(0.2f);
  bgs.SetBackgroundCloud(bg);

  fg = bgs.GetForeGroundCloud(raw);
  cout<< fg->points.size() <<" out of " << raw->points.size() <<" remains.\n";
  return fg;
}

// Image based filter
CloudType::Ptr ImageFilter(CloudType::Ptr bg, CloudType::Ptr raw)
{
  double eps = 1e-2; // should it scale with distance?

  std::vector<bool> mask(raw->points.size(), 0);

  assert(raw->points.size() == bg->points.size());
  int count = 0;
  for(int i=0; i< mask.size(); ++i)
    {
      PointType pb = bg->points[i];
      PointType pf = raw->points[i];

      if (pf.z < pb.z *(1- pb.z * eps))
	{
	  mask[i] = 1;
	  count++;
	}
    }

  //  printf("Foregound %d out of %d\n", count, mask.size());
  // Create the foreground point cloud
  CloudType::Ptr fg (new CloudType);
  fg->width = count;
  fg->height = 1;
  fg->points.resize(count);
  int j=0;
  for (int i=0; i< mask.size(); ++i)
    {
      if(mask[i])
	fg->points[j++] = raw->points[i];
    }
  return fg;
}
*/
int main(int argc, char** argv)
{
  if (argc< 3)
    {
      printf("Usage: %s [bg pcd] [fg pcd]\n", argv[0]);
      return -1;
    }

  // Load PCD file:
  CloudType::Ptr bg (new CloudType), raw(new CloudType);

  if (pcl::io::loadPCDFile<PointType> (argv[1], *bg) == -1 ||
      pcl::io::loadPCDFile<PointType> (argv[2], *raw) == -1)
    {
      PCL_ERROR ("Couldn't read file\n");
      return (-1);
    }

  // Imagebased background subtraction
  CloudType::Ptr fg;// = ImageFilter(bg, raw);
  //  CloudType::Ptr fg = OctreeFilter(bg, raw);
  
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (fg);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  CloudType::Ptr fgfiltered(new CloudType);
  sor.filter (*fgfiltered);  
 
  //  printf("Done filtering\n");
  char outfile[64];
  argv[2][strlen(argv[2])-4] = '\0';
  sprintf(outfile, "fg_%s.pcd", argv[2]);
  pcl::io::savePCDFileASCII(outfile, *fgfiltered);
}
