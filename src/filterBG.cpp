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

// PCL stuff
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// For background subtraction
#include "bg_sub.hpp"

using namespace std;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

int main(int argc, char** argv)
{
  if (argc< 3)
    {
      printf("Usage: %s [bg pcd] [fg pcd]\n", argv[0]);
      return -1;
    }

  // Load PCD file:
  CloudType::Ptr bg (new CloudType), raw(new CloudType), fg;

  if (pcl::io::loadPCDFile<PointType> (argv[1], *bg) == -1 ||
      pcl::io::loadPCDFile<PointType> (argv[2], *raw) == -1)
    {
      PCL_ERROR ("Couldn't read file\n");
      return (-1);
    }

  // Build octree with 1cm resolution
  BackgroundSubtractor bgs(0.5f);
  bgs.SetBackgroundCloud(bg);

  fg = bgs.GetForeGroundCloud(raw);
  cout<< fg->points.size() <<" out of " << raw->points.size() <<" remains.\n";

  char outfile[64];
  argv[2][strlen(argv[2])-4] = '\0';
  sprintf(outfile, "fg_%s.pcd", argv[2]);
  pcl::io::savePCDFileASCII(outfile, *fg);
}
