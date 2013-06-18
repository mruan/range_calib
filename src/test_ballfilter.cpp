/*
This one works better with background subtracted data
*/

#include <iostream>

#include "ball_filter.hpp"
#include "fit_sphere.hpp"
#include "pcl_utils.hpp"
#include "glog/logging.h"

#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

double RayCostFunction::R = 0.1275;

/******************************************************************
                           Main Function
 *****************************************************************/
int main(int argc, char** argv)
{
  if (argc <2)
    {
      PCL_ERROR("Usage: %s <input.pcd> \n", argv[0]);
      return -1;
    }

  google::InitGoogleLogging(argv[0]);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud)==-1)
    {
      PCL_ERROR("Couldn't read file %s\n", argv[1]);
      return -1;
    }
  
  BallFilter bf(0.128, 0.03);
  bf.SetInputPCL(cloud);
  if (!bf.ApplyFilter())
    {
      PCL_ERROR("Couldn't find a ball center\n");
      return -1;
    }

  // Colorize points
  const std::vector<bool>& mask = bf.GetMask();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ccl = ColorizeCloud(cloud, mask);
  
  //  std::cout<< "Num of inliers " << inlier << std::endl;
  
  // Try to fit a sphere:
  double x, y, z;
  bf.GetCenter(x, y, z); // get initial guess
 
  SphereFitter sf(x, y, z);
  sf.SetInputCloud(cloud, mask);
  double cx, cy, cz;
  if (!sf.FitSphere(cx, cy, cz))
    {
      PCL_ERROR("failed to solve the problem\n");
      return -1;
    }
  printf("Initial guess is [%8.5lf, %8.5lf, %8.5lf]\n",x, y, z);
  printf("Final answer is  [%8.5lf, %8.5lf, %8.5lf]\n",cx, cy, cz);
  
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  // black background
  viewer.setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(ccl);
  viewer.addPointCloud<pcl::PointXYZRGB> (ccl, rgb, "sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer.addCoordinateSystem (0.2);
  viewer.initCameraParameters ();
  viewer.addSphere(pcl::PointXYZ(cx, cy, cz), 0.1275, 0.0, 1.0, 0.0);
  //  viewer.setCameraPosition(0.0, 0.0, 0.0, 0.0, -1.0, 0.0);
  
  while(!viewer.wasStopped())
    {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  return 0;
}
