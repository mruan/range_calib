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

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

double RayCostFunction::R = 0.1275;

bool SegSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
	       double threshold,
	       pcl::PointIndices::Ptr inlier,
	       pcl::ModelCoefficients::Ptr coeff)
{
  //  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
  //  pcl::PointIndices::Ptr inlier(new pcl::PointIndices());

  //Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_SPHERE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(threshold);
  // Diameter from 25.3cm to 25.5 cm
  seg.setRadiusLimits(0.12, 0.13);

  // Segment the largest spherical component from the remaining cloud
  seg.setInputCloud(cloud);
  seg.segment(*inlier, *coeff);

  bool flag = true;
  /*
  if(inlier->indices.size() ==0)
    {
      std::cerr<<"Could not estimate a spherical model at threshold "<< threshold<< " for the given dataset."<<std::endl;
      flag = false;
    }
  */
  std::cout <<"Threshold at " << threshold << " Coeffs: " << coeff->values[0] << " " << coeff->values[1] << " "<< coeff->values[2] << " "<< coeff->values[3] << std::endl;
  return flag;
}

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

/*

  pcl::PointIndices::Ptr inlierIdx (new pcl::PointIndices ());
  pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients ());
  double threshold = 0.005;
  if (!SegSphere(cloud, threshold, inlierIdx, coeff))
    return -1;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ccl = ColorizeCloud(cloud, inlierIdx);

  double x = coeff->values[0], y = coeff->values[1], z = coeff->values[2];
  SphereFitter sf(x, y, z);
  sf.SetInputCloud(cloud, inlierIdx->indices);
*/
