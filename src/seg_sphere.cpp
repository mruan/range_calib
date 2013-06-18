/*
  Author: Ming Ruan
  Date: June 10, 2012
  
  This program tries to extract a sphere from a point cloud
  Pre-condition:
  1. The shape of the sphere is given (known radius)

  Procedures:
  1. Load the PCD file, which has point type: PointXYZ
  2. Extract the parametric sphere from the point cloud
     2.1
  3. Colorize points based on inlier/outlier
  4. Display the point cloud along with the parametric sphere
*/

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include "pcl_utils.hpp"

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
SetupViewer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
	    pcl::ModelCoefficients::Ptr coeff);
bool SegSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	       double threshold,
	       pcl::PointIndices::Ptr inlier,
	       pcl::ModelCoefficients::Ptr coeff);

/*******************************************************
                 Main
********************************************************/
int main(int argc, char** argv)
{
  if (argc < 3)
    {
      std::cout<<"Usage: v1 [input cloud] [threshold]" <<std::endl;
      return -1;
    }

  // Load PCD file:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1)
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }

  double threshold;
  sscanf(argv[2], " %lf", &threshold);
  // Segment the sphere:
  pcl::PointIndices::Ptr inlierIdx (new pcl::PointIndices ());
  pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients ());
  bool flag = SegSphere(cloud, threshold, inlierIdx, coeff);
  if (!flag)
    return -1;

  // Separate and colorize inliers and outliers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clr_cloud = ColorizeCloud(cloud, inlierIdx);

  // Display the final result
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = SetupViewer(clr_cloud, coeff);

  while( !viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
SetupViewer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
	    pcl::ModelCoefficients::Ptr coeff)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  // black background
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  //  viewer->initCameraParameters ();
  viewer->setCameraPosition(coeff->values[0], coeff->values[1], coeff->values[2],
			    0.0, -1.0, 0.0);

  //  pcl::PointXYZ p(coeff.values[0], coeff.values[1], coeff.values[2], 
  viewer->addSphere(*coeff);

  return (viewer);
}

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
