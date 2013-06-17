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
#include <algorithm> // for std::sort

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

boost::shared_ptr<pcl::visualization::PCLVisualizer> 
SetupViewer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
	    pcl::ModelCoefficients::Ptr coeff);
bool SegSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
	       double threshold,
	       pcl::PointIndices::Ptr inlier,
	       pcl::ModelCoefficients::Ptr coeff);
void ColorizePts(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
		 pcl::PointIndices::Ptr inlierIdx,
		 pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);

uint32_t red = (static_cast<uint32_t>(255) << 16 |
		static_cast<uint32_t>(0) << 8 | static_cast<uint32_t>(0));

uint32_t blue= (static_cast<uint32_t>(0) << 16 |
		static_cast<uint32_t>(0) << 8 | static_cast<uint32_t>(255));

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
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  ColorizePts(cloud, inlierIdx, clr_cloud);

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

void ColorizePts(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
		 pcl::PointIndices::Ptr inlierIdx,
		 pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
  output->points.resize(input->points.size());

  //  uint8_t r(255), g(15), b(15);
  
  // pack r/g/b into rgb
  int red = ((int)255) << 16 | ((int)0) << 8 | ((int)0);
  int blue= ((int)0)   << 16 | ((int)0) << 8 | ((int)255);

  int numInlier = 0, numOutlier = 0;
  // In case it wasn't sorted
  std::sort(inlierIdx->indices.begin(), inlierIdx->indices.end());
  size_t j=0;
  for(size_t i=0; i< input->points.size(); ++i)
    {
      if (i==inlierIdx->indices[j])
	{
	  output->points[i].rgb = *reinterpret_cast<float*>(&red);
	  ++j;
	  numInlier++;
	}
      else
	{
	  output->points[i].rgb = *reinterpret_cast<float*>(&blue);
	  numOutlier++;
	}
      
      output->points[i].x = input->points[i].x;
      output->points[i].y = input->points[i].y;
      output->points[i].z = input->points[i].z;
    }
  std::cout << "Num inliers="<< numInlier << ", Num outliers="<< numOutlier<<std::endl;
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
