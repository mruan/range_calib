#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <stdio.h>
#include <vector>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;

void save_error(CloudT::Ptr cloud, pcl::ModelCoefficients::Ptr coeff, std::string filename="error.txt");
void view_plane(CloudT::Ptr cloud, pcl::ModelCoefficients::Ptr coeff, pcl::PointIndices::Ptr inliers);

int main (int argc, char** argv)
{
  if (argc <2)
    {
      PCL_ERROR("Usage: %s <1..m>.pcd [Options]\n", argv[0]);
      PCL_ERROR("Options: -t [threshold]: Set RANSAC threshold, defualt 0.05\n");
      PCL_ERROR("         -v:             View fitted plane\n");
      PCL_ERROR("         -s:             Save errors to file\n");
      exit(1);
    }

  std::vector<int> pcds = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
  bool display_flag = pcl::console::find_switch(argc, argv, "-v");
  bool save_flag    = pcl::console::find_switch(argc, argv, "-s");
  float threshold = 0.05f;
  pcl::console::parse_argument (argc, argv, "-t", threshold);
  PCL_WARN("RANSAC threshold = %f\n", threshold);

  for (int i=0 ; i < pcds.size(); i++)
    {
      CloudT::Ptr cloud(new CloudT);
      
      std::string pcd_name(argv[pcds[i]]);
      if (pcl::io::loadPCDFile<PointT> (pcd_name, *cloud) == -1)
	{
	  PCL_ERROR("Failed to load file %s\n", pcd_name.c_str());
	  exit(1);
	}

      pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<PointT> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (threshold);
      
      seg.setInputCloud (cloud);
      seg.segment (*inliers, *coeff);

      if (inliers->indices.size () == 0)
	{
	  PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	  return (-1);
	}

      std::cerr << "Model coefficients: " << coeff->values[0] << " " 
		<< coeff->values[1] << " "
		<< coeff->values[2] << " " 
		<< coeff->values[3] << std::endl;


      if(save_flag)
	{
	  unsigned found = pcd_name.find_last_of(".");
	  std::string filename("e"+pcd_name.substr(0, found)+".txt");
	  save_error(cloud, coeff, filename);
	}
      // Display fitted plane
      if (display_flag) 
	view_plane(cloud, coeff, inliers);
    }
  return (0);
}

void save_error(CloudT::Ptr cloud, pcl::ModelCoefficients::Ptr coeff, std::string filename)
{
  float a = coeff->values[0];
  float b = coeff->values[1];
  float c = coeff->values[2];
  float d = coeff->values[3];
  float n = 1/std::sqrt(a*a+b*b+c*c);

  float error;
  int count_nan = 0;
  std::ofstream os(filename.c_str());
  for (int row = 0; row < cloud->height; ++row)
    {
      for(int col=0; col < cloud->width; ++col)
	{
	  int linear_idx = row*cloud->width + col;
	  PointT& p = cloud->points[linear_idx];
	  if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
	    {
	      count_nan++;
	    }
	    error = (p.x*a + p.y*b + p.z*c + d)*n;
	  os << error << " ";
	}
      os << std::endl;
    }
  os.close();
  PCL_WARN("Save error data to file %s (%d out of %d are nan)\n", filename.c_str(), count_nan, cloud->size());
}


void view_plane(CloudT::Ptr cloud, pcl::ModelCoefficients::Ptr coeff, pcl::PointIndices::Ptr inliers)
{
  static pcl::visualization::PCLVisualizer viewer("3D Viewer");
  static bool first_time = true;
  CloudT::Ptr inlier_cloud(new CloudT), outlier_cloud(new CloudT);
  
  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;
      
  // Extract the inliers
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*inlier_cloud);
      
  // Create the filtering object
  extract.setNegative (true);
  extract.filter (*outlier_cloud);
  
  // Set up a custom color handler
  pcl::visualization::PointCloudColorHandlerCustom<PointT> outlier_handler(outlier_cloud,   0, 255, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT>  inlier_handler( inlier_cloud, 255,   0, 0);
  
  if(first_time)
    {
#define POINT_SIZE pcl::visualization::PCL_VISUALIZER_POINT_SIZE       
      viewer.addPointCloud<PointT> (inlier_cloud, inlier_handler, "inlier_cloud");
      viewer.setPointCloudRenderingProperties (POINT_SIZE , 1, "inlier_cloud");
      viewer.addPointCloud<PointT> (outlier_cloud, outlier_handler, "outlier_cloud");
      viewer.setPointCloudRenderingProperties (POINT_SIZE , 1, "outlier_cloud");
      viewer.addPlane(*coeff, "plane");
      
      viewer.setBackgroundColor (0, 0, 0);
      
      viewer.addCoordinateSystem (0.05);
      viewer.initCameraParameters ();
      viewer.setCameraPose(0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
      first_time = false;
    }
  else
    {
      viewer.updatePointCloud<PointT>(inlier_cloud, inlier_handler, "inlier_cloud");
      viewer.updatePointCloud<PointT>(outlier_cloud, outlier_handler, "outlier_cloud");
      viewer.removeShape("plane");
      viewer.addPlane(*coeff, "plane");
    }

  while(!viewer.wasStopped())
    {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  viewer.resetStoppedFlag();
}
