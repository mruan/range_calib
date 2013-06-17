#include <iostream>

#include "ball_filter.hpp"

#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
  if (argc <2)
    {
      PCL_ERROR("Usage: %s <input.pcd> \n", argv[0]);
      return -1;
    }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud)==-1)
    {
      PCL_ERROR("Couldn't read file %s\n", argv[1]);
      return -1;
    }
  
  BallFilter bf(0.128, 0.03, cloud->width, cloud->height);
  bf.SetInputPCL(cloud);
  if (!bf.ApplyFilter())
    {
      PCL_ERROR("Couldn't find a ball center\n");
      return -1;
    }

  // Colorize points
  const std::vector<bool>& mask = bf.GetMask();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ccl(new pcl::PointCloud<pcl::PointXYZRGB>);
  ccl->points.resize(mask.size());

  // pack r/g/b into rgb
  int red = ((int)255) << 16 | ((int)0) << 8 | ((int)0);
  int blue= ((int)0)   << 16 | ((int)0) << 8 | ((int)255);

  int inlier = 0;
  for(int i=0; i< mask.size(); ++i)
    {
      if (mask[i])
	{
	  ccl->points[i].rgb = *reinterpret_cast<float*>(&red);
	  inlier++;
	}
      else
	{
	  ccl->points[i].rgb = *reinterpret_cast<float*>(&blue);
	}
      ccl->points[i].x = cloud->points[i].x;
      ccl->points[i].y = cloud->points[i].y;
      ccl->points[i].z = cloud->points[i].z;
    }
  
  //  std::cout<< "Num of inliers " << inlier << std::endl;

  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  // black background
  viewer.setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(ccl);
  viewer.addPointCloud<pcl::PointXYZRGB> (ccl, rgb, "sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer.addCoordinateSystem (0.2);
  viewer.initCameraParameters ();
  //  viewer.setCameraPosition(0.0, 0.0, 0.0, 0.0, -1.0, 0.0);
  
  while(!viewer.wasStopped())
    {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}
