/*
  How is the nlsqr better than linear solution?
 */

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include "pcl_utils.hpp"
#include "resource_manager.hpp"

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

int main(int argc, char** argv)
{
  if (argc<4)
    {
      printf("Usage: %s <config file> <r1.pcd> <r2.pcd>\n", argv[0]);
      exit(1);
    }

  CloudType::Ptr c0(new CloudType), c1(new CloudType);
  CloudType::Ptr c0t(new CloudType), c1t(new CloudType);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr c0c, c1c;

  if(pcl::io::loadPCDFile<PointType>(argv[2], *c0)==-1)
    {
      PCL_ERROR("Fail to load cloud: %s", argv[1]);
      exit(1);
    }
  if(pcl::io::loadPCDFile<PointType>(argv[3], *c1)==-1)
    {
      PCL_ERROR("Fail to load cloud: %s", argv[2]);
      exit(1);
    }

  ResourceManager rm;
  ifstream ifs(argv[1], std::ifstream::in);
  if(!rm.ReadFromStream(ifs))
    {
      PCL_ERROR("Error loading resource manager from stream\n");
      exit(1);
    }

  Eigen::Vector3f t0;
  Eigen::Quaternionf q0;
  rm.GetSensorTransform(0, t0, q0);
  pcl::transformPointCloud(*c0, *c0t, t0, q0);

  Eigen::Vector3f t1;
  Eigen::Quaternionf q1;
  rm.GetSensorTransform(1, t1, q1);
  pcl::transformPointCloud(*c1, *c1t, t1, q1);

  c0c = ColorizeCloud(c0t, "red");
  c1c = ColorizeCloud(c1t, "green");

  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  viewer.setBackgroundColor (0, 0, 0);

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> h0(c0c);
  viewer.addPointCloud<pcl::PointXYZRGB> (c0c, h0, "cloud0");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud0");

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> h1(c1c);
  viewer.addPointCloud<pcl::PointXYZRGB> (c1c, h1, "cloud1");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");

  viewer.addCoordinateSystem (0.1);
  viewer.initCameraParameters ();

  while(!viewer.wasStopped())
    {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

  return 0;
}
/*
// Visualization
  
 */
