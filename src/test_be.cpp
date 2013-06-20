
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include "blob_extractor.hpp"
#include "pcl_utils.hpp"

/*******************************************************
                 Main
********************************************************/
int main(int argc, char** argv)
{
  if (argc < 2)
    {
      printf("Usage: %s [input cloud] [threshold]\n", argv[0]);
      return -1;
    }

  // Load PCD file:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1)
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }
 
  BlobExtraction::KDTreeBE be;
  be.SetInputCloud(cloud);

  double cx, cy, cz;
  std::vector<int> inlierIdx;
  if( !be.ExtractBallCenter(cx, cy, cz, inlierIdx))
    return -1;
 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ccl = ColorizeCloud(cloud, inlierIdx);

  // Visualization
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  viewer.setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(ccl);
  viewer.addPointCloud<pcl::PointXYZRGB> (ccl, rgb, "sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer.addCoordinateSystem (0.1);
  viewer.initCameraParameters ();
  viewer.addSphere(pcl::PointXYZ(cx, cy, cz), 0.1275, 1.0, 1.0, 0.0);

  while(!viewer.wasStopped())
    {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

  return 0;
}
