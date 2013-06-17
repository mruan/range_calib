#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char** argv)
{
  if (argc < 3)
    {
      std::cout<<"Usage: downsample [input file] [output file]"<<std::endl;
      return -1;
    }

 sensor_msgs::PointCloud2::Ptr blob(new sensor_msgs::PointCloud2()), filtered_blob(new sensor_msgs::PointCloud2());
  
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read (argv[1], *blob); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << blob->width * blob->height 
	    << " data points (" << pcl::getFieldsList (*blob) << ").\n";
  // Downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud(blob);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*filtered_blob);

  std::cerr << "PointCloud after filtering: " << filtered_blob->width * filtered_blob->height << " data points (" << pcl::getFieldsList (*filtered_blob) << ").\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>), cutoff_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to the templated PointCloud
  pcl::fromROSMsg(*filtered_blob, *pcl_cloud);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (pcl_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cutoff_cloud);

  std::cerr << "PointCloud after cut-off: " << cutoff_cloud->points.size() << " data points (" << pcl::getFieldsList (*cutoff_cloud) << ").\n";

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (argv[2], *cutoff_cloud, false);
  
  return 0;
}
