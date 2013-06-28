/*
  Test the first half of the pipeline up to
  fitting a sphere to a blob and write to a file via resource manager
 */

#include <pcl/io/pcd_io.h>
// For visualization code
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <iostream>
#include <fstream>

#include "bg_subtractor.hpp"
#include "blob_extractor.hpp"
#include "resource_manager.hpp"

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

void showfgCloud(CloudType::Ptr fg);
void showfittedSphere(CloudType::Ptr fg, double x, double y, double z);
void help(char* info)
{
  printf("Usage: %s <config file> <out config file>\n", info);
  exit(1);
}

// Global variable used throughout all files:
const double g_Radius = 0.1275; // radius of the target ball

int main(int argc, char** argv)
{
  if (argc < 3)
    help(argv[0]);

  ifstream is(argv[1], std::ifstream::in);
  if(!is.is_open())
    {
      std::cerr<< "Cannot open file: "<< std::string(argv[1])<<std::endl;
      return 1;
    }

  std::string token;
  is >> std::skipws >> token;
  // Skip comments
  while(token=="#") 
    {
      std::getline(is, token);
      is>> token;
    }

  int num_sensors, num_bg_pcds, num_fg_pcds;
  std::string glob_prefix;

  // Read in header information
  assert(token == "NumSensors:");
  is >> num_sensors >> token;
  assert(token == "NumBackgroundPCDs:");
  is >> num_bg_pcds >> token;
  assert(token == "NumForegroundPCDs:");
  is >> num_fg_pcds >> token;
  assert(token == "GlobalPathPrefix:");
  is >> glob_prefix;

  std::cout<<"Summary: "<< std::endl;
  std::cout<<"NumSensors: "<< num_sensors<<std::endl;
  std::cout<<"NumBackgroundPCDs: "<< num_bg_pcds<<std::endl;
  std::cout<<"NumForegroundPCDs: "<< num_fg_pcds<<std::endl;

  BGS::ImageFilter bgs;
  BlobExtraction::KDTreeBE be;
  ResourceManager rm(num_sensors, num_fg_pcds);

  for(int i=0; i<num_sensors; ++i)
    {
      std::string name, path_prefix, type, filename;
      is >> token;
      assert(token == "Name:");
      is >> name >> token;
      assert(token == "Type:");
      is >> type >> token;
      assert(token == "PathPrefix:");
      is >> path_prefix;

      std::shared_ptr<Sensor> sensor(new RangeSensor(name, num_fg_pcds));

      // TODO: Reset background: bgs.ResetBackground;
      for(int j=0; j<num_bg_pcds; ++j)
	{
	  // Load pcds and build background model
	  CloudType::Ptr bg (new CloudType);

	  // Read in the file name:
	  is >> filename;
	  if(pcl::io::loadPCDFile<PointType>(glob_prefix+path_prefix+filename, *bg) == -1)
	    {
	      PCL_ERROR("Failed to load background file\n");
	      std::cerr<< filename << std::endl;
	      exit(1);
	    }

	  // TODO: API changes to AddBackground;
	  bgs.SetBackgroundCloud(bg);
	}

      // Find calibration target in the (foreground) pointcloud
      for(int k=0; k<num_fg_pcds; ++k)
	{
	  CloudType::Ptr raw(new CloudType);

	  // Read in the file name
	  is >> filename;
	  if(pcl::io::loadPCDFile<PointType>(glob_prefix+path_prefix+filename, *raw)==-1)
	    {
	      PCL_ERROR("Failed to load background file:");
	      std::cerr<< filename << std::endl;
	      exit(1);
	    } 
	  std::cout << "Process: "<< filename<<". "<<std::endl;

	  // Filter out background
	  CloudType::Ptr fg = bgs.GetForegroundCloud(raw);

	  //showfgCloud(fg);
	  
	  // Extract ball center
	  be.SetInputCloud(fg);

	  Point3d center;
	  std::vector<int> inlierIdx;
	  if(!be.ExtractBallCenter(center, inlierIdx))
	    exit(1);

	  //	  showfittedSphere(fg, center[0], center[1], center[2]);

	  // Add it to the resource manager
	  sensor->AddObservation(k, center);

	  // First sensor is used as reference;
	  // TODO: currently it must be a range sensor(x,y,z must be available)
	  if (i==0)
	    rm.AddLandmark(center);

	}
      // Add the entire sensor to the manager
      rm.AddSensor(sensor); 
    }
  // Close the config file
  is.close();

  // First solve a linear problem and use that for initial guess
  //  rm.SolveLinearRigidTf();
  
  // Save the file for bundle adjustment
  ofstream os(argv[2], std::ofstream::out);
  rm.WriteToStream(os);
  os.close();
  
  std::cout<< "Wrote BA problem to file: "<< std::string(argv[2])<<std::endl;
  return 0;
}

void showfgCloud(CloudType::Ptr cloud)
{
  pcl::visualization::PCLVisualizer viewer("Foreground Cloud Viewer");
  viewer.setBackgroundColor (0, 0, 0);

  viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer.addCoordinateSystem (0.05);
  viewer.initCameraParameters ();

  while(!viewer.wasStopped())
    {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

void showfittedSphere(CloudType::Ptr cloud, double x, double y, double z)
{
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  viewer.setBackgroundColor (0, 0, 0);

  viewer.addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer.addCoordinateSystem (0.05);
  viewer.initCameraParameters ();

  viewer.addSphere(pcl::PointXYZ(x, y, z), 0.1275, 1.0, 1.0, 0.0);

  while(!viewer.wasStopped())
    {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }  
}
