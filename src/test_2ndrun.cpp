/*
  Test script for the second run of the optimization,
  where the ball positions in 3D space had been found in the previous step.

  Once we know where the balls are, we can remask out point clouds and do a better fitting
 */

#include <pcl/io/pcd_io.h>

#include <iostream>
#include <fstream>

#include "bg_subtractor.hpp"
#include "blob_extractor.hpp"
#include "resource_manager.hpp"

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> CloudType;

void help(char* info)
{
  printf("Usage: %s <.ba file> <.config file> <out .ba file>\n", info);
  exit(1);
}

// Global variable used throughout all files:
const double g_Radius = 0.1275; // radius of the target ball

int main(int argc, char** argv)
{
  if (argc < 4)
    help(argv[0]);

  ifstream is(argv[1], std::ifstream::in);
  ResourceManager rm;
  if(!rm.ReadFromStream(is))
    {
      PCL_ERROR("Error loading resource manager with file %s\n", argv[1]);
      return 1;
    }
  is.close();

  // Then open up the config file
  is.open(argv[2], std::ifstream::in);
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

      // Skip all the background files
      for(int j=0; j<num_bg_pcds; ++j)
	is >> filename;

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

	  Point3d center;
	  BallSelector bs(0.1);// allow 10% error
	  double f = 525.0;    // emperical focal length for xtion
	  bs.SetPreMask(rm.GetSensorObservation(i,k), f);
	  bs.ComputeScore();
	  center = bs.GetCenter();

	  SphereFitter sf(center);
	  sf.SetInputCloud(raw, bs.GetMask());
	  sf.FitSphere(center);

	  rm.UpdateSensorObservation(i, k, center);

	  if (i==0)
	    rm.UpdateLandmark(i, center);
	}
    }
    is.close();

    // Save the file for bundle adjustment
  ofstream os(argv[3], std::ofstream::out);
  rm.WriteToStream(os);
  os.close();
  
  std::cout<< "Wrote BA problem to file: "<< std::string(argv[2])<<std::endl;
  return 0;
}
