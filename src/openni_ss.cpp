/*
  Author: Ming Ruan
  Date: June 11, 2013

  This program opens up a OpenNI device (Xtion or Kinect) and displays
  the point cloud in the PCLVisualizer window.

  Press <S> to save in PCD format. Copy the saved PCDs to a another location 
  otherwise next time you run the program they will be overwritten by new ones

  Before starting the program, you may want to "killall Xn[tab]" to shut off 
  any dangling services
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#define DEFAULT  0
#define SAVE_PCD 1

class OpenNIViewer
{
  typedef pcl::visualization::PCLVisualizer Visualizer;
  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> CloudType;
private:
  
  boost::shared_ptr<Visualizer> viewer;
  CloudType::ConstPtr _cloud;
  int counter;
  int key_status;
  bool first_time;
  bool updateflag;
  boost::mutex updateMutex;

  void _cloud_cb (const CloudType::ConstPtr &cloud)
  {
    if (viewer->wasStopped())
      return;

    boost::mutex::scoped_lock updateLock(updateMutex);
    _cloud = cloud;
    updateflag = true;
    updateLock.unlock();
  }

  void _key_cb(const pcl::visualization::KeyboardEvent &event)
  {
    //    Visualizer* pViewer = static_cast<Visualizer *> (view_void);
    if (event.getKeySym () == "s" && event.keyDown ())
      {
	key_status = SAVE_PCD;
      }
  }

public:
  OpenNIViewer()
 :counter(0), key_status(0),
  _cloud(new CloudType),
  viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"))
  {
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (0.5);
    viewer->addPointCloud<PointType>(_cloud, "cloud");

    boost::function<void (const pcl::visualization::KeyboardEvent&)> key_cb = boost::bind(&OpenNIViewer::_key_cb, this, _1);
    
    viewer->registerKeyboardCallback(key_cb);

    std::cout<< "viewer setup" << std::endl;
  }
   
  void run (std::string name)
  {
    
    pcl::Grabber* interface = new pcl::OpenNIGrabber();
    
    boost::function<void (const CloudType::ConstPtr&)> cloud_cb =
      boost::bind (&OpenNIViewer::_cloud_cb, this, _1);
    
    interface->registerCallback (cloud_cb);
    
    interface->start ();

    std::cout<< "Start frame grabber"<< std::endl;
    while (!viewer->wasStopped())
      {
	viewer->spinOnce (100);
	// Main viewer loop
	boost::mutex::scoped_lock updateLock(updateMutex);
	if(updateflag)
	  {
	    updateflag = false;

	    viewer->updatePointCloud(_cloud, "cloud");
	    
	    if (key_status == SAVE_PCD)
	      {
		std::stringstream ss;
		ss << name << ".pcd";
		pcl::io::savePCDFileASCII(ss.str(), *_cloud);
		std::cerr<<"Saved "<< _cloud->points.size() << " data points to: "<< ss.str() << std::endl;
		
		++counter;
		// Reset key_status to default
		key_status = DEFAULT;
	      }
	  }
	updateLock.unlock();

	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }

    interface->stop ();
  }
  
 };

int main (int argc, char** argv)
{
  //  SimpleOpenNIViewer v;
  OpenNIViewer v;
  v.run (argv[1]);
  return 0;
}
