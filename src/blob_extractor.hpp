/*
  Class and Helper functions to extract blob -> sphere from point cloud
  It normally does the following 3 steps:
  1. Seperate the point cloud into blobs
  2. Select which blob contains the ball and find a rough estimate
  3. Do a nonlinear least square fit to find the center of the ball
 */

#ifndef BLOB_EXTRACTION_HPP
#define BLOB_EXTRACTION_HPP

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include "ball_selector.hpp"
#include "sphere_fitter.hpp"

namespace BlobExtraction{
  
  typedef pcl::PointXYZ PointT;
  typedef pcl::PointCloud<PointT> CloudT;

  float ball_score_fn(CloudT::Ptr cloud, Point3d& center)
  {
    // Err = err% error (relative to Radius)
    BallSelector bs(0.2);
    bs.SetInputCloud(cloud);
    float score = bs.ComputeScore();// the higher the more likely to be a ball;
    center = bs.GetCenter();
    return score;
  }

  class KDTreeBE
  {
  public:
    KDTreeBE():_kdt(new pcl::search::KdTree<PointT>)
    {
      _ec.setClusterTolerance (0.05); // in meters
      _ec.setMinClusterSize (200);
      _ec.setMaxClusterSize (25000);
      _ec.setSearchMethod (_kdt);      
    }

    void SetInputCloud(CloudT::Ptr in_cloud)
    {
      _kdt->setInputCloud(in_cloud);
      _ec.setInputCloud(in_cloud);
      raw_cloud = in_cloud;
    }

    bool ExtractBallCenter(Point3d& center, std::vector<int>& inlierIdx)
    {
      
      std::vector<pcl::PointIndices> cluster_idx;
      _ec.extract(cluster_idx);
      
      std::cout<< "Num of clusters found: "<< cluster_idx.size() << std::endl;

      std::vector<pcl::PointIndices>::iterator it = cluster_idx.begin();
      std::vector<pcl::PointIndices>::iterator best_it = cluster_idx.end();
      double this_score = 0.0, best_score = 0.0;
      double x, y, z;
      double cx, cy, cz;
      int j=0;
      while(it != cluster_idx.end())
	{
	  // TODO: this is probably an unnecessary copy
	  CloudT::Ptr cloud_cluster (new CloudT);
	  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	    cloud_cluster->points.push_back (raw_cloud->points[*pit]); //*

	  cloud_cluster->width = cloud_cluster->points.size ();
	  cloud_cluster->height = 1;
	  cloud_cluster->is_dense = true;

	  // How likely does this blob contain the target ball
	  this_score = ball_score_fn(cloud_cluster, center);
	  printf("cluster %d, score= %f\n", j++, this_score);
	  if (this_score > best_score)
	    {
	      best_it = it;
	      best_score = this_score;
	      cx = x; cy =y; cz = z;
	    }
	  it++; // move on to the next one
	}
      if (best_it == cluster_idx.end())
	{
	  // failed to find a good blob
	  printf("Failed to find a good blob that meet the criteria\n");
	  return false;
	}

      printf("Best score is %f\n", best_score);

      // Refine the estimate with a nonlinear least square fit
      SphereFitter sf(center);// set initial guess
      sf.SetInputCloud(raw_cloud, best_it->indices);
      sf.FitSphere(center);

      inlierIdx = best_it->indices;

      return true;
    }

    CloudT::Ptr raw_cloud;
    pcl::search::KdTree<PointT>::Ptr _kdt;
    pcl::EuclideanClusterExtraction<PointT> _ec;
  };
};

#endif
	  /*
	  CloudT::Ptr cluster (new CloudT);
	  cluster->points.reserve(it->indices.size());
	  for(int i=0; i< it->indices.size(); i++)
	    cluster->points.push_back(raw_cloud->points[it->indices[i]]);
	  
	  cluster->width = it->indices.size();
	  cluster->height= 1;
	  cloud_cluster->is_dense = true;
	  */
