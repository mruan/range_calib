/*
  Class and Helper functions to extract blob -> sphere from point cloud
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

  float ball_score_fn(CloudT::Ptr cloud, double&x, double&y, double&z)
  {
    // R = 0.1275, Err = err% error (relative to Radius)
    BallSelector bs(0.1275, 0.2);
    bs.SetInputCloud(cloud);
    return bs.ComputeScore();// the higher the more likely to be a ball
  }

  class KDTreeBE
  {
  public:
    KDTreeBE():_kdt(new pcl::search::KdTree<PointT>)
    {
      _ec.setClusterTolerance (0.05); // in meters
      _ec.setMinClusterSize (50);
      _ec.setMaxClusterSize (25000);
      _ec.setSearchMethod (_kdt);      
    }

    void SetInputCloud(CloudT::Ptr in_cloud)
    {
      _kdt->setInputCloud(in_cloud);
      _ec.setInputCloud(in_cloud);
      raw_cloud = in_cloud;
    }

    bool ExtractBallCenter(double &cx, double& cy, double& cz, 
			   std::vector<int>& inlierIdx)//CloudT::Ptr out_cloud)
    {
      
      std::vector<pcl::PointIndices> cluster_idx;
      _ec.extract(cluster_idx);
      
      std::cout<< "Num of clusters found: "<< cluster_idx.size() << std::endl;

      std::vector<pcl::PointIndices>::iterator it = cluster_idx.begin();
      std::vector<pcl::PointIndices>::iterator best_it = cluster_idx.end();
      double this_score = 0.0, best_score = 0.0;
      double x, y, z;
      int j=0;
      while(it != cluster_idx.end())
	{
	  // for now, save blobs to pcd for debugging
	  CloudT::Ptr cloud_cluster (new CloudT);
	  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
	    cloud_cluster->points.push_back (raw_cloud->points[*pit]); //*

	  cloud_cluster->width = cloud_cluster->points.size ();
	  cloud_cluster->height = 1;
	  cloud_cluster->is_dense = true;
	  
	  //	  char outfile[64];
	  //	  sprintf(outfile, "blob%d.pcd", j++);
	  //	  pcl::io::savePCDFileASCII(outfile, *cloud_cluster);
	  
	  this_score = ball_score_fn(cloud_cluster, x, y, z);
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

      SphereFitter sf(cx, cy, cz);// set initial guess
      sf.SetInputCloud(raw_cloud, best_it->indices);
      sf.FitSphere(cx, cy, cz);
      
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
