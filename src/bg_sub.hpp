
/*
  Does background subtraction
 */
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
//#include <pcl/filters/extract_indices.h>

class BackgroundSubtractor
{
  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> CloudType;
public:
  BackgroundSubtractor(float res = 0.01f)
    :resolution(res), octree (resolution)
  {}

  void SetBackgroundCloud(CloudType::Ptr bg)
  {
    octree.setInputCloud (bg);
    octree.addPointsFromInputCloud();
    // Switch octree buffers: This resets octree 
    // but keeps previous tree structure in memory.
    octree.switchBuffers ();
  }

  CloudType::Ptr GetForeGroundCloud(CloudType::Ptr diff)
  {
    // Add points from cloudB to octree
    octree.setInputCloud (diff);
    octree.addPointsFromInputCloud();

    // Get vector of point indices from octree voxels 
    // which did not exist in previous buffer
    fgIdx.clear();
    octree.getPointIndicesFromNewVoxels(fgIdx);

    // Allocate space for the new point cloud
    CloudType::Ptr fg(new CloudType);
    fg->width = fgIdx.size();
    fg->height = 1;
    fg->points.resize(fgIdx.size());
    
    // Fill the pointcloud
    for(int i=0; i<fgIdx.size(); ++i)
      {
	fg->points[i] = diff->points[fgIdx[i]];
      }
    return fg;
  }

private:
  float resolution;
  // Instantiate octree-based point cloud change detection class
  pcl::octree::OctreePointCloudChangeDetector<PointType> octree;
  std::vector<int> fgIdx;
};
