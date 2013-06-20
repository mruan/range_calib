
/*
  Does background subtraction
 */
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
//#include <pcl/filters/extract_indices.h>

namespace BGS // background subtraction
{
  typedef pcl::PointXYZ PointType;
  typedef pcl::PointCloud<PointType> CloudType;
  
  
  /**************************************************************/
  /**************************************************************/
  //                    Octree based filter
  /**************************************************************/
  /**************************************************************/
  class OctreeFilter
  {
  public:
    OctreeFilter(float res = 0.01f)
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

    CloudType::Ptr GetForeGroundCloud(CloudType::Ptr raw)
    {
      // Add points from cloudB to octree
      octree.setInputCloud (raw);
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
	  fg->points[i] = raw->points[fgIdx[i]];
	}
      return fg;
    }

  private:
    float resolution;
    // Instantiate octree-based point cloud change detection class
    pcl::octree::OctreePointCloudChangeDetector<PointType> octree;
    std::vector<int> fgIdx;
  };


  /**************************************************************/
  /**************************************************************/
  //                     Image based filter
  /**************************************************************/
  /**************************************************************/
  class ImageFilter
  {
  public:
    ImageFilter(double epsilon = 1e-2):_eps(epsilon){}

    void SetBackgroundCloud(CloudType::Ptr bg) { _bg = bg;}

    CloudType::Ptr GetForegroundCloud(CloudType::Ptr raw)
    {
      return DoFilter(raw, true);
    }

    const std::vector<bool> GetForegroundMask(CloudType::Ptr raw)
    {
      DoFilter(raw, false);
      return _mask;
    }

  private:
    double _eps;
    CloudType::Ptr _bg;
    std::vector<bool> _mask;

    CloudType::Ptr DoFilter(CloudType::Ptr raw, bool returnCloud)
    {
      _mask.resize(_bg->points.size());
      int count = 0;
      for(int i=0; i< _mask.size(); ++i)
	{
	  PointType pb = _bg->points[i];
	  PointType pf = raw->points[i];
	  
	  if (pf.z < pb.z *(1- pb.z * _eps))
	    {
	      _mask[i] = 1;
	      count++;
	    }
	  else
	    _mask[i] = 0;
	}

      DoMorph();

      if (returnCloud)
	{
	  CloudType::Ptr fg (new CloudType);

	  fg->width = count;
	  fg->height = 1;
	  fg->points.resize(count);
	  int j=0;
	  for (int i=0; i< _mask.size(); ++i)
	    {
	      if(_mask[i])
		fg->points[j++] = raw->points[i];
	    }
	  return fg;
	}
    }

    // Erosion and Dilation
    void DoMorph()
    {
      std::vector<bool> temp(_mask.size(), 1);
      int R = 2;
      // Erosion
      for(int i=0; i< _bg->height; ++i)
	{
	  for(int j=0; j< _bg->width; ++j)
	    {
	      if (_mask[i]) continue;
	       
	      for(int k= -R; k<= R; k++)
		{
		  int r = i+k;
		  int c = j+k;
		  if(r>=0 && r< _bg->height && c>=0 && c< _bg->width)
		      temp[i*_bg->width + j] = 0;
		}
	    }
	}
      for (int i=0; i< _mask.size(); ++i)
	{
	  _mask[i] = _mask[i] & temp[i];
	  temp[i] = 0; // reset to all 0 for use in dialation
	}
      
      // Dialation
      for(int i=0; i< _bg->height; ++i)
	{
	  for(int j=0; j< _bg->width; ++j)
	    {
	      if (!_mask[i]) continue;
	       
	      for(int k= -R; k<= R; k++)
		{
		  int r = i+k;
		  int c = j+k;
		  if(r>=0 && r< _bg->height && c>=0 && c< _bg->width)
		      temp[i*_bg->width + j] = 1;
		}
	    }
	}
      for (int i=0; i< _mask.size(); ++i)
	_mask[i] = _mask[i] | temp[i];
    }
  };
};
