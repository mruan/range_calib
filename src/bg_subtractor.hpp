
/*
  Does background subtraction
 */
#pragma once

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
      int count =0;
      _mask.resize(_bg->points.size());
      for(int i=0; i< _mask.size(); ++i)
	{
	  PointType pb = _bg->points[i];
	  PointType pf = raw->points[i];
	  
	  if (pf.z < pb.z *(1- pb.z * _eps) ||
	      !isnan(pf.z) && isnan(pb.z))
	    {
	      _mask[i] = 1;
	      count++;
	    }
	  else
	    _mask[i] = 0;
	}

      // Morphological operation: Erosion and dilation
      //      count = Dilate2_5D(50.0);
      count = Erode2_5D(100.0);
      count = Dilate2_5D(100.0);

      if (returnCloud)
	{
	  CloudType::Ptr fg (new CloudType);

	  fg->width = count;
	  fg->height = 1;
	  fg->points.resize(count);
	  int j=0;
	  for (int i=0; i< _mask.size(); ++i)
	    {
	      if(_mask[i] && !isnan(raw->points[i].z))
		fg->points[j++] = raw->points[i];
	    }
	  return fg;
	}
    }

    // Erosion and Dilation in 2.5D (where the radius scales with depth)
    int Erode2_5D(float alpha)
    {
      int count = 0;
      std::vector<bool> temp(_mask.size(), 1);// start with all ones

      for(int i=0; i<_mask.size(); i++)
	{
	  if(_mask[i])
	    count++;
	}
      //      printf("%d left\n", count);
      count = 0;

      // Erosion
      for(int i=0; i< _bg->height; ++i)
	{
	  for(int j=0; j< _bg->width; ++j)
	    {
	      int index = i*_bg->width+j;
	      if (!_mask[index]) //if it's already 0, leave it that way
		continue;
	      
	      // This pixel is 1, but if any of my neighbor is 0, I become 0.
	      int R = ceil(alpha / _bg->points[index].z);
	      bool break_flag = false;
	      for(int k= -R; k<= R; k++)
		{
		  for(int l=-R; l<=R; l++)
		    {
		      int r = i+k;
		      int c = j+l;
		      if(r>=0 && r< _bg->height && c>=0 && c< _bg->width
			 && _mask[r*_bg->width + c] ==0)
			{
			  temp[index] = 0;
			  break_flag = true;
			  break;
			}
		    }
		  if (break_flag)
		    { break_flag=false; break;}
		}
	    }
	  // next pixel
	}

      for (int i=0; i< _mask.size(); ++i)
	{
	  _mask[i] = _mask[i] & temp[i];
	  if(_mask[i])
	    count++;
	}
      //      printf("%d left\n", count);
      return count;
    }
   
    int Dilate2_5D(float alpha)
    {
      int count = 0;
      std::vector<bool> temp(_mask.size(), 0);// start with all zeros

      for(int i=0; i< _bg->height; ++i)
	{
	  for(int j=0; j< _bg->width; ++j)
	    {
	      int index = i*_bg->width+j;
	      if (!_mask[index]) // i'm 0, don't do anything
		continue;
	      
	      // I'm 1, all my neighbors will become 1
	      int R = ceil(alpha /_bg->points[index].z);
	      for(int k= -R; k<= R; k++)
		{
		  for(int l=-R; l<=R; l++)
		    {
		      int r = i+k;
		      int c = j+l;
		      if(r>=0 && r< _bg->height && c>=0 && c< _bg->width)
			temp[r*_bg->width + c] = 1;
		    }
		}
	    }
	}
      count = 0;
      for (int i=0; i< _mask.size(); ++i)
	{
	  _mask[i] = _mask[i] | temp[i];
	  if (_mask[i])
	    count++;
	}
      //      printf("%d left\n", count);
      return count;
    }
    
    // Erosion and Dilation
    // TODO: Currently deprecated
    int Erode2D(int R = 1)
    {
      int count = 0;
      std::vector<bool> temp(_mask.size(), 1);

      for(int i=0; i<_mask.size(); i++)
	{
	  if(_mask[i])
	    count++;
	}
      //      printf("%d left w: %d h: %d\n", count, _bg->width, _bg->height);
      count = 0;
      // Erosion
      for(int i=0; i< _bg->height; ++i)
	{
	  for(int j=0; j< _bg->width; ++j)
	    {
	      if (_mask[i*_bg->width+j]) 
		continue;
	     	       
	      for(int k= -R; k<= R; k++)
		{
		  int r = i+k;
		  int c = j+k;
		  if(r>=0 && r< _bg->height && c>=0 && c< _bg->width)
		    temp[r*_bg->width + c] = 0;		      
		}
	    }
	}

      for (int i=0; i< _mask.size(); ++i)
	{
	  _mask[i] = _mask[i] & temp[i];
	  //	  temp[i] = 0; // reset to all 0 for use in dialation
	  if(_mask[i])
	    count++;
	}
      //      printf("%d left\n", count);
    }

    int Dilate2D(int R = 1)
    {
      int count = 0;
      std::vector<bool> temp(_mask.size(), 0);
      // Dialation
      for(int i=0; i< _bg->height; ++i)
	{
	  for(int j=0; j< _bg->width; ++j)
	    {
	      if (!_mask[i*_bg->width+j]) 
		continue;
	       
	      for(int k= -R; k<= R; k++)
		{
		  int r = i+k;
		  int c = j+k;
		  if(r>=0 && r< _bg->height && c>=0 && c< _bg->width)
		      temp[r*_bg->width + c] = 1;
		}
	    }
	}

      count = 0;
      for (int i=0; i< _mask.size(); ++i)
	{
	  _mask[i] = _mask[i] | temp[i];
	  if (_mask[i])
	    count++;
	}
      //      printf("%d left\n", count);
      return count;
    }
  };
};
