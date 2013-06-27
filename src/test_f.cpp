/*
  Try to find the focal length in terms of pixel
  so that 3D distance can be converted to pixel distance directly
 */
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;

int main(int argc, char** argv)
{
  std::vector<int> pcd_idx = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

  pcl::PCDReader pcd;
  CloudT::Ptr c1(new CloudT);
  Eigen::Vector4f origin;
  Eigen::Quaternionf quat;
  int version;

  for (int j=0; j< pcd_idx.size(); ++j)
    {
      // Load both pcd files
      if (pcl::io::loadPCDFile<PointT>(argv[pcd_idx.at(j)], *c1)==-1)
	{
	  PCL_ERROR("Error loading file %s", argv[pcd_idx.at(j)]);
	  exit(1);
	}
      
      float num=0.0f, den=0.0f;
      for(int i=0; i< c1->size(); i++)
	{
	  int v = i / c1->width - c1->height/2;
	  int u = i % c1->width - c1->width /2;
	  
	  PointT& p = c1->points[i];

	  if (isnan(p.z))
	    continue;

	  num += (p.x*u+p.y*v)*p.z;
	  den += (  u*u+  v*v)*p.z*p.z;
	  /*
	    x/u=y/v=za 
	    SUM[(x-zua)^2 + (y-zva)^2]
	    SUM[(x-zua)zu + (y-zva)zv]
	    SUM[zxu - z^2 u^2 a + zyv - z*2 v^2 a] = 0
	    SUm[zxu + zyv] = SUM[(u^2+v^2)*z^2]a
	    
	    x = za u -> dx = za du = za R -> R = dx /za = dx f/z;
	  */
	}

      float f = den / num;
      printf("%d den=%f, num=%f, f = %10f\n", den, num, j, f);
    }
}
 
