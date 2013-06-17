
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class BallFilter
{
public:
  BallFilter(double r, double err, int width, int height);

  void SetInputPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  bool ApplyFilter();

  const std::vector<bool>& GetMask();

  //  void Reset();

  //  void Reset(double r, int width, int height);

private:
#define MIN_COUNT 10
  double _radius;
  double _err;
  int _w, _h;
  std::vector<bool> _flt_mask; // filter mask
  // std::vector<bool> _pre_mask;
  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
};
