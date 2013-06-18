#include "ball_filter.hpp"

BallFilter::BallFilter(double r, double err, int width, int height)

{
}

void BallFilter::SetInputPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

  //  printf("W = %d, H= %d\n", cloud->width, cloud->height);
}

const std::vector<bool>& BallFilter::GetMask()
{
  
}


bool BallFilter::ApplyFilter()

