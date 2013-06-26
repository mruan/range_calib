
#include "pipeline.hpp"

bool Pipeline::ReadConfigFile(std::string& filepath)
{

}

void Pipeline::Run()
{
  for(int i=0; i< num_sensors; ++i)
    {
      // Load background(model)
      for(int j=0; j< num_bg_pcds; ++j)
	;
      // bgs.SetBackgroundCloud();

      for(int j=0; j< num_fg_pcds; ++j)
	{
	  // Load raw cloud
	  
	  // Subtract background
	  //	  CloudType::Ptr fg = _bgs.GetForegroundCloud;

	  // Extract blob:
	  //	  _be.SetInputCloud(cloud);

	  // Fit Sphere and grab the center
	  //	  _be.ExtractBallCenter();

	  // Add the center to resource manager
	}
    }

  // run the linear solver to compute an initial guess

  // run the nlsqr solver
}
