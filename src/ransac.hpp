
#ifndef CUSTOM_RANSAC
#define CUSTOM_RANSAC

template <class Model>
class RANSAC
{
  // from sampleconsensus
  int _max_iteration;
  double _threshold;
  
  bool computeModel(int verbosity=0)
  {
    if (_threshold == std::numeric_limits<double>::max())
      {
	std::cout<< "[RANSAC] No threshold set\n"<<std::endl;
	return false;
      }

    int iteration = 0;
    int best_inliers_count = -INT_MAX;
    double k = 1.0;

    std::vector<int> selection;

    int inliers_count = 0;
    int skipped_count = 0;

    // supress infinite loops by just allowing 10x maximum allowed iterations
    // for invalid model parameters
    const int max_skip = _max_iterations*10;

    // Start the main loop
    while(iteration < k && skipped_count < max_skip)
      {
	// Get X samples which satisfy the model criteria
	// 1. Select random samples...

	// Search for inliers
	// 2. Compute a model

	// Select inliers that are within threshold
	// 3. Count the number of inliers

	// Better match?
	if (inliers_count > best_inliers_count)
	  {
	    best_inliers_count = inliers_count;
	    
	  }
      }
  }
  
};
#endif
