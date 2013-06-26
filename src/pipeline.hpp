
#pragma once

#include "bg_subtractor.hpp"
#include "blob_extractor.hpp"
#include "resource_manager.hpp"

class Pipeline
{
  bool ReadConfigFile(std::string& filepath);

  void run();
  BGS::ImageFilter _bgs;
  BlobExtraction::KDTreeBE _be;
  ResourceManager _rm;
};
