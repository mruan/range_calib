#pragma once

#include <ceres/ceres.h>

#include <iostream>
#include <vector>
#include <string>
#include <assert.h>

#include <memory>

#include "types.hpp"
#include "generic_sensor.hpp"

/************************************************************
                   Resource Manager definition
 ************************************************************/
class ResourceManager
{
public:
  ResourceManager(int num_sensors=0, int num_landmarks=0)
  {
    // Set up allowable sensor types
    _type2id["RangeSensor"] = 1;
    _type2id["Camera"]      = 2;

    _sensors.reserve(num_sensors);
    _landmarks.reserve(num_landmarks);
  }

  // API for adding something to the manager
  void AddSensor(std::shared_ptr<Sensor> sensor_sptr)
  {
    _sensors.push_back(sensor_sptr);
  }

  void AddLandmark(double cx, double cy, double cz)
  {
    _landmarks.push_back(Point3d(cx, cy, cz));
  }

  void GetSensorTransform(int i, Eigen::Vector3f& t, Eigen::Quaternionf& q)
  {
    _sensors.at(i)->GetTransform(t, q);
  }

  void SolveLinearRigidTf()
  {
    for(int i=0; i< _sensors.size(); ++i)
      _sensors[i]->SolveLinearTf(_landmarks);
  }

  // Basic I/O
  bool ReadFromStream(std::istream& is)
  {
    std::string token; // just a buffer
    int num_sensors, num_landmarks;

    // Skip comments (if there is any):
    is >> std::skipws >> token;
    while(token=="#")
      {
	std::getline (is, token);
	is >> token;
      }

    // Read in header information
    assert(token == "NumSensors:");
    is >> num_sensors >> token;
    _sensors.resize(num_sensors);

    assert(token == "NumLandmarks:");
    is >> num_landmarks;
    _landmarks.resize(num_landmarks);

    // Create each individual sensor
    for(int i=0; i< num_sensors; ++i)
      {
	is>>token;
	assert(token == "Type:");

	// Determin what type of sensor to create
	is>>token;	
	switch (_type2id[token])
	  {
	  case 1:
	    _sensors[i] = std::shared_ptr<Sensor>(new RangeSensor(is));
	    break;
	  case 2:
	    _sensors[i] = std::shared_ptr<Sensor>(new Camera(is));
	    break;
	  default:
	    std::cerr<< "Unknown sensor type"<<std::endl;
	    break;
	  }
      }

    // Add all points locations (could be non-sense random guesses)
    is >> token;
    assert(token == "Landmarks:");
    for (int i=0; i< num_landmarks; ++i)
      is >> _landmarks[i].x >> _landmarks[i].y >> _landmarks[i].z;

    return true;
  }

  void WriteToStream(std::ostream& os)
  {
    os << "NumSensors: "<< _sensors.size() << std::endl;
    os << "NumLandmarks: " << _landmarks.size() << std::endl;

    // Write out sensor informations
    for(int i=0; i< _sensors.size(); ++i)
      _sensors[i]->WriteToStream(os);
    
    // Write out points informations
    os << "Landmarks:" << std::endl;
    for(int i=0; i< _landmarks.size(); ++i)
      {
	Point3d& p = _landmarks[i];
	os << p.x <<" " << p.y << " "<< p.z<<std::endl;
      }
  }

  // Construct a ceres::Problem 
  void BuildCeresProblem(ceres::Problem& prob)
  {
    // make sure this manager has a problem to create...
    assert(!_sensors.empty() && !_landmarks.empty());

    // Set the first sensor as a fixed reference frame
    //    _sensors[0]->InitProblem(prob, _landmarks, true);
    // The rest sensors are allowed to move
    for(int i=0; i< _sensors.size(); i++)
      _sensors[i]->InitCeresProblem(prob, _landmarks, false);
  }

private:
  std::vector<std::shared_ptr<Sensor> > _sensors;
  std::vector<Point3d> _landmarks;
  std::map<std::string, int> _type2id;
};


/*

class Stereo: public Sensor
{
public:
  Stereo(std::string _name):Sensor(_name){}

  std::ostream& operator<< (std::ostream& os)
  {
    os << "Type: Stereo" << std::endl;
    PrintBaseInfo(os);
    return os;
  }
private:

};

  const std::vector<Point3d>& GetLandmarks()
  {
    return _landmarks;
  }

  //  void GetSensorObservations(int i)
  //  {
  //    return _sensors[i]->observations;
  //  }

  void SetSensorTf(int i, double t[3], double q[4])
  {
    std::copy(&t[0], &t[2], _sensors[i]->t);
    std::copy(&q[0], &q[3], _sensors[i]->q);
  }


*/
