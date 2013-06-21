#pragma once

#include <ceres/ceres.h>

#include <iostream>
#include <vector>
#include <string>
#include <assert.h>

#include "error_models.hpp"

/***********************************************************
                   Basic Type definition
 **********************************************************/
struct Point3d
{
  Point3d(double a=0, double b=0, double c=0):x(a),y(b),z(c){}
  Point3d(double p[3]):x(p[0]),y(p[1]),z(p[2]){}
  double x, y, z;
};

struct Point2i
{
  Point2i(int a, int b):u(a),v(b){}
  int u,v;
};

/************************************************************
                   Sensor Type definition
 ************************************************************/
class Sensor
{
public:
  Sensor() { InitParameters(); }
  Sensor(std::string _name):name(_name) { InitParameters(); }
  Sensor(std::istream& is);

  ~Sensor(){delete q; delete t;}

  virtual void InitProblem(ceres::Problem& prob, std::vector<Point3d>& landmarks) = 0;
  virtual std::ostream& operator<< (std::ostream& os) =0;
  //  void SetExtrinsic(double  
protected:
  // Write base information to out-stream
  void WriteBaseInfo(std::ostream& os)
  {
    os << "Name: " << name << std::endl;
    os << "Quaternion: " << q[0] <<" "<<q[1]<<" " <<q[2]<<" "<< q[3]<<std::endl;
    os << "Translation: "<< t[0] <<" "<<t[2]<<" " <<t[3]<<std::endl;
  }

  // Read base information from in-nstream
  void ReadBaseInfo(std::istream& is)
  {
    std::string token;
    is >> token;
    assert(token == "Name:");
    is >> name >> token;
    assert(token == "Quaternion:");
    is >> q[0] >> q[1] >> q[2] >> q[3] >> token;
    assert(token == "Translation:");
    is >> t[0] >> t[0] >> t[2];
  }

  // Parameters that will be optimized
  double* q;//quaternion
  double* t;//translation
  std::string name;

private:
  void InitParameters()
  {
    try
      {
	double* q = new double[4];
	double* t = new double[3];
      }
    catch (std::bad_alloc&)
      {
	std::cerr << "Error allocating memory." << std::endl;
      }
    q[0] = 1.0;// unit quaternion
  }
};

class Camera: public Sensor
{
public:
  Camera(std::istream& is){}
  Camera(std::string _name):Sensor(_name){}

  std::ostream& operator<< (std::ostream& os){}

  void InitProblem(ceres::Problem& prob, std::vector<Point3d>& landmarks){}
private:
  //  double intrinsic[6];
  std::vector<std::pair<int, Point2i> > observations;
};

class RangeSensor: public Sensor
{
public:
  // Initialize from an input stream
  RangeSensor(std::istream& is)
  {
    ReadBaseInfo(is);
    std::string token;
    is >> token;
    assert(token == "NumObservations:");
    int numObs;
    is >> numObs;
    observations.resize(numObs);
    std::vector<std::pair<int, Point3d> >::iterator it = observations.begin();
    while(it != observations.end())
      {
	is >> it->first >> it->second.x >> it->second.y >> it->second.z;
      }
  }

  // Default constructor
  RangeSensor(std::string _name):Sensor(_name){}

  std::ostream& operator<< (std::ostream& os)
  {
    os << "Type: RangeSensor" << std::endl;
    WriteBaseInfo(os);
    os << "NumObservations: " << observations.size() << std::endl;
    for (int i=0; i < observations.size(); i++)
      {
	Point3d& p = observations[i].second;
	os << observations[i].first <<" "<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;
      }
    return os;
  }

  // Initialize a ceres::Problem
  void InitProblem(ceres::Problem& prob, std::vector<Point3d>& landmarks)
  {
    for(int i=0; i < observations.size(); ++i)
      {
	Point3d& p = observations[i].second;
	int index  = observations[i].first;
	ceres::CostFunction* cf = new ceres::AutoDiffCostFunction<Euclidean3DError,3,4,3,3>(
				    new Euclidean3DError(p.x, p.y, p.z));

	ceres::LossFunction* lf = NULL;//new HuberLoss(1.0);
	prob.AddResidualBlock(cf, lf, q, t, &landmarks[index].x);
      }
    ceres::LocalParameterization* lp = new ceres::QuaternionParameterization;
    prob.SetParameterization(q, lp);
  }

private:
  std::vector<std::pair<int, Point3d> > observations;
};

/************************************************************
                   Resource Manager definition
 ************************************************************/
class ResourceManager
{
public:
  ResourceManager()
  {
    // Set up allowable sensor types
    _type2id["RangeSensor"] = 1;
    _type2id["Camera"]      = 2;
  }

  // Basic I/O
  bool ReadfromFile(std::istream& is)
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
	    _sensors[i] = new RangeSensor(is);
	    break;
	  case 2:
	    _sensors[i] = new Camera(is);
	    break;
	  default:
	    std::cerr<< "Unknown sensor type"<<std::endl;
	    break;
	  }
      }

    // Add all points locations (could be non-sense random guesses)
    for (int i=0; i< num_landmarks; ++i)
      is >> _landmarks[i].x >> _landmarks[i].y >> _landmarks[i].z;

    return true;
  }

  void WriteToFile(std::ostream& os)
  {
    os << "NumSensors: "<< _sensors.size() << std::endl;
    os << "NumLandmarks: " << _landmarks.size() << std::endl;

    // Write out sensor informations
    for(int i=0; i< _sensors.size(); ++i)
      os << _sensors[i];
    
    // Write out points informations
    for(int i=0; i< _landmarks.size(); ++i)
      {
	Point3d& p = _landmarks[i];
	os << p.x <<" " << p.y << " "<< p.z<<std::endl;
      }
  }

  // Construct a ceres::Problem 
  void BuildProblem(ceres::Problem& prob)
  {
    // make sure this manager has a problem to create...
    assert(!_sensors.empty() && !_landmarks.empty());

    for(int i=0; i< _sensors.size(); i++)
      _sensors[i]->InitProblem(prob, _landmarks);
  }

  ~ResourceManager()
  {
    for(int i=0; i< _sensors.size(); i++)
      delete _sensors[i];
  }
private:
  std::vector<Sensor*> _sensors;
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
*/
