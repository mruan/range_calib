/*
  Generic sensor classes
 */

#pragma once

#include <ceres/ceres.h>

#include <iostream>
#include <vector>

#include "types.hpp"
#include "error_models.hpp"
#include "linearTF_solver.hpp"

/************************************************************
                   Sensor Type definition
 ************************************************************/
class ResourceManager;// forward declaration

class Sensor
{
  friend class ResourceManager;
public:
  Sensor() { InitParameters(); }
  Sensor(std::string _name):name(_name) { InitParameters(); }
  Sensor(std::istream& is){ InitParameters(); ReadBaseInfo(is); };

  ~Sensor(){delete q; delete t;}
  
  void GetTransform(Eigen::Vector3f& origin, Eigen::Quaternionf& orientation)
  {
    origin(0) = static_cast<float>(t[0]);
    origin(1) = static_cast<float>(t[1]);
    origin(2) = static_cast<float>(t[2]);
    orientation.w() = static_cast<float>(q[0]);
    orientation.x() = static_cast<float>(q[1]);
    orientation.y() = static_cast<float>(q[2]);
    orientation.z() = static_cast<float>(q[3]);
  }

  // Add one observation, passed as a pointer to an array
  virtual void AddObservation(int i, double* p) =0;

  // Solve a linear problem to find an initial guess
  virtual void SolveLinearTf(std::vector<Point3d>&) = 0;

  // Initialize a ceres nonlinear least square problem (residual block)
  virtual void InitCeresProblem(ceres::Problem& prob, 
				std::vector<Point3d>& landmarks, bool fixed) = 0;

  // Write sensor information to an output stream
  virtual void WriteToStream (std::ostream& os) = 0;
  virtual void ReadFromStream(std::istream& is) = 0;

  //  void SetExtrinsic(double  
protected:
  // Write base information to out-stream
  void WriteBaseInfo(std::ostream& os)
  {
    os << "Name: " << name << std::endl;
    os << "Quaternion: " << q[0] <<" "<<q[1]<<" " <<q[2]<<" "<< q[3]<<std::endl;
    os << "Translation: "<< t[0] <<" "<<t[1]<<" " <<t[2]<<std::endl;
  }

  // Read base information from in-stream
  void ReadBaseInfo(std::istream& is)
  {
    std::string token;
    is >> token;
    assert(token == "Name:");
    is >> name >> token;
    assert(token == "Quaternion:");
    is >> q[0] >> q[1] >> q[2] >> q[3] >> token;
    //    std::cout << "Quaternion: " << q[0] <<" "<<q[1]<<" " <<q[2]<<" "<< q[3]<<std::endl;
    assert(token == "Translation:");
    is >> t[0] >> t[1] >> t[2];
    //    std::cout << "Translation: "<< t[0] <<" "<<t[1]<<" " <<t[2]<<std::endl;
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
	q = new double[4];
	t = new double[3];
      }
    catch (std::bad_alloc&)
      {
	std::cerr << "Error allocating memory." << std::endl;
      }
    q[0] = 1.0;// unit quaternion
    q[1] = 0.0; q[2] = 0.0; q[3] = 0.0;
    t[0] = 0.0; t[1] = 0.0; t[2] = 0.0;
  }
};


/*************************************************
                Generic 3D sensor
 ************************************************/
class RangeSensor: public Sensor
{
public:
  // Default constructor
  RangeSensor(std::string _name, int numObs=0):Sensor(_name)
  {
    observations.reserve(numObs);
    landmark_idx.reserve(numObs);
  }

  // Initialize from an input stream
  RangeSensor(std::istream& is):Sensor(is)
  { ReadFromStream(is); }

  void AddObservation(int i, double* p)
  {
    landmark_idx.push_back(i);
    observations.push_back(Point3d(p[0], p[1], p[2]));
  }

  void SolveLinearTf(std::vector<Point3d>& landmarks)
  {
    LinearTfSolver lts;
    // TODO: Now assume no missing points, so copy the entire landmarks
    // vector is very inefficient
    lts.SetRefPoints(landmarks);
    lts.SetTgtPoints(observations);

    // Warning, old values of t and q are erased
    lts.EstimateTfSVD(t, q);
  }

  // Initialize a ceres::Problem
  void InitCeresProblem(ceres::Problem& prob, std::vector<Point3d>& landmarks, bool fixedCam)
  {
    for(int i=0; i < observations.size(); ++i)
      {
	Point3d& p = observations[i];
	int index  = landmark_idx[i];
	ceres::CostFunction* cf=Euclidean3DError::Create(p.x, p.y, p.z,fixedCam);
	
	ceres::LossFunction* lf=NULL;//new HuberLoss(1.0);
	prob.AddResidualBlock(cf, lf, q, t, &(landmarks[index].x));
      }
    ceres::LocalParameterization* lp = new ceres::QuaternionParameterization;
    prob.SetParameterization(q, lp);
  }

  // Polymophic function
  void WriteToStream(std::ostream& os)
  {
    os << "Type: RangeSensor" << std::endl;
    WriteBaseInfo(os);
    os << "NumObservations: " << observations.size() << std::endl;
    for (int i=0; i < observations.size(); i++)
      {
	Point3d& p = observations[i];
	os << landmark_idx[i] <<" "<<p.x<<" "<<p.y<<" "<<p.z<<std::endl;
      }
  }

  void ReadFromStream(std::istream& is)
  {
    std::string token;
    is >> token;
    assert(token == "NumObservations:");
    int numObs;
    is >> numObs;
    observations.resize(numObs);
    landmark_idx.resize(numObs);
    for(int i=0; i< numObs; ++i)
      {
	Point3d& p = observations[i];
	is >> landmark_idx[i] >> p.x >> p.y >> p.z;
      }
  }

private:
  std::vector<Point3d> observations;
  std::vector<int> landmark_idx;
};

/*************************************************
               Camera (2D sensor)
 ************************************************/
class Camera: public Sensor
{
public:
  Camera(){}
  Camera(std::istream& is){}
  Camera(std::string _name):Sensor(_name){}

  void AddObservation(int i, double* p){}
  void ReadFromStream(std::istream& is){}
  void WriteToStream (std::ostream& os){}
  void SolveLinearTf(std::vector<Point3d>&){}
  void InitCeresProblem(ceres::Problem& prob, std::vector<Point3d>& landmarks, bool fixed){}
private:
  //  double intrinsic[6];
  std::vector<Point2d> observations;
  std::vector<int> landmark_idx;
};
