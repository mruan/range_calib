/* 
   Definitions of some simple points
 */


#pragma once

/***********************************************************
                   Basic Type definition
 **********************************************************/
template <typename T>
struct Point3
{
  Point3(T a=0, T b=0, T c=0):x(a),y(b),z(c){}
  Point3(T p[3]):x(p[0]),y(p[1]),z(p[2]){}
  T x, y, z;
};

template <typename T>
struct Point2
{
  Point2(T a, T b):u(a),v(b){}
  T u,v;
};

typedef Point3<double> Point3d;
typedef Point2<double> Point2d;
typedef Point2<int>    Point2i;

