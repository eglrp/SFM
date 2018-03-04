#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

#include <vector>
#include <Eigen/Core>
#include <iostream>
#include <string>

using namespace std;

#define print(x) cout << x << endl;

//typedef std::vector < Point > PointSet;

struct Image
{
  string name;
  int id;
  int w;
  int h;
  //std::vector < Point > points;
  double f;
  double k1;
  double k2;

  Eigen::Matrix3d R;
  Eigen::Vector3d t;

  void printImage()
  {
    print("id: " << id);
    print("f: " << f);
    print("k1: " << k1);
    print("k2: " << k2);
    print("R: " << R);
    print("t: " << t);
    print("name:" << name);

  }


};

typedef vector< Image > ImagesVec;
typedef pair<int, Eigen::Vector2d> KeyPoint;
typedef vector< KeyPoint  > Occurrences;

struct Track
{
  Eigen::Vector3d groundTruth;
  Eigen::Vector3i color;
  Occurrences occurrences;
  int nPoints;
  void printTrack()
  {
    print("GT: " << groundTruth.transpose());
    print("Color: " << color.transpose());
    print("number of points: " << occurrences.size());
    for(auto el: occurrences)
    {
      print("camkey:" << el.first << ". Position:" << el.second(0) << "," <<el.second(1));
    }
  }

};

typedef vector< Track > Tracks;


using Mat3X = Eigen::Matrix<double, 3, Eigen::Dynamic>;
using Mat34 = Eigen::Matrix<double, 3, 4>;
using Vec4 = Eigen::Vector4d;
using Vec3 = Eigen::Vector3d;
using Mat4 = Eigen::Matrix<double, 4, 4>;

#endif
