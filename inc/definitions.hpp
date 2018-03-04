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

  Eigen::Matrix3f R;
  Eigen::Vector3f t;

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
typedef pair<int, Eigen::Vector2f> KeyPoint;
typedef vector< KeyPoint  > Occurrences;

struct Track
{
  int id;
  Eigen::Vector3f groundTruth;
  Eigen::Vector3f position3D;
  Eigen::Vector3i color;
  Occurrences occurrences;
  int nPoints;
  void printTrack()
  {
    print("GT: " << groundTruth);
    print("Color: " << color);
    print("number of points: " << occurrences.size());
    for(auto el: occurrences)
    {
      print("camkey:" << el.first);
      print("Position:" << el.second);
    }
  }

};

typedef vector< Track > Tracks;

#endif
