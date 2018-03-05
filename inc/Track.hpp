#ifndef TRACK_HPP
#define TRACK_HPP

#include <vector>
#include <Eigen/Core>
#include <iostream>
#include <string>
#include "definitions.hpp"
using namespace Eigen;
using namespace std;

typedef pair<int, Eigen::Vector2d> KeyPoint;
typedef vector< KeyPoint  > Occurrences;

class Track
{

  public:
    Track();
    Vector3d groundTruth;
    Vector3i color;
    Occurrences occurrences;
    int nPoints;
    void printTrack();
    double reProjectionError();
  private:

};

typedef vector< Track > Tracks;


#endif
