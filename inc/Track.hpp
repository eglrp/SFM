#ifndef TRACK_HPP
#define TRACK_HPP

#include <vector>
#include <iostream>
#include <string>
#include <Eigen/Core>

#include "definitions.hpp"

using namespace Eigen;
using namespace std;

typedef pair<int, Vector2d> KeyPoint;
typedef vector< KeyPoint  > Occurrences;

class Track
{

  public:
    Track();
    //the id
    int id;
    // the GT from the Noah Bundler
    Vector3d groundTruth;
    // the color from the Noah Bundler
    Vector3i color;
    // to be found
    Vector3d worldPosition;

    // Correspondences
    Occurrences occurrences;
    int nPoints;
    void printTrack();
  private:

};
typedef vector< Track > Tracks;



#endif
