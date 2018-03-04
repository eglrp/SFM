#ifndef MATH_HPP
#define MATH_HPP

#include <Eigen/Core>
#include <Eigen/SVD>
#include "definitions.hpp"

#include "ceres/ceres.h"
#include <algorithm>

using namespace Eigen;
using namespace std;

void triangulateTrackDLT(const Track& track, const ImagesVec& images);

void computeProjectionMatrix(const Eigen::Matrix3f& R,
  const Eigen::Vector3f& t,
  Eigen::Matrix<float, 3,4>& P);

void triangulateCERES(Tracks& tracks, ImagesVec& images);


#endif
