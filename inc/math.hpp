#ifndef MATH_SFM_HPP
#define MATH_SFM_HPP

#include <Eigen/Core>
#include <Eigen/SVD>

#include <assert.h> 
#include "ceres/ceres.h"
#include <algorithm>
#include <math.h>

#include "definitions.hpp"

using namespace Eigen;
using namespace std;

Vec3 triangulateTrackDLT(const Track& track, const ImagesVec& images);

void IterativeLinearLSTriangulation(const Track& track, const ImagesVec& images);

void undistortPoint(Vector2f inputPoint, Vector2f& outputPoint, float f, float k1,float k2);

void computeProjectionMatrix(const Eigen::Matrix3f& R,
  const Eigen::Vector3f& t,
  Eigen::Matrix<float, 3,4>& P);

void triangulateCERES(Tracks& tracks, ImagesVec& images);


bool TriangulateNViewAlgebraic
(
  const Mat3X & points,
  const std::vector<Mat34>& poses,
  Vec4* X
);

/*bool TriangulateNViewAlgebraic
(
  const Matrix<float, 3, 3> & points,
  const std::vector<Matrix<float, 3, 4>>& poses,
  Vector4f* X
);
*/
#endif
