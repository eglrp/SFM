#ifndef MATH_SFM_HPP
#define MATH_SFM_HPP

#include <Eigen/Core>
#include <Eigen/SVD>

#include <assert.h>
#include "ceres/ceres.h"
#include "Image.hpp"
#include "Track.hpp"
#include <algorithm>
#include <math.h>

#include "definitions.hpp"

using namespace Eigen;
using namespace std;

Vec3 triangulateTrackDLT(const Track& track, const ImagesVec& images);

void IterativeLinearLSTriangulation(const Track& track, const ImagesVec& images);

void undistortPoint(Vector2d inputPoint, Vector2d& outputPoint,double cx,double cy, double f, double k1,double k2);

void project3DPointToPixel(const Vector4d& inputPoint, Vector2d& outputPoint,
  Matrix3d R, Vector3d t, double f, double k1, double k2);

void computeProjectionMatrix(const Eigen::Matrix3d& R,
  const Eigen::Vector3d& t,
  Eigen::Matrix<double, 3,4>& P);

void triangulateCERES(Tracks& tracks, ImagesVec& images);

double calculateReprojectionError(const Track& track, const ImagesVec& images);

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
