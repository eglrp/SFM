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


// Triangulates the track given with the intrinsics information from images
Vec3 triangulateTrackDLT(Track& track, const ImagesVec& images);

// Not implemented
void IterativeLinearLSTriangulation(const Track& track, const ImagesVec& images);

// Undistorts radial distortion created by the fisheye lens camera model
void undistortPoint(Vector2d inputPoint, Vector2d& outputPoint,
  double cx,double cy, double f, double k1,double k2);



// Projects the world position to pixel coordinates following the dataset specification
//  P = R * X + t       (conversion from world to camera coordinates)
//  p = -P / P.z        (perspective division)
//  p' = f * r(p) * p
void project3DPointToPixel(const Vector4d& inputPoint, Vector2d& outputPoint,
  Matrix3d R, Vector3d t, double f, double k1, double k2);


// Computes a projection Matrix P given R and t
void computeProjectionMatrix(const Eigen::Matrix3d& R,
  const Eigen::Vector3d& t,
  Eigen::Matrix<double, 3,4>& P);

// Not implemented
void triangulateCERES(Tracks& tracks, ImagesVec& images);

// Calculates the reprojection error for a track.
double calculateReprojectionError(Track& track, ImagesVec& images);

// Triangulates N views
bool TriangulateNViewAlgebraic
(
  const Mat3X & points,
  const std::vector<Mat34>& poses,
  Vec4* X
);
#endif
