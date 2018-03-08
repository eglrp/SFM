#ifndef MATH_SFM_HPP
#define MATH_SFM_HPP

#include <Eigen/Core>
#include <Eigen/SVD>

#include <assert.h>
#include "ceres/ceres.h"
#include "Camera.hpp"
#include "Track.hpp"
#include <algorithm>
#include <math.h>


#include "definitions.hpp"

using namespace Eigen;
using namespace std;


// Triangulates the track given with the intrinsics information from images
Vec3 triangulateTrackDLT(Track& track, const CamerasVec& cameras);

// Not implemented
void IterativeLinearLSTriangulation(const Track& track, const CamerasVec& cameras);

// Distorts a pixel using the pinhole camera model
// p' = r(p) * p
void distortPoint(Vector2d undistortedPoint, Vector2d& distortedPoint, double k1, double k2);

// Undistorts radial distortion created by the lens camera model
void undistortPoint(Vector2d inputPoint, Vector2d& outputPoint,
  double cx,double cy, double f, double k1,double k2);

// Projects the world position to pixel coordinates following the dataset specification
//  P = R * X + t       (conversion from world to camera coordinates)
//  p = -P / P.z        (perspective division)
//  p' = f * r(p) * p
void project3DPointToPixel(Vector4d& worldPosition, Vector2d& pixelCoords,
  Matrix3d R, Vector3d t, double f, double k1, double k2);

  // Projects the world position to camera coordinates following the dataset specification
  //  P = R * X + t       (conversion from world to camera coordinates)
  //  p = -P / P.z        (perspective division)
void project3DPointToCamera(Vector4d& inputPoint, Vector2d& outputPoint,
  Matrix3d& R, Vector3d& t);

  // Projects the camera position to pixel coordinates following the dataset specification
  //  q = K * p
  // K = [fx, 0, cx; 0, fy, cy; 0, 0,1]
void projectCameraPointToPixel(Vector2d& inputPoint, Vector2d& outputPoint,
  double f, double cx, double cy);

// Computes a projection Matrix P given R and t
void computeProjectionMatrix(const Eigen::Matrix3d& R,
  const Eigen::Vector3d& t,
  Eigen::Matrix<double, 3,4>& P);

// Not implemented
void triangulateCERES(Tracks& tracks, CamerasVec& cameras);

// Calculates the reprojection error for a track.
double calculateReprojectionError(Track& track, CamerasVec& cameras);

// Triangulates N views
Vec4 TriangulateNViewsHomogeneous
(
  const Mat3X & points,
  const std::vector<Mat34>& poses
);
Vec4 TriangulateNViewsNonHomogeneous
(
  const Mat3X & points,
  const std::vector<Mat34>& poses
);

void undistortedPixelToCamera(Vector2d inputPoint, Vector2d& cameraCoords,double cx, double cy, double f, double k1, double k2);
void undistortPointInPixels(Vector2d inputPoint, Vector2d& outputPoint, double k1,double k2);
void undistortOpenCV(Vector2d inputPoint, Vector2d& cameraCoords,double cx, double cy, double f, double k1, double k2);
#endif
