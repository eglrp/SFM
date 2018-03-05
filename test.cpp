#include <stdio.h>
#include <iostream>

#include "SFM.hpp"
#include "math.hpp"
#include "Eigen/Core"
#include "definitions.hpp"
#include "Image.hpp"
#include "Track.hpp"
#include <assert.h>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{

  string datasetFolder, imagesList, bundleFile;
  switch(argc)
  {
    case 1:
      datasetFolder = "../NotreDame/";
      imagesList = "../NotreDame/list.txt";
      bundleFile = "../NotreDame/notredame.out";
    break;
    case 2:
      datasetFolder = argv[1];
      imagesList = "../NotreDame/list.txt";
      bundleFile = "../NotreDame/notredame.out";
    break;
    case 3:
      datasetFolder = argv[1];
      imagesList = argv[2];
      bundleFile = "../NotreDame/notredame.out";
    break;
    case 4:
      datasetFolder = argv[1];
      imagesList = argv[2];
      bundleFile = argv[3];
    break;
    default:
    print ("Usage: ./SFM <path to dataset file> <Input image list> <Input bundle file> ");
    break;
  }


  // We can project a 3D point to our pixel coordinates easily:
  // Define our point
  Vector4d X(1.0,0.5,0.4,1.0);
  // Define R and t for the camera
  Matrix3d R = Matrix3d::Identity();
  Vector3d t(0.5,0.0,0.1);

  //Define intrinsics parameters
  double f = 1000;
  double k1 = 0.1;
  double k2 = 0.01;
  // The pixel coordinates vector
  Vector2d pixelCoords;
  // Compute
  project3DPointToPixel(X,pixelCoords,R,t,f,k1,k2);

  print("The 3D world point (" << X.transpose() << ") is projected to (" << pixelCoords.transpose() << ") pixel coords.");
  print("Camera fisheye model:")
  print("Rotation Matrix: ")
  print(R)
  print("Translation vector")
  print(t.transpose())
  print("Intrinsics: focal length f: " << f << ". Distortion parameters (k1,k2): ("<< k1 <<"," << k2 <<")")
  print("")


  // Images and tracks can be created with ease:
  ImagesVec images(2);

  images[0].id = 0;
  images[1].id = 1;
  // Both cameras have the same direction
  images[0].R = R;
  images[1].R = R;
  // But different location
  images[0].t = Vector3d(-1,0,0);
  images[1].t = Vector3d(1,0,0);

  // Similar intrinsics
  images[0].f = 500;
  images[1].f = 700;

  images[0].k1 = 0.01;
  images[1].k1 = 0.1;

  images[0].k2 = 0.1;
  images[1].k2 = 0.01;

  Vector2d pixelCoordsCam0,pixelCoordsCam1;

  project3DPointToPixel(X,pixelCoordsCam0,images[0].R,images[0].t,images[0].f,images[0].k1,images[0].k2);
  project3DPointToPixel(X,pixelCoordsCam1,images[1].R,images[1].t,images[1].f,images[1].k1,images[1].k2);


  Occurrences occurrences(2);
  occurrences[0] = KeyPoint(0,pixelCoordsCam0);
  occurrences[1] = KeyPoint(1,pixelCoordsCam1);

  Track track;
  track.groundTruth = Vector3d(X(0),X(1),X(2));
  track.color = Vector3i(255,0,0);

  track.nPoints = 2;
  track.occurrences = occurrences;

  Vec3 output = triangulateTrackDLT(track,images);

  // We can project all the points (the GT in this case to check) to a single camera plane
  // Image boundaries are not checked
  SFM sfmProjection(datasetFolder,imagesList, bundleFile);

  Image im = sfmProjection.getImages()[0];
  Tracks tracks = sfmProjection.getTracks();

  Matrix<double,Dynamic,6> cloudPoint;
  cloudPoint.resize(tracks.size(),6);
  Matrix3d R_ = im.R;
  Vector3d t_ = im.t;

  //Define intrinsics parameters
  double f_ = im.f;
  double k1_ = im.k1;
  double k2_ = im.k2;

  int j = 0;
  for(auto el: tracks)
  {
    Vector4d X_(el.groundTruth(0),el.groundTruth(1),el.groundTruth(2),1);
    Vector3d color = el.color.cast<double>();
    Vector2d pixelCoordsCam;
    project3DPointToPixel(X_,pixelCoordsCam,R_,t_,f_,k1_,k2_);
    Matrix<double,3,4> P;
    computeProjectionMatrix(R_,t_,P);
    Vector3d cameraCoords = P * X_;
    cloudPoint(j,0) = pixelCoordsCam(0);
    cloudPoint(j,1) = pixelCoordsCam(1);
    cloudPoint(j,2) = 1;
    cloudPoint(j,3) = color(0);
    cloudPoint(j,4) = color(1);
    cloudPoint(j,5) = color(2);
    j++;
  }
  sfmProjection.setCloudPoint(cloudPoint);
  sfmProjection.writePLY("projectionTest.ply");

  print("")
  print("")

// We can triangulate by passing our track and our images.
// Create a SFM pipeline with a known dataset.
  SFM sfm(datasetFolder,imagesList, bundleFile);

  sfm.computeSFM();
  print("Reconstruction error: " << sfm.reprojectionError());
  print("GT error: " << sfm.GTError());

  sfm.writePLY("output.ply");
  sfm.writePLYGT("GT.ply");
  sfm.drawCameras("cameras.ply");

}
