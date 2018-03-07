#include <stdio.h>
#include <iostream>

#include "SFM.hpp"
#include "math.hpp"
#include "Eigen/Core"
#include "definitions.hpp"
#include "Camera.hpp"
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

  print("Camera fisheye model:")
  print("Rotation Matrix: ")
  print(R)
  print("Translation vector")
  print(t.transpose())
  print("Intrinsics: focal length f: " << f << ". Distortion parameters (k1,k2): ("<< k1 <<"," << k2 <<")")
  print("The 3D world point (" << X.transpose() << ") is projected to (" << pixelCoords.transpose() << ") pixel coords under these circumstances.");
  print("")



  printRed("Triangulation of one point Example")
  // Images and tracks can be created with ease:
  CamerasVec cameras(2);

  cameras[0].id = 0;
  cameras[1].id = 1;
  // Both cameras have the same direction
  cameras[0].R = R;
  cameras[1].R = R;
  // But different location
  cameras[0].t = Vector3d(-1,0,0);
  cameras[1].t = Vector3d(1,0,0);

  // Similar intrinsics
  cameras[0].f = 500;
  cameras[1].f = 700;

  cameras[0].k1 = 0.01;
  cameras[1].k1 = 0.1;

  cameras[0].k2 = 0.1;
  cameras[1].k2 = 0.01;

  print("Camera 0:")
  cameras[0].printCamera();

  print("Camera 1:")
  cameras[1].printCamera();

  Vector2d pixelCoordsCam0,pixelCoordsCam1;

  Vector4d testPoint(0,0,-10,1);

  project3DPointToPixel(testPoint,pixelCoordsCam0,cameras[0].R,cameras[0].t,cameras[0].f,cameras[0].k1,cameras[0].k2);
  project3DPointToPixel(testPoint,pixelCoordsCam1,cameras[1].R,cameras[1].t,cameras[1].f,cameras[1].k1,cameras[1].k2);

  Vector2d cameraCoordsFromPixel0,cameraCoordsFromPixel1;

  undistortPoint(Vector2d(pixelCoordsCam0(0),pixelCoordsCam0(1)),cameraCoordsFromPixel0,0,0,cameras[0].f,cameras[0].k1,cameras[0].k2);
  undistortPoint(Vector2d(pixelCoordsCam1(0),pixelCoordsCam1(1)),cameraCoordsFromPixel1,0,0,cameras[1].f,cameras[1].k1,cameras[1].k2);

  print("Point [" << testPoint.transpose() <<"] is projected to [" << pixelCoordsCam0.transpose() <<
  "] in image 0 and to [" << cameraCoordsFromPixel0.transpose()<<"] in camera 0")
  print("Point [" << testPoint.transpose() <<"] is projected to [" << pixelCoordsCam1.transpose() <<
  "]   in image 1 and to [" << cameraCoordsFromPixel1.transpose()<<"] in camera 1")


  Occurrences occurrences(2);
  occurrences[0] = KeyPoint(0,pixelCoordsCam0);
  occurrences[1] = KeyPoint(1,pixelCoordsCam1);

  Track track;
  track.groundTruth = Vector3d(testPoint(0),testPoint(1),testPoint(2));
  track.color = Vector3i(255,0,0);

  track.nPoints = 2;
  track.occurrences = occurrences;

  print("The point summary:")
  track.printTrack();

  Vector3d sol = triangulateTrackDLT(track,cameras);
  print("Point [" << testPoint.transpose() << "] has been triangulated to [" << sol.transpose() <<"].")
  double error = calculateReprojectionError(track,cameras);
  print("Reprojection error: " << error);

  // But what if we add a THIRD VIEW???? **DO WE DARE???**
  printRed("But what if we add a THIRD VIEW???? DO WE DARE???")
  // We create the camera
  Camera additionalCamera;
  additionalCamera.id = 2;
  additionalCamera.R = R;
  additionalCamera.t = Vector3d(0,1,0);
  additionalCamera.f = 600;
  additionalCamera.k1 = 0.1;
  additionalCamera.k2 = 0.1;
  additionalCamera.printCamera();
  cameras.push_back(additionalCamera);

  Vector2d pixelCoordsCam2;
  Vector2d cameraCoordsFromPixel2;
  project3DPointToPixel(testPoint,pixelCoordsCam2,cameras[2].R,Vector3d(0,1,0),cameras[2].f,cameras[2].k1,cameras[2].k2);
  undistortPoint(Vector2d(pixelCoordsCam2(0),pixelCoordsCam2(1)),cameraCoordsFromPixel2,0,0,cameras[2].f,cameras[2].k1,cameras[2].k2);

  print("Point [" << testPoint.transpose() <<"] is projected to [" << pixelCoordsCam2.transpose() <<
  "] in image 2 and to [" << cameraCoordsFromPixel2.transpose()<<"] in camera 2")

  // Update the track
  track.occurrences.push_back(KeyPoint(2,pixelCoordsCam2));
  track.nPoints = 3;
  track.id = 0;
  print("The new point summary:")
  track.printTrack();

  // And triangulate again!
  sol = triangulateTrackDLT(track,cameras);
  double newError = calculateReprojectionError(track,cameras);
  print("Point [" << testPoint.transpose() << "] has been triangulated to [" << sol.transpose() <<"].")
  print("Reprojection error: " << newError << "(+" << newError - error << ")")

  print("")
  // We can project all the points (the GT in this case to check) to a single camera plane
  // Image boundaries are not checked
  SFM sfmProjection(datasetFolder,imagesList, bundleFile);

  Camera im = sfmProjection.getCameras()[0];
  Tracks tracks = sfmProjection.getTracks();

  Matrix<double,Dynamic,6> cloudPoint;
  //cloudPoint.resize(tracks.size(),6);
  Matrix3d R_ = im.R;
  Vector3d t_ = im.t;

  double w = im.w;
  double h = im.h;

  double min_w = -w/2;
  double max_w = w/2;
  double min_h = -h/2;
  double max_h = h/2;

  //Define intrinsics parameters
  double f_ = im.f;
  double k1_ = im.k1;
  double k2_ = im.k2;
/*
  for(int j = 0; j < tracks.size(); j++)
  {

    Vector4d X_(tracks[j].groundTruth(0),tracks[j].groundTruth(1),tracks[j].groundTruth(2),1);
    Vector3d color = tracks[j].color.cast<double>();
    Vector3d cameraCoords;
    Matrix<double,3,4> P;
    computeProjectionMatrix(R,t,P);
    cameraCoords = P * X_;
    cameraCoords = -cameraCoords/cameraCoords(2);
    cloudPoint(j,0) = cameraCoords(0);
    cloudPoint(j,1) = cameraCoords(1);
    cloudPoint(j,2) = 1;
    cloudPoint(j,3) = color(0);
    cloudPoint(j,4) = color(1);
    cloudPoint(j,5) = color(2);
  }
  sfmProjection.setCloudPoint(cloudPoint);
  sfmProjection.writePLY("projectionToCameraTest.ply");
  */
std::vector<Matrix<double,1,6> > pointsToDraw;
for(int j = 0; j < tracks.size(); j++)
  {
    Vector4d X_(tracks[j].groundTruth(0),tracks[j].groundTruth(1),tracks[j].groundTruth(2),1);
    Vector3d color = tracks[j].color.cast<double>();
    Vector2d pixelCoordsCam;
    project3DPointToPixel(X_,pixelCoordsCam,R_,t_,f_,k1_,k2_);

    double pixelRow = pixelCoordsCam(0);
    double pixelCol = pixelCoordsCam(1);

    if(pixelCol >= min_h && pixelCol <= max_h && pixelRow >= min_w && pixelRow <= max_w)
    {
      double r = color(0);
      double g = color(1);
      double b = color(2);
      Matrix<double,1,6> pointToDraw;
      pointToDraw(0) = pixelCol;
      pointToDraw(1) = pixelRow;
      pointToDraw(2) = 1.0;
      pointToDraw(3) = r;
      pointToDraw(4) = g;
      pointToDraw(5) = b;

      pointsToDraw.push_back(pointToDraw);
    }
  }
  cloudPoint.resize(pointsToDraw.size(),6);
  for(int iPointToDraw = 0; iPointToDraw < pointsToDraw.size(); iPointToDraw ++)
  {
    cloudPoint.row(iPointToDraw) = pointsToDraw[iPointToDraw];
  }

  sfmProjection.setCloudPoint(cloudPoint);
  sfmProjection.writePLY("projectionToPixelsTest.ply");
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
