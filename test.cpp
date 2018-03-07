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

  //Vector4d testPoint(0,0,-10,1);
  Vector4d testPoint(0.05,-0.02,0.05,1);

  project3DPointToPixel(testPoint,pixelCoordsCam0,cameras[0].R,cameras[0].t,cameras[0].f,cameras[0].k1,cameras[0].k2);
  project3DPointToPixel(testPoint,pixelCoordsCam1,cameras[1].R,cameras[1].t,cameras[1].f,cameras[1].k1,cameras[1].k2);

  Vector2d cameraCoordsFromPixel0,cameraCoordsFromPixel1;

  project3DPointToCamera(testPoint, cameraCoordsFromPixel0,cameras[0].R, cameras[0].t);
  project3DPointToCamera(testPoint, cameraCoordsFromPixel1,cameras[1].R, cameras[1].t);

  //undistortPoint(Vector2d(pixelCoordsCam0(0),pixelCoordsCam0(1)),cameraCoordsFromPixel0,0,0,cameras[0].f,cameras[0].k1,cameras[0].k2);
  //undistortPoint(Vector2d(pixelCoordsCam1(0),pixelCoordsCam1(1)),cameraCoordsFromPixel1,0,0,cameras[1].f,cameras[1].k1,cameras[1].k2);

  print("Point [" << testPoint.transpose() <<"] is projected to [" << cameraCoordsFromPixel0.transpose() <<
  "] in camera 0 and to [" << pixelCoordsCam0.transpose()<<"] in pixels")
  print("Point [" << testPoint.transpose() <<"] is projected to [" << cameraCoordsFromPixel1.transpose() <<
  "] in camera 1 and to [" << pixelCoordsCam1.transpose()<<"] in pixels")


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
  printGreen("Reprojection error: " << error);

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

  print("Point [" << testPoint.transpose() <<"] is projected to [" << cameraCoordsFromPixel2.transpose() <<
  "] in camera 1 and to [" << pixelCoordsCam2.transpose()<<"] in pixels 0")

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
  printGreen("Reprojection error: " << newError << "(+" << newError - error << ")")
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

  print("")
  // We can project all the points (the GT in this case to check) to a single camera plane
  // Image boundaries are not checked
  sfm.projectCamera("CameraProjection.ply",0);

}
