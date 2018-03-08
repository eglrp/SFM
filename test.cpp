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
  Vector3d t(0.5,-0.5,0.1);

  //Define intrinsics parameters
  double f = 10;
  double k1 = 0.01;
  double k2 = 0.0001;
  double cx = 0;
  double cy = 0;
  // The pixel coordinates vector
  Vector2d cameraCoords,pixelCoords,pixelCoordsDist;
  // Compute
  //project3DPointToPixel(X,pixelCoords,R,t,f,k1,k2);
  project3DPointToCamera(X,cameraCoords,R,t);
  projectCameraPointToPixel(cameraCoords,pixelCoords,f,cx,cy);
  distortPoint(pixelCoords,pixelCoordsDist,k1,k2);
  printBlue("Projection of a single point to pixel coordinates")
  print("Camera fisheye model:")
  print("Rotation Matrix: ")
  print(R)
  print("Translation vector")
  print(t.transpose())
  print("Intrinsics: focal length f: " << f << ". Distortion parameters (k1,k2): ("<< k1 <<"," << k2 <<")")
  print("3D world [" << X.transpose() << "] -- project3DPointToCamera --> [" << cameraCoords.transpose() << "] Camera Coords");
  print("Camera Coords [" << cameraCoords.transpose() << "] -- projectCameraPointToPixel --> [" << pixelCoords.transpose() << "] Pixel Coords (No distortion)");
  print("Pixel Coords (No distortion) [" << pixelCoords.transpose() << "] -- distortPoint --> [" << pixelCoordsDist.transpose() << "] Pixel Coords (Distortion)");
  Vector2d pixelCoords_recon,cameraCoords_recon;
  undistortPoint(pixelCoordsDist,pixelCoords_recon,cx,cy,f,k1,k2);
  print("Pixel Coords (Distortion) [" << pixelCoordsDist.transpose() << "] -- distortPoint --> [" << pixelCoords_recon.transpose() << "] Camera Coords");


  print("")


  // Check for a whole projection and back



  /*

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

  print("Point [" << testPoint.transpose() <<"] is projected to [" << cameraCoordsFromPixel0.transpose() <<
  "] in camera 0 and to [" << pixelCoordsCam0.transpose()<<"] in pixels")
  print("Point [" << testPoint.transpose() <<"] is projected to [" << cameraCoordsFromPixel1.transpose() <<
  "] in camera 1 and to [" << pixelCoordsCam1.transpose()<<"] in pixels")

  Vector2d distortedPoint0,distortedPoint1;
  distortPoint(pixelCoordsCam0,distortedPoint0,cameras[0].k1,cameras[0].k2);
  distortPoint(pixelCoordsCam1,distortedPoint1,cameras[1].k1,cameras[1].k2);

  print("Pixel coords ["<< pixelCoordsCam0.transpose() << "] is distorted to ["<< distortedPoint0.transpose() << "] at camera 0")
  print("Pixel coords ["<< pixelCoordsCam1.transpose() << "] is distorted to ["<< distortedPoint1.transpose() << "] at camera 1")

  Vector2d undistortedFromDistorted0,undistortedFromDistorted1;

  undistortedPixelToCamera(distortedPoint0,undistortedFromDistorted0,0,0,cameras[0].f,cameras[0].k1,cameras[0].k2);

  printRed("Distorted point [" << distortedPoint0.transpose() << "] is projected to [" << undistortedFromDistorted0.transpose() << "] in camera coordinates at camera 0")

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
  */
}
