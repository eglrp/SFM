#include <stdio.h>
#include <iostream>

#include "SFM.hpp"
#include "math.hpp"
#include "Eigen/Core"
#include "definitions.hpp"
#include <assert.h>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{

  // We can project a 3D point to our pixel coordinates easily:
  // Define our point
  Vector4d X(10.0,5.0,4.0,1.0);
  // Define R and t for the camera
  Matrix3d R = Matrix3d::Identity();
  Vector3d t(5,0,1);

  //Define intrinsics parameters
  double f = 1000;
  double k1 = 0.1;
  double k2 = 0.01;
  // The pixel coordinates vector
  Vector2d pixelCoords;
  // Compute
  project3DPointToPixel(X,pixelCoords,R,t,f,k1,k2);
  
  print(pixelCoords.transpose());




  /*


  SFM* sfm = new SFM(argv[1],argv[2]);

  sfm->computeSFM();
  sfm->writePLY("output.ply");
  sfm->writePLYComparison("comparison.ply");
  sfm->writePLYGT("GT.ply");
  sfm->drawCameras("cameras.ply");
  */


}
