#include <stdio.h>
#include <iostream>

#include "SFM.hpp"

int main(int argc, char** argv)
{

  SFM* sfm = new SFM(argv[1],argv[2]);

  sfm->computeSFM();
  sfm->writePLY("output.ply");
  sfm->writePLYComparison("comparison.ply");
  sfm->writePLYGT("GT.ply");
  sfm->drawCameras("cameras.ply");



}
