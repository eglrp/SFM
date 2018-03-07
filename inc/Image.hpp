#ifndef IMAGE_HPP
#define IMAGE_HPP

#include <vector>
#include <iostream>
#include <string>
#include <Eigen/Core>

#include "definitions.hpp"

class Image
{
  public:
    Image();
    // name of the image
    string name;
    // id, the same as the line they appear in the list.txt
    int id;
    // Height and width
    int h;
    int w;
    // intrinsics
    double f;
    double k1;
    double k2;
    // extrinsics
    Matrix3d R;
    Vector3d t;
    // for debugg purposes
    void printImage();
};

typedef vector< Image > ImagesVec;

#endif
