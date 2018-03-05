#ifndef IMAGE_HPP
#define IMAGE_HPP

#include <vector>
#include <Eigen/Core>
#include <iostream>
#include <string>

#include "definitions.hpp"

using namespace std;
using namespace Eigen;

class Image
{
  public:
    Image();
    string name;
    int id;
    int w;
    int h;
    double f;
    double k1;
    double k2;
    Matrix3d R;
    Vector3d t;

    void printImage();
};

typedef vector< Image > ImagesVec;

#endif
