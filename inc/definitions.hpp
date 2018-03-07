#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP

#include <vector>
#include <Eigen/Core>
#include <iostream>
#include <string>
#include "Track.hpp"

using namespace std;
using namespace Eigen;

#define print(x) cout << x << endl;
#define printRed(x) cout << "\033[0;31m" << x <<"\033[0m"<< endl;
// For ease of use

using Mat3X = Eigen::Matrix<double, 3, Eigen::Dynamic>;
using Mat34 = Eigen::Matrix<double, 3, 4>;
using Vec4 = Eigen::Vector4d;
using Vec3 = Eigen::Vector3d;
using Mat4 = Eigen::Matrix<double, 4, 4>;

#endif
