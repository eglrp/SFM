#include "Camera.hpp"

Camera::Camera()
{

}

void Camera::printCamera()
{
  printRed("Camera id: " << id);
  print("name:" << name);
  print("f: " << f);
  print("k1: " << k1);
  print("k2: " << k2);
  print("R:" << std::endl << R);
  print("t: " << t.transpose());
}
