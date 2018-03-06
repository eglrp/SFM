#include "Image.hpp"

Image::Image()
{

}

void Image::printImage()
{
  print("name:" << name);
  print("id: " << id);
  print("f: " << f);
  print("k1: " << k1);
  print("k2: " << k2);
  print("R:" << std::endl << R);
  print("t: " << t.transpose());
}
