#include "Image.hpp"

Image::Image()
{

}

void Image::printImage()
{
  print("id: " << id);
  print("f: " << f);
  print("k1: " << k1);
  print("k2: " << k2);
  print("R: " << R);
  print("t: " << t);
  print("name:" << name);

}
