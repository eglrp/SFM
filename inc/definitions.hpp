#include <vector>
#include <Eigen/Core>

#define print(x) std::cout << x << std::endl;

//typedef std::vector < Point > PointSet;

struct Image
{
  int id;
  int w;
  int h;
  //std::vector < Point > points;
  double f;
  double k1;
  double k2;

  Eigen::Matrix3f R;
  Eigen::Vector3f t;

  void printImage()
  {
    print("id: " << id);
    print("f: " << f);
    print("k1: " << k1);
    print("k2: " << k2);
    print("R: " << R);
    print("t: " << t);

  }


};

typedef std::vector< Image > ImagesVec;
typedef std::pair<int, Eigen::Vector2f> KeyPoint;
typedef std::vector< KeyPoint  > Occurrences;

struct Track
{
  int id;
  Eigen::Vector3f groundTruth;
  Eigen::Vector3i color;
  Occurrences occurrences;

  void printTrack()
  {
    print("GT: " << groundTruth);
    print("Color: " << color);
    print("number of points: " << occurrences.size());
    for(auto el: occurrences)
    {
      print("camkey:" << el.first);
      print("Position:" << el.second);

    }
  }

};

typedef std::vector< Track > Tracks;
