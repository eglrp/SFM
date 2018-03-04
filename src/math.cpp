#include "math.hpp"


void triangulateTrackDLT(const Track& track, const ImagesVec& images)
{
  int n = track.nPoints;
  Matrix<float,Dynamic,3> A;
  Matrix<float,Dynamic,1> B;
  A.resize(2*n,3);
  B.resize(2*n,1);

  int i = 0;
  for(auto el:track.occurrences)
  {
    int camKey = el.first;
    float x = el.second(0);
    float y = el.second(1);

    // Look for image:
    // predicate to find the right camera
    auto pred = [camKey](const Image& im)
    {
      return (im.id == camKey);
    };
    auto it = find_if(begin(images),end(images),pred);

    Matrix<float,3,4> P;
    computeProjectionMatrix(it->R,it->t, P);


    float f = it->f;
    float k1 = it->k1;
    float k2 = it->k2;

    float r2 = x*x + y*y;
    float r4 = r2 * r2;

    float distortion = 1.0 + k1 * r2 + k2 *r4;

    x = f * distortion * x;
    y = f * distortion * y;



    RowVector4f p0,p1,p2;
    p0 = P.row(0);
    p1 = P.row(1);
    p2 = P.row(2);


    A.row(2*i + 0) = Vector3f(x*p2(0) - p0(0), x*p2(1) - p0(1),x*p2(2) - p0(2));
    B(2*i + 0) = x*p2(3) - p0(3);
    A.row(2*i + 1) = Vector3f(y*p2(0) - p1(0), y*p2(1) - p1(1),y*p2(2) - p1(2));
    B(2*i + 1) = y*p2(3) - p1(3);

    i++;
  }
  // Solve by the Linear-LS method X = (x,y,z,1), non-homogeneous system

  JacobiSVD<MatrixXf> svd(A, ComputeThinU | ComputeThinV);
  Vector3f sol = svd.solve(B);
  sol = sol.normalized();

  print(sol.transpose());
  print(track.groundTruth.transpose());

}



void computeProjectionMatrix(const Matrix3f& R, const Vector3f& t, Matrix<float,3,4>& P)
{
  P << R, t;
}
