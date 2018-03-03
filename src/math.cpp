#include "math.hpp"


void triangulateTrackDLT(const Track& track, const ImagesVec& images)
{
  int n = track.nPoints;
  Matrix<float,Dynamic,4> A;
  int nEqs = 2;
  A.resize(nEqs*n,4);

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

    Vector4f p1,p2,p3;
    p1 = P.row(0);
    p2 = P.row(1);
    p3 = P.row(2);

    A.row(nEqs*i + 0) = x*p3 - p1;
    A.row(nEqs*i + 1) = y*p3 - p2;
    if(nEqs == 3)
    {
      A.row(nEqs*i + 2) = x*p2 - y*p1;
    }
    i++;
  }

  JacobiSVD<MatrixXf> svd(A, ComputeThinU | ComputeThinV);
  Vector3f rhs(0.0000000001,0,0);

  MatrixXf V = svd.matrixV();
  print(V);
  VectorXf result = V.rightCols(1);
  RowVectorXf result2 = V.bottomRows(1);
  MatrixXf result3 = svd.solve(rhs);
  result3 = result3 / result3(3);

  print("");
  print(V);
  print("");


  print(result);
  print("");
  print(result2);
  print("");
  print(result3);
  print("");
  print(track.groundTruth);


}

void computeProjectionMatrix(const Matrix3f& R, const Vector3f& t, Matrix<float,3,4>& P)
{
  P << R, t;
}
