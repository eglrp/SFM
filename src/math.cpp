#include "math.hpp"

template<typename matrix_t, typename vector_t>
void solveNullspaceLU(const matrix_t& A, vector_t& x){
    x = A.fullPivLu().kernel();
    x.normalize();
}

template<typename matrix_t, typename vector_t>
void solveNullspaceQR(const matrix_t& A, vector_t& x){
    auto qr = A.transpose().colPivHouseholderQr();
    matrix_t Q = qr.householderQ();
    x = Q.col(A.rows() - 1);
    x.normalize();
}
template<typename matrix_t, typename vector_t>
void solveNullspaceSVD(const matrix_t& A, vector_t& x){
    x = A.jacobiSvd(Eigen::ComputeFullV).matrixV().col( A.rows() - 1 );
    x.normalize();
}

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

    RowVector4f p1,p2,p3;
    p1 = P.row(0);
    p2 = P.row(1);
    p3 = P.row(2);

    p1 = p1.normalized();
    p2 = p2.normalized();
    p3 = p3.normalized();

    A.row(nEqs*i + 0) = x*p3 - p1;
    A.row(nEqs*i + 1) = y*p3 - p2;
    if(nEqs == 3)
    {
      A.row(nEqs*i + 2) = x*p2 - y*p1;
    }
    i++;

  }

  VectorXf resSVD,resQR,resLU;
  solveNullspaceSVD(A,resSVD);
  solveNullspaceQR(A,resQR);
  solveNullspaceLU(A,resLU);

  print(resSVD.normalized());
  print(resQR.normalized());
  print(resLU.normalized());

  print(track.groundTruth.transpose() );


}



void computeProjectionMatrix(const Matrix3f& R, const Vector3f& t, Matrix<float,3,4>& P)
{
  P << R, t;
}
