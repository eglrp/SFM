#include "math.hpp"


Vec3 triangulateTrackDLT(const Track& track, const ImagesVec& images)
{
  int nViews = track.nPoints;
  Matrix<float,Dynamic,3> A;
  Matrix<float,Dynamic,1> B;
  A.resize(2*nViews,3);
  B.resize(2*nViews,1);

  int i = 0;
  std::vector<Mat34> poses(nViews);
  Mat3X points(3, nViews);

//  points.resize(3,nViews);

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

    assert(camKey == it->id);
    if(camKey == -1)
    {
      print(it->id);
      print(it->name);
      print(it->f);
      print(camKey);
    }
    float f = it->f;
    float k1 = it->k1;
    float k2 = it->k2;
    Vector2f p(x,y);
    Vector2f q;
    undistortPoint(p,q,f,k1,k2);
    x = q(0);
    y = q(1);

    Vec3 point(x,y,1);

    Matrix<float,3,4> P;
    computeProjectionMatrix(it->R,it->t, P);

    poses[i] = P.cast<double>();

    points.col(i) = point;

    i++;
  }
  Vec4 X;
  //print(points.rows() << "x" << points.cols());
  //print(points)
  bool success = TriangulateNViewAlgebraic(points.cast<double>(),poses, &X);
  //Vec3 sol(X(0),X(1),X(2));
  Vec3 sol(X(0)/X(3),X(1)/X(3),X(2)/X(3));
  return sol;
}

void IterativeLinearLSTriangulation(const Track& track, const ImagesVec& images)
{
  int n = track.nPoints;
  Matrix<float,Dynamic,3> A;
  Matrix<float,Dynamic,1> B;
  A.resize(2*n,3);
  B.resize(2*n,1);
  std::vector<int> weights;
}

void undistortPoint(Vector2f inputPoint, Vector2f& outputPoint, float f, float k1,float k2)
{
  float x = inputPoint(0);
  float y = inputPoint(1);

  Vector2f pw(x/f, y/f);
  double scale = 1.0;

  double theta_d = sqrt(pw(0)* pw(0) + pw(1)*pw(1));

  if(theta_d > 1e-8)
  {
    double theta = theta_d;

    const double EPS = 1e-8;
    for(int i = 0; i < 10; i ++)
    {
      double theta2 = theta*theta;
      double theta4 = theta2*theta2;

      double k1_theta2 = k1 * theta2;
      double k2_theta4 = k2 * theta4;

      double theta_fix = (theta * ( 1 + k1_theta2 + k2_theta4) - theta_d)/
        (1 + 3*k1_theta2 + 5 * k2_theta4);
      theta = theta - theta_fix;
      if(fabs(theta_fix) < EPS)
      {
        break;
      }
    }
    scale = tan(theta)/theta_d;

    outputPoint = pw * scale;


  }


}

bool TriangulateNViewAlgebraic
(
  const Mat3X & points,
  const std::vector<Mat34>& poses,
  Vec4* X
)
{
  assert(poses.size() == points.cols());

  Mat4 AtA = Mat4::Zero();
  for (Mat3X::Index i = 0; i < points.cols(); ++i)
  {
    const Vec3 point_norm = points.col(i).normalized();
    const Mat34 cost =
        poses[i] -
        point_norm * point_norm.transpose() * poses[i];
    AtA += cost.transpose() * cost;
  }

  Eigen::SelfAdjointEigenSolver<Mat4> eigen_solver(AtA);
  *X = eigen_solver.eigenvectors().col(0);
  return eigen_solver.info() == Eigen::Success;
}

/*
bool TriangulateNViewAlgebraic
(
  const Matrix<float, 3, 3> & points,
  const std::vector<Matrix<float, 3, 4>>& poses,
  Vector4f* X
)
{
  assert(poses.size() == points.cols());

  Matrix4f AtA = Matrix4f::Zero();
  for(Matrix3f::Index i = 0; i < points.cols(); ++i)
  {
    const Vector3f point_norm = points.col(i).normalized();
    const Matrix<float,3,4> cost =
      poses[i] -
      point_norm * point_norm.transpose() * poses[i];
    AtA += cost.transpose() * cost;
  }
  SelfAdjointEigenSolver<Matrix4f> eigen_solver(AtA);
  *X = eigen_solver.eigenvectors().col(0);
  return eigen_solver.info() == Success;
}
*/
void computeProjectionMatrix(const Matrix3f& R, const Vector3f& t, Matrix<float,3,4>& P)
{
  P << R, t;
}
