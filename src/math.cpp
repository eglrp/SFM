#include "math.hpp"


Vec3 triangulateTrackDLT(const Track& track, const ImagesVec& images)
{
  int nViews = track.nPoints;

  std::vector<Mat34> poses(nViews);
  Mat3X points(3, nViews);

  for(int i = 0; i < nViews; i++)
  {
    KeyPoint kp = track.occurrences[i];
    int camKey = kp.first;
    Vector2d distorted = kp.second;

    // Look for image:
    // predicate to find the right camera
    auto pred = [camKey](const Image& im)
    {
      return (im.id == camKey);
    };
    auto it = find_if(begin(images),end(images),pred);

    assert(camKey == it->id);

    // Get R,t,and K:
    double f,k1,k2;
    f = it->f;
    k1 = it->k1;
    k2 = it->k2;

    // Undistort the points: pixel -> image coords
    double cx = 0,cy = 0; // Pixel positions are already counted from center
    Vector2d undistorted;
    undistortPoint(distorted, undistorted,cx,cy,f,k1,k2);
    points.col(i) = undistorted.homogeneous();

    // Get the pose:
    Matrix3d R = it->R;
    Vector3d t = it->t;
    Matrix<double,3,4> P;
    computeProjectionMatrix(R,t,P);
    poses[i] = P;
  }
  Vec4 X;
  bool success = TriangulateNViewAlgebraic(points.cast<double>(),poses, &X);
  if(success)
  {
  Vec3 sol(X(0)/X(3),X(1)/X(3),X(2)/X(3));
  return sol;
  }
  else
  {
      return Vec3(X(0),X(1),X(2));
  }
}

void project3DPointToPixel(const Vector4d& inputPoint, Vector2d& outputPoint,
  Matrix3d R, Vector3d t, double f, double k1, double k2)
{
    Matrix<double,3,4> P;
    computeProjectionMatrix(R,t,P);
    // world to image coords
    Vector3d imageCoords = P * inputPoint;
    print(imageCoords.transpose());
    // Perspective division
    imageCoords = -imageCoords / imageCoords(2);
    print(imageCoords.transpose());
    // Conversion to pixel coordinates:
    double x = imageCoords(0);
    double y = imageCoords(1);
    double p2 = (x*x + y*y);
    double p4 = p2*p2;
    print(p2);
    print(p4);
    double distortion = (1 + k1*p2 + k2*p4);
    print(distortion);
    outputPoint(0) = f * distortion * x;
    outputPoint(1) = f * distortion * y;
}

void undistortPoint(Vector2d inputPoint, Vector2d& outputPoint,double cx,double cy, double f, double k1,double k2)
{
  float x = inputPoint(0);
  float y = inputPoint(1);

  Vector2d pw(x/f, y/f);
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
    scale = std:: tan(theta)/theta_d;
    outputPoint = pw * scale;
  }
  else
  {
    print("no distortion??");
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
void computeProjectionMatrix(const Matrix3d& R, const Vector3d& t, Matrix<double,3,4>& P)
{
  P << R, t;
}
