#include "math.hpp"

Vec3 triangulateTrackDLT(Track& track, const ImagesVec& images)
{
  int nViews = track.nPoints;
  if(nViews == 3)
  {
    track.printTrack();
  }
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

    // Get the pose:
    Matrix3d R = it->R;
    Vector3d t = it->t;
    Matrix<double,3,4> P;
    computeProjectionMatrix(R,t,P);

//    project3DPointToCamera(track.groundTruth,undistorted,R,t);
    Vector3d projectedGT = P * track.groundTruth.homogeneous();
    projectedGT = - projectedGT/projectedGT(2);
    projectedGT(2) = -projectedGT(2);
    points.col(i) = projectedGT;

    poses[i] = P;
    if(nViews == 3)
    {
      printRed(camKey << " (lines " << 3+5*camKey<< ":"<< 3+5*camKey+4<< ")")
      print(poses[i]);

      print("")
      print("Distorted: [" << distorted.transpose() << "]");
      print("f: "<< f << ", k1: " << k1 << " ,k2: " << k2)
      print("Undistorted: [" << undistorted.homogeneous().transpose()<< "]");
      print("ProjectedGT: [" << projectedGT.transpose()<< "]");
    }
  }


  Vec4 X = TriangulateNViewsNonHomogeneous(points.cast<double>(),poses);
  Vec3 sol(X(0)/X(3),X(1)/X(3),X(2)/X(3));
  track.worldPosition = sol;
  if(nViews==3) print("Solution:" << sol.transpose() << endl <<"GT:" << track.groundTruth.transpose())
  return sol;
}

void project3DPointToPixel(Vector4d& inputPoint, Vector2d& outputPoint,
  Matrix3d R, Vector3d t, double f, double k1, double k2)
{
    Matrix<double,3,4> P;
    computeProjectionMatrix(R,t,P);
    // world to image coords
    Vector3d imageCoords = P * inputPoint;
    // Perspective division
    imageCoords = - imageCoords / imageCoords(2);
    // Conversion to pixel coordinates:
    double x = imageCoords(0);
    double y = imageCoords(1);
    // Intrinsics parameters
    double p2 = (x*x + y*y);
    double p4 = p2*p2;
    double distortion = (1.0 + k1*p2 + k2*p4);
    // Final conversion, no need to add cx/cy as the reference is already centered
    outputPoint(0) = f * distortion * x;
    outputPoint(1) = f * distortion * y;
}
void project3DPointToCamera(Vector4d& inputPoint, Vector2d& outputPoint,
  Matrix3d& R, Vector3d& t)
  {
    Matrix<double,3,4> P;
    computeProjectionMatrix(R,t,P);
    // world to image coords
    Vector3d imageCoords = P * inputPoint;
    // Perspective division
    imageCoords = - imageCoords / imageCoords(2);
    outputPoint(0) = imageCoords(0);
    outputPoint(1) = imageCoords(1);
  }

void projectCameraPointToPixel(Vector2d& inputPoint, Vector2d& outputPoint,
  double f, double k1, double k2)
  {
    double x = inputPoint(0);
    double y = inputPoint(1);
    // Intrinsics parameters
    double p2 = (x*x + y*y);
    double p4 = p2*p2;
    double distortion = (1.0 + k1*p2 + k2*p4);
    // Final conversion, no need to add cx/cy as the reference is already centered
    outputPoint(0) = f * distortion * x;
    outputPoint(1) = f * distortion * y;
  }

void undistortPoint(Vector2d inputPoint, Vector2d& outputPoint,double cx,double cy, double f, double k1,double k2)
{
  // Code "borrowed" from OpenCV library. No k3 or k4.
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


Vec4 TriangulateNViewAlgebraic
(
  const Mat3X & points,
  const std::vector<Mat34>& poses
)
{

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
  return eigen_solver.eigenvectors().col(0);
}

Vec4 TriangulateNViewsNonHomogeneous
(
  const Mat3X & points,
  const std::vector<Mat34>& poses
)
{
  assert(poses.size() == points.cols());

  Matrix<double,Dynamic,3> A;
  Matrix<double,Dynamic,1> B;

  A.resize(2*poses.size(),3);
  B.resize(2*poses.size(),1);

  for(int i = 0; i < points.cols(); i++)
  {
    const Vec3 point = points.col(i);
    double x = point(0);
    double y = point(1);
    double w = 1;

    const Matrix<double,3,3> R = poses[i].block<3,3>(0,0);
    const Matrix<double,3,1> t = poses[i].col(3);

    RowVector3d p0 = R.row(0);
    RowVector3d p1 = R.row(1);
    RowVector3d p2 = R.row(2);

    A.row(2*i + 0) = x * p2 - p0;
    A.row(2*i + 1) = y * p2 - p1;
    B(2*i +0) = x * t(2) - t(0);
    B(2*i +1) = y * t(2) - t(1);

  }
  if(poses.size()==3)
  {
    printRed("Points:")
    print(points)
    Matrix<double,Dynamic,4> C;
    C.resize(A.rows(),4);
    C << A,B;
    printRed("A | B:")
    print(C)
    print("A: "<< A.rows() << "x"<< A.cols())
    print("B: "<< B.rows() << "x"<< B.cols())
    print("C: "<< C.rows() << "x"<< C.cols())
  }
  Vector3d triangulation =  A.jacobiSvd(ComputeThinU | ComputeThinV).solve(B);
  return triangulation.homogeneous();
}

double calculateReprojectionError(Track& track, ImagesVec& images)
{
  double error = 0;
  for(int iTrack = 0; iTrack < track.occurrences.size(); iTrack++)
  {
    KeyPoint kp = track.occurrences[iTrack];
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
    // Get the pose:
    Matrix3d R = it->R;
    Vector3d t = it->t;
    Matrix<double,3,4> P;
    computeProjectionMatrix(R,t,P);
    //Vector4d X = track.worldPosition.homogeneous();
    Vector4d X = track.worldPosition.homogeneous();
    Vector2d pixelCoords;
    project3DPointToPixel(X,pixelCoords,R,t,f,k1,k2);

    double dx = distorted(0) - pixelCoords(0);
    double dy = distorted(1) - pixelCoords(1);

    error += sqrt(dx*dx + dy*dy);
  }
  return error/track.nPoints;
}

void computeProjectionMatrix(const Matrix3d& R, const Vector3d& t, Matrix<double,3,4>& P)
{
  P << R, t;
}
