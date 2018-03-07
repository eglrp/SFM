#include "SFM.hpp"

SFM::SFM()
{

}

SFM::SFM(string datasetFolder, string inputImagesFile, string bundleFile)
: _datasetFolder(datasetFolder),
 _inputImagesFile(inputImagesFile),
 _bundleFile(bundleFile)
{
  //TODO: make a regex here to get the bundle.out file (not always NotreDame).
  // In other words, unhardcode this. list.txt is fine, it's always the same name
  if( datasetFolder.back() == '/')
  {
    _listOfImages = datasetFolder + "list.txt";
    //_bundleFile = datasetFolder + "notredame.out";
  }
  else
  {
    _listOfImages = datasetFolder + "/list.txt";
    //_bundleFile = datasetFolder + "/notredame.out";
  }
  print("Using images from " << _inputImagesFile);
  print("Reading bundle data from " << _bundleFile);
  print("Reading images from " << _listOfImages);
  print("");

  clock_t begin,end;
  begin = clock();
  print("Populating cameras");
  populateCameras();
  end = clock();
  print(_cameras.size() << " cameras created. Elapsed time: " << double(end-begin)/CLOCKS_PER_SEC << " s.");

  begin = clock();
  print("Populating tracks");
  populateTracks();
  end = clock();
  print(_tracks.size() << " tracks created. Elapsed time: " << double(end-begin)/CLOCKS_PER_SEC * 1000 << " ms.");
}
CamerasVec SFM::getCameras()
{
  return _cameras;
}
Tracks SFM::getTracks()
{
  return _tracks;
}

void SFM::setCloudPoint(Matrix<double,Dynamic,6> cloudPoint)
{
  _cloudPoint = cloudPoint;
}

void SFM::populateCameras()
{

  vector< string > allImages, inputImages;

  string line;

  ifstream inputImagesFile (_inputImagesFile.c_str());
  if( inputImagesFile.is_open())
  {
    while(getline (inputImagesFile,line))
    {
      inputImages.push_back(line);
    }
    inputImagesFile.close();
  }
  else
  {
    print("Could not open file " + _inputImagesFile);
  }

  ifstream listOfImages (_listOfImages.c_str());
  if( listOfImages.is_open())
  {
    while(getline (listOfImages,line))
    {
      allImages.push_back(line);
    }
    listOfImages.close();
  }
  else
  {
    print("Could not open file " + _listOfImages);
  }


  for(int iInputImage = 0; iInputImage < inputImages.size(); iInputImage++)
  {
    for(int iAllImages = 0; iAllImages < allImages.size(); iAllImages++)
    {
      if( inputImages[iInputImage] == allImages[iAllImages])
      {
        Camera cam;
        cam.id = iAllImages;
        cam.name = inputImages[iInputImage];
        populateCamera(cam);
        _cameras.push_back(cam);
        _cameraIDs.push_back(iAllImages);
      }
    }
  }
}

void SFM::populateCamera(Camera& im)
{
  cv::Mat image = cv::imread(_datasetFolder +  im.name);
  im.w = image.cols;
  im.h = image.rows;

  int startingLine = 5 * im.id + 2; //5 lines per camera + 2 from the header
  ifstream bundleFile (_bundleFile.c_str());
  string line;
  if(bundleFile.is_open())
  {
    for (int lineNumber = 0; lineNumber < startingLine; lineNumber++)
    {
      getline(bundleFile,line);
    }

    // Read f,k1,k2
    getline(bundleFile,line);

    istringstream k,r0,r1,r2,t;

    k.str(line);
    k >> im.f >> im.k1 >> im.k2;
    // Read R
    getline(bundleFile,line);
    r0.str(line);
    r0 >> im.R(0,0) >> im.R(0,1) >> im.R(0,2);

    getline(bundleFile,line);
    r1.str(line);
    r1 >> im.R(1,0) >> im.R(1,1) >> im.R(1,2);

    getline(bundleFile,line);
    r2.str(line);
    r2 >> im.R(2,0) >> im.R(2,1) >> im.R(2,2);

    // Read t
    getline(bundleFile,line);
    t.str(line);
    t >> im.t(0) >> im.t(1) >> im.t(2);

    bundleFile.close();
  }
  else
  {
    print("Could not open file " + _bundleFile);
  }

  if(_debugCameras)
  {
    print("Camera " << im.name << " read. (" << im.id <<")" )
    print("lines from " << 3 + 5*im.id << " to " << 7 + 5*im.id );
    print("f,k1,k2: " << im.f << "," << im.k1 << "," << im.k2);
    print("R:");
    print(im.R);
    print("t: " << im.t.transpose());
  }

}

void SFM::populateTracks()
{
  ifstream bundleFile (_bundleFile.c_str());
  string line;
  if(bundleFile.is_open())
  {
    int nCameras, nPoints;
    //Read the header
    getline(bundleFile,line);
    //Read the number of cameras & points
    getline(bundleFile,line);
    istringstream in;

    in.str(line);
    in >> nCameras >> nPoints;
    int startingLine = 5 * nCameras; //5 lines per camera
    // Read the bundle until the points.
    for(int lineNumber = 0; lineNumber < startingLine; lineNumber++)
    {
      getline(bundleFile,line);
    }
    //First point:
    for(int iPoint = 0; iPoint < nPoints; iPoint++)
    {
      populateTrack(bundleFile,iPoint);
      if(_debugTracks)
      {
        _tracks.back().printTrack();
      }
    }

    bundleFile.close();
  }
  else
  {
    print("Could not open " + _bundleFile);
  }
}

void SFM::populateTrack(ifstream& openFile, int id)
{
  Eigen::Vector3d groundTruth;
  Eigen::Vector3i color;
  Occurrences occurrences;

  string line;
  istringstream gt,col,pos;
  getline(openFile,line);
  gt.str(line);
  gt >> groundTruth(0) >> groundTruth(1) >> groundTruth(2);

  getline(openFile,line);
  col.str(line);
  col >> color(0) >> color(1) >> color(2);

  getline(openFile,line);
  pos.str(line);
  int nPoints;

  pos >> nPoints;
  for(int iPoint = 0; iPoint < nPoints; iPoint++)
  {
    int cameraKey, kpKey;
    double x, y;
    pos >> cameraKey >> kpKey >> x >> y;
    if(find(_cameraIDs.begin(),_cameraIDs.end(),cameraKey) != _cameraIDs.end())
    {
      KeyPoint kp;
      kp.first  = cameraKey;
      kp.second = Eigen::Vector2d(x,y);
      occurrences.push_back(kp);
    }
  }
  // Point has to be at at least 2 images to be projected.
  if(occurrences.size() > 1)
  {
    Track track;
    track.id = id;
    track.groundTruth = groundTruth;
    track.color = color;
    track.occurrences = occurrences;
    track.nPoints = occurrences.size();
    _tracks.push_back(track);
  }
  return;
}

void SFM::computeSFM()
{
  assert(_tracks.size() < 1 || _cameras.size() < 2);

  _cloudPoint.resize(_tracks.size(),6);
  _cloudPointGT.resize(_tracks.size(),6);
  clock_t begin,end;
  begin = clock();

  for(int i = 0; i < _tracks.size(); i++)
  {
    Vector3d X = triangulateTrackDLT(_tracks[i], _cameras);
    if(_tracks[i].nPoints == 3) break;
    Vector3d color = _tracks[i].color.cast<double>();
    Vector3d GT = _tracks[i].groundTruth;
    _cloudPoint(i,0) = X(0);
    _cloudPoint(i,1) = X(1);
    _cloudPoint(i,2) = X(2);
    _cloudPoint(i,3) = color(0);
    _cloudPoint(i,4) = color(1);
    _cloudPoint(i,5) = color(2);

    _cloudPointGT(i,0) = GT(0);
    _cloudPointGT(i,1) = GT(1);
    _cloudPointGT(i,2) = GT(2);
    _cloudPointGT(i,3) = color(0);
    _cloudPointGT(i,4) = color(1);
    _cloudPointGT(i,5) = color(2);
  }
  end = clock();
  print("Reconstruction done. Elapsed time: " << double(end-begin)/CLOCKS_PER_SEC * 1000 << " ms.");
}


void SFM::writePLY(string outputFile)
{
  ofstream myfile (outputFile);
  if(myfile.is_open())
  {
    // Write header:
    myfile << "ply\n";
    myfile << "format ascii 1.0\n";
    myfile << "element vertex " << _cloudPoint.rows() << "\n";
    myfile << "property float32 x\n";
    myfile << "property float32 y\n";
    myfile << "property float32 z\n";
    myfile << "property uchar red\n";
    myfile << "property uchar green\n";
    myfile << "property uchar blue\n";
    myfile << "end_header\n";
    for(int i = 0; i < _cloudPoint.rows();i++)
    {
      myfile << _cloudPoint(i,0) << " " << _cloudPoint(i,1) << " " << _cloudPoint(i,2) << " ";
       myfile << _cloudPoint(i,3) << " " << _cloudPoint(i,4) << " " << _cloudPoint(i,5) << "\n";
    }
    myfile.close();
    print("Cloudpoint saved to " + outputFile);
  }
  else
  {
    print("Unable to open file " + outputFile);
  }

}
void SFM::writePLYGT(string outputFile)
{
  ofstream myfile (outputFile);
  if(myfile.is_open())
  {
    // Write header:
    myfile << "ply\n";
    myfile << "format ascii 1.0\n";
    myfile << "element vertex " << _cloudPointGT.rows() << "\n";
    myfile << "property float32 x\n";
    myfile << "property float32 y\n";
    myfile << "property float32 z\n";
    myfile << "property uchar red\n";
    myfile << "property uchar green\n";
    myfile << "property uchar blue\n";
    myfile << "end_header\n";
    for(int i = 0; i < _cloudPoint.rows();i++)
    {
      myfile << _cloudPointGT(i,0) << " " << _cloudPointGT(i,1) << " " << _cloudPointGT(i,2) << " ";
       myfile << _cloudPointGT(i,3) << " " << _cloudPointGT(i,4) << " " << _cloudPointGT(i,5) << "\n";
    }
    myfile.close();
    print("Cloudpoint saved to " + outputFile);
  }
  else
  {
    print("Unable to open file " + outputFile);
  }
  /*
  for(int iTrack = 0; iTrack < _tracks.size(); iTrack++)
  {
    _tracks[iTrack].worldPosition = _tracks[iTrack].groundTruth;
  }
  writePLY(outputFile);
  */
}

void SFM::drawCameras(string outputFile)
{
  ofstream myfile (outputFile);
  if(myfile.is_open())
  {
    // Write header:
    myfile << "ply\n";
    myfile << "format ascii 1.0\n";
    myfile << "element vertex " << _cameras.size() << "\n";
    myfile << "property float32 x\n";
    myfile << "property float32 y\n";
    myfile << "property float32 z\n";
    myfile << "property uchar red\n";
    myfile << "property uchar green\n";
    myfile << "property uchar blue\n";
    myfile << "end_header\n";
    for(int i = 0; i < _cameras.size();i++)
    {
      Vector3d cameraPosition = - _cameras[i].R.transpose() * _cameras[i].t;

      myfile << cameraPosition(0) << " " << cameraPosition(1) << " " << cameraPosition(2) << " ";
       myfile << 255 << " " << 0 << " " << 0 <<  "\n";
    }
    myfile.close();
    print("Cloudpoint saved to " + outputFile);
  }
  else
  {
    print("Unable to open file " + outputFile);
  }
}

double SFM::reprojectionError()
{

    double error = 0;
    for(int i = 0; i < _tracks.size(); i++)
    {
      error += calculateReprojectionError(_tracks[i], _cameras);
    }
    return error/_tracks.size();
}

double SFM::GTError()
{
  for(int i = 0; i < _tracks.size(); i++)
  {
    _tracks[i].worldPosition = _tracks[i].groundTruth;
  }
  return reprojectionError();
}

void SFM::setTracks(Tracks tracks)
{
  _tracks = tracks;
}

void SFM::setCameras(CamerasVec cameras)
{
  _cameras = cameras;
}
