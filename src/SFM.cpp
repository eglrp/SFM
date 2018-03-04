#include "SFM.hpp"

SFM::SFM(string datasetFolder, string inputImagesFile)
: _datasetFolder(datasetFolder), _inputImagesFile(inputImagesFile)
{

  //TODO: make a regex here to get the bundle.out file (not always NotreDame).
  // In other words, unhardcode this. list.txt is fine, it's always the same name
  if( datasetFolder.back() == '/')
  {
    _listOfImages = datasetFolder + "list.txt";
    _bundleFile = datasetFolder + "notredame.out";
  }
  else
  {
    _listOfImages = datasetFolder + "/list.txt";
    _bundleFile = datasetFolder + "/notredame.out";
  }
  print("Using images from " << _inputImagesFile);
  print("Reading bundle data from " << _bundleFile);
  print("Reading images from " << _listOfImages);
  print("");

  clock_t begin,end;
  begin = clock();
  print("Populating images");
  populateImages();
  end = clock();
  print(_images.size() << " images created. Elapsed time: " << double(end-begin)/CLOCKS_PER_SEC * 1000 << " ms.");

  begin = clock();
  print("Populating tracks");
  populateTracks();
  end = clock();
  print(_tracks.size() << " tracks created. Elapsed time: " << double(end-begin)/CLOCKS_PER_SEC * 1000 << " ms.");
}

void SFM::populateImages()
{
  vector< string > allImages, inputImages;

  string line;
  //TODO: Create function to do this twice.

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


  for(auto inputImage : inputImages)
  {
    int id = 0;
    for(auto el : allImages)
    {
      if( inputImage == el)
      {
        Image im;
        im.id = id;
        populateImage(id,im);
        im.name = inputImage;
        _images.push_back(im);
        _imageIDs.push_back(id);
      }
      id++;
    }
  }

}

void SFM::populateImage(int id, Image& im)
{
  int startingLine = 5 * id + 2; //5 lines per camera + 2 from the header
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
      populateTrack(bundleFile);
    }
  }
}

void SFM::populateTrack(ifstream& openFile)
{

  Eigen::Vector3f groundTruth;
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
    float x, y;
    pos >> cameraKey >> kpKey >> x >> y;
    if(find(_imageIDs.begin(),_imageIDs.end(),cameraKey) != _imageIDs.end())
    {
      KeyPoint kp;
      kp.first  = cameraKey;
      kp.second = Eigen::Vector2f(x,y);
      occurrences.push_back(kp);
    }
  }
  // Point has to be at at least 2 images to be projected.
  if(occurrences.size() > 1)
  {
    Track track;
    track.id = 0;
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
  if(_tracks.size() < 2 || _images.size() < 2)
  {
    print("Error, insufficient images to perform SFM. Aborting.");
    return;
  }

  triangulateTrackDLT(_tracks[0], _images);

}
