#ifndef SFM_HPP
#define SFM_HPP

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <ctime>
#include <assert.h>
#include "Eigen/Core"

#include "definitions.hpp"
#include "Track.hpp"
#include "Image.hpp"
#include "math.hpp"


using namespace std;

class SFM
{
  public:
    SFM(string datasetFolder, string inputImagesFile);
    void computeSFM();
    void writePLY(string outputFile);
    void writePLYComparison(string outputFile);
    void writePLYGT(string outputFile);
    void drawCameras(string outputFile);
  private:
    bool _debugImages = true;
    bool _debugTracks = false;
    void populateImages();
    void populateImage(Image& im);

    void populateTracks();
    void populateTrack(ifstream& bundleFile);


    vector<int> _imageIDs;

    int _nImages;
    string _datasetFolder;
    string _inputImagesFile;
    string _listOfImages;
    string _bundleFile;

    ImagesVec _images;
    Tracks _tracks;

    Matrix<double,Dynamic,6> _cloudPoint;
    Matrix<double,Dynamic,6> _cloudPointGT;


};
#endif
