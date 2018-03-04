#ifndef SFM_HPP
#define SFM_HPP

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <ctime>
#include "Eigen/Core"

#include "definitions.hpp"
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

  private:

    void populateImages();
    void populateImage(int id, Image& im);

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

    Matrix<float,Dynamic,6> _cloudPoint;
    Matrix<float,Dynamic,6> _cloudPointGT;


};
#endif
