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
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "definitions.hpp"
#include "Track.hpp"
#include "Camera.hpp"
#include "math.hpp"


using namespace std;

class SFM
{
  public:

    //SFM pipeline constructor for custom points/cameras/images/correspondences
    SFM();
    // SFM pipeline constructor for Noah Bundler datasets.
    // Input parameters:
    // datasetFolder: the folder containing the bundle.out file and the list of
    // images
    // inputImagesFile: file with the list of images, one line per image
    // bundleFile: the file outputed by the Noah Bundler.
    SFM(string datasetFolder, string inputImagesFile, string bundleFile);

    // Compute SFM from the input given by the class initialization
    void computeSFM();

    // Write the reconstructed structure into the file
    void writePLY(string outputFile);
    // Write the groundTruth structure into the file
    void writePLYGT(string outputFile);
    // Write just the cameras into the file
    void drawCameras(string outputFile);
    // Calculate the reprojection error for the structure
    double reprojectionError();
    // Calculate the reprojection error for the Noah Bundler data
    double GTError();
    // Type of triangulation:
    // 1: non homogeneous
    // 0: Algebraic (from OpenMVG)

    void setCameras(CamerasVec cameras);

    CamerasVec getCameras();

    void setTracks(Tracks tracks);

    Tracks getTracks();

    void setCloudPoint(Matrix<double,Dynamic,6> cloudPoint);

    void projectCamera(string outputFile, int cameraID);
  private:
    // Enable to verbose the Cameras and Tracks creation
    bool _debugCameras = false;
    bool _debugTracks = false;
    // Creates the images vector.
    void populateCameras();
    // Populates a Camera from the Noah Bundler camera info. The Camera needs a
    // valid ID beforehand.
    void populateCamera(Camera& cam);

    // Creates the tracks vector
    void populateTracks();
    // Populates a track from the Noah Bundler point info
    void populateTrack(ifstream& bundleFile, int id);


    vector<int> _cameraIDs;

    int _nCameras;
    string _datasetFolder;
    string _inputImagesFile;
    string _listOfImages;
    string _bundleFile;

    CamerasVec _cameras;
    Tracks _tracks;

    Matrix<double,Dynamic,6> _cloudPoint;
    Matrix<double,Dynamic,6> _cloudPointGT;


};
#endif
