# SFM

A small library to compute 3D points given images, point correspondences and calibration parameters. This library works with the data provided by the Bundler by Noah Snavely https://github.com/snavely/bundler_sfm, but can work on any case, with the correct input of data. A test dataset can be downloaded at http://phototour.cs.washington.edu/datasets/.

### Prerequisites

Eigen as all the matrix operations are done with it.
To install eigen, open your terminal and type:

```
sudo apt install libeigen3-dev
```

### Installing

For installation, clone the repository:

```
git clone https://github.com/alejandronespereira/SFM.git
```

And build from source:

```
cmake .
make
```

Download the dataset at http://phototour.cs.washington.edu/datasets/ and decompress it. (In this example, it will be decompressed at the same folder the zip is located.

```
  unzip NotreDame.zip -d
```

Run the binary located at bin/SFM, with the right 

```
 bin/SFM [dataset Folder] [inputImages]
```
Where the dataset Folder is the folder we just extracted, the inputImages is a .txt file with the relative path (from the file) to the images. Note that for it to work with a NoahBunlder dataset, the images have to be present at the list.txt located in the folder.

We can alsoo use custom data, but we need to create our camera poses and tracks (points present in more than one image):

```C++
// Images and tracks can be created with ease:
  ImagesVec images(2);
  images[0].id = 0;
  images[1].id = 1;

// Both cameras have the same direction
  images[0].R = R;
  images[1].R = R;
  // But different location
  images[0].t = Vector3d(-1,0,0);
  images[1].t = Vector3d(1,0,0);

  // Similar intrinsics
  images[0].f = 500;
  images[1].f = 700;

  images[0].k1 = 0.01;
  images[1].k1 = 0.1;

  images[0].k2 = 0.1;
  images[1].k2 = 0.01;

  Vector2d pixelCoordsCam0(10,0);
  Vector2d pixelCoordsCam1(0,10);

  Occurrences occurrences(2);
  occurrences[0] = KeyPoint(0,pixelCoordsCam0);
  occurrences[1] = KeyPoint(1,pixelCoordsCam1);

  Track track;

  track.nPoints = 2;
  track.occurrences = occurrences;
  Tracks tracks(1);
  tracks[0] = track;
  
  // We can then feed it to a SFM class:
  SFM* sfm = new SFM()
  sfm->setTracks(tracks);
  sfm->setImages(images);
  sfm->computeSFM();
  // and save the output to a .PLY file
  sfm->writePLY("output.ply");
```

## Authors

* **Alejandro Nespereira** -


## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
