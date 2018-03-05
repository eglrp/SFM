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

## Authors

* **Alejandro Nespereira** -


## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
