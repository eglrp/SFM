import numpy as np
import sys


if __name__ == "__main__":
    bundleFile =  sys.argv[1]

    with open(bundleFile,"r") as f:
        data = f.readlines()

    line = data[1]
    line = line.split(" ")
    nCameras = int(line[0])
    nPoints = int(line[1])

    camerasRange =  range(6, 5 *(1 + nCameras),5)
    cameras =  [data[el] for el in camerasRange ]
    camerasColor = [255,0,0]

    #for i,el in enumerate(cameras):
    #    cameras[i] = [ float(el) for el in  cameras[i].split(" ")]

    pointsRangePosition = range(2 +5*nCameras,2 +5*nCameras +3 * nPoints, 3)
    pointsPosition = [data[el] for el in pointsRangePosition ]

    #for i,el in enumerate(pointsPosition):
    #    pointsPosition[i] = [float(el) for el in pointsPosition[i].split(" ")]

    pointsColor = [data[el + 1] for el in pointsRangePosition ]
    #for i,el in enumerate(pointsColor):
    #    pointsPosition[i] = [int(el) for el in pointsColor[i].split(" ")]


    with open("camerasGT.ply","w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write("element vertex " + str(nCameras + nPoints) + "\n")
        f.write("property float32 x\n")
        f.write("property float32 y\n")
        f.write("property float32 z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        for cam in cameras:
            f.write(cam[:-1] + " 255 0 0\n")
        for pos,color in zip(pointsPosition,pointsColor):
            f.write(pos[:-1] + " " + color)
