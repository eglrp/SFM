import cv2
import numpy as np
import sys

if __name__ == "__main__":
    imagesFile = sys.argv[1]
    bundlefile = sys.argv[2]
    imageID = int(sys.argv[3])

    with open(imagesFile) as f:
        images = f.readlines()

    im = images[imageID][:-1]

    with open(bundlefile) as f:
        bundleData = f.readlines()

    camIntrinsics = bundleData[2+5*imageID]
    camIntrinsics = [float(el) for el in camIntrinsics.split(" ")]
    focal = camIntrinsics[0]
    k1 = camIntrinsics[1]
    k2 = camIntrinsics[2]

    imagePath = "/".join(imagesFile.split("/")[:-1]) + "/" + "/".join(im.split("/"))
    dist = cv2.imread(imagePath,0)
    cv2.namedWindow("Distorted",cv2.WINDOW_NORMAL)
    cv2.imshow("Distorted",dist)
    cv2.waitKey(0)

    imagePathUndistorted = imagePath.replace(".jpg",".rd.jpg")
    undistGT = cv2.imread(imagePathUndistorted,0)
    cv2.namedWindow("Undistorted",cv2.WINDOW_NORMAL)
    cv2.imshow("Undistorted",undistGT)
    cv2.waitKey(0)

    comparison = np.hstack((dist,undistGT))

    cv2.namedWindow("Comparison distorted vs undistorted",cv2.WINDOW_NORMAL)
    cv2.imshow("Comparison distorted vs undistorted",comparison)
    cv2.waitKey(0)

    difference = dist - undistGT
    cv2.namedWindow("Difference between distorted and undistorted",cv2.WINDOW_NORMAL)
    cv2.imshow("Difference between distorted and undistorted",difference)
    cv2.waitKey(0)

    # Undistort using OpenCV fisheye camera model and Noah data:
    K = np.eye(3)
    K[0,0] = focal
    K[1,1] = focal
    D = np.array([k1,k2,0,0])

    undistortedOpenCV = cv2.undistort(dist, K, distCoeffs=D,dst = None,newCameraMatrix=K.copy())

    cv2.namedWindow("OpenCV Undistortion Comparison",cv2.WINDOW_NORMAL)
    cv2.imshow("OpenCV Undistortion Comparison", np.hstack((dist,undistGT,undistortedOpenCV)))
    cv2.waitKey(0)

    # Trying with a different scale just in case
    Knew = K.copy()
    Knew[0,0] = 0.5 * focal;
    Knew[1,1] = 0.5 * focal;
    undistortedOpenCV_scaled = cv2.undistort(dist, K, distCoeffs=D,dst = None,newCameraMatrix=K.copy())

    cv2.namedWindow("OpenCV Undistortion Comparison scaled",cv2.WINDOW_NORMAL)
    cv2.imshow("OpenCV Undistortion Comparison scaled", np.hstack((dist,undistGT,undistortedOpenCV_scaled)))
    cv2.waitKey(0)

    # Let's try just in case to undistort the undistorted. Because it's not
    # entirely clear from the documentation which one is the distorted
    undistortedOpenCV_fromUndistort = cv2.undistort(undistGT, K, distCoeffs=D,dst = None,newCameraMatrix=K.copy())


    cv2.namedWindow("OpenCV Undistortion Comparison with undistort",cv2.WINDOW_NORMAL)
    cv2.imshow("OpenCV Undistortion Comparison with undistort", np.hstack((dist,undistGT,undistortedOpenCV_fromUndistort)))
    cv2.waitKey(0)

    # Just for the giggles, let's try to modify the intrinsics parameters, just
    # in case there's a units error (mm for m or something. I don't even know at
    # this point)


    for i in range(-5,5):
        power = np.power(10.,i)
        title = "focal = focal * " + str(power) + ". Distortion - Undistortion GT - openCV undistortion - difference "
        K = np.eye(3)
        K[0,0] = focal * power;
        K[1,1] = focal * power;
        undistortedOpenCV_differentF = cv2.undistort(dist, K, distCoeffs=D,dst = None,newCameraMatrix=K.copy())

        fullComparison = np.vstack((comparison, np.hstack((undistortedOpenCV_differentF,np.abs(dist - undistortedOpenCV_differentF)))))
        cv2.namedWindow(title,cv2.WINDOW_NORMAL)
        cv2.imshow(title,fullComparison)
        cv2.waitKey(0)
