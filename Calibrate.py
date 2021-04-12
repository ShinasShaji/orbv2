import glob
import json
import os
import fnmatch

import cv2
import numpy as np


class Calibrate:
    """Class for camera calibration"""
    def __init__(self, chessBoardSize, squareSize, leftCam = True, path = "captures/calibrate"):
        # Termination criteria for distortion calibration
        self.cornerSubPixCriteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.stereoCriteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # Chessboard properties
        self.chessBoardSize = chessBoardSize
        self.squareSize = squareSize # mm

        # Preparing object points like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0), multiplied by square size
        self.objectPointPrep = np.zeros((self.chessBoardSize[0]*self.chessBoardSize[1],3), np.float32)
        self.objectPointPrep[:,:2] = \
            np.multiply(np.mgrid[0:self.chessBoardSize[0],0:self.chessBoardSize[1]].T.reshape(-1,2), self.squareSize)

        # Arrays to store object points and image points from all images
        self.objectPoints = [] # 3D point in real world space
        self.imagePointsL = [] # 2D points in image plane, left
        self.imagePointsR = [] # right

        # Pull up checkerboard images
        self.path = path
        imageFormat = ".png"
        self.imagesL = sorted(glob.glob("".join([self.path, "/left_*", imageFormat])))
        self.imagesR = sorted(glob.glob("".join([self.path, "/right_", imageFormat])))

        # Flags
        self.calibrated = False
        self.leftCam = leftCam


    def findCorners(self):
        """Find chessboard corners on images from folder"""
        self.calibrated = False

        # Counting number of images in directory
        # Ensure that only required and matching images are stored here
        imageCount = len(fnmatch.filter(os.listdir(self.path), '*.png'))
        print("".join(["Found ", str(imageCount), " images in folder"]))

        for (fileNameL, fileNameR) in (self.imagesL, self.imagesR):
            # Reading left and right images
            imageL = cv2.imread(fileNameL)
            imageR = cv2.imread(fileNameR)

            # Converting to grayscale
            self.grayImageL = cv2.cvtColor(imageL, cv2.COLOR_BGR2GRAY)
            self.grayImageR = cv2.cvtColor(imageR, cv2.COLOR_BGR2GRAY)

            # Finding chess board corners
            retValL, cornersL = cv2.findChessboardCorners(self.grayImageL, self.chessBoardSize, None)
            retValR, cornersR = cv2.findChessboardCorners(self.grayImageR, self.chessBoardSize, None)

            # If found, add object points, image points (after refining them)
            if retValL and retValR:
                self.objectPoints.append(self.objectPointPrep)

                cornersL = cv2.cornerSubPix(self.grayImageL, cornersL, (11,11), (-1,-1), self.cornerSubPixCriteria)
                cornersR = cv2.cornerSubPix(self.grayImageR, cornersR, (11,11), (-1,-1), self.cornerSubPixCriteria)
                
                self.imagePointsL.append(cornersL)
                self.imagePointsR.append(cornersR)

                # Draw and display the corners
                cv2.drawChessboardCorners(imageL, self.chessBoardSize, cornersL, retValL)
                cv2.drawChessboardCorners(imageR, self.chessBoardSize, cornersR, retValR)

                cv2.imshow('Chessboard_Left', imageL)
                cv2.imshow('Chessboard_Right', imageR)

                cv2.waitKey(500)

        cv2.destroyAllWindows()
        

    def calibrate(self):
        """Calculate camera matrix, distortion coefficients and rotation and translation vectors"""
        # Finding camera matrices, distortion coefficients, rotation and
        # translation vectors
        # Left
        retValL, self.cameraMatrixL, self.distortionCoeffsL, \
            self.rotationVecsL, self.translationVecsL = \
            cv2.calibrateCamera(self.objectPoints, self.imagePointsL, \
                self.grayImageL.shape[::-1], None, None)
        # Right
        retValR, self.cameraMatrixR, self.distortionCoeffsR, \
            self.rotationVecsR, self.translationVecsR = \
            cv2.calibrateCamera(self.objectPoints, self.imagePointsR, \
                self.grayImageR.shape[::-1], None, None)

        # Finding optimal camera matrices and regions of interest
        # Left
        heightL, widthL = self.grayImageL.shape[:2]
        self.optimalCameraMatrixL, self.roiL = \
            cv2.getOptimalNewCameraMatrix(self.cameraMatrixL,\
                self.distortionCoeffsL, (widthL, heightL), 1,\
                (widthL, heightL))
        # Right
        heightR, widthR = self.grayImageR.shape[:2]
        self.optimalCameraMatrixR, self.roiR = \
            cv2.getOptimalNewCameraMatrix(self.cameraMatrixR,\
                self.distortionCoeffsR, (widthR, heightR), 1,\
                (widthR, heightR))

        if retValL and retValR:
            self.calibrated = True
            print("Calibration complete")

        else:
            self.calibrated = False
            print("Calibration failed")

    
    def stereoCalibrate():
        """Calibrate camera pair with respect to each other"""
        # Flags for calibration
        # Uncomment flags to use
        flags = 0
        flags |= cv2.CALIB_FIX_INTRINSIC
        #flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
        #flags |= cv2.CALIB_USE_INTRINSIC_GUESS
        #flags |= cv2.CALIB_FIX_FOCAL_LENGTH
        #flags |= cv2.CALIB_FIX_ASPECT_RATIO
        #flags |= cv2.CALIB_ZERO_TANGENT_DIST
        #flags |= cv2.CALIB_RATIONAL_MODEL
        #flags |= cv2.CALIB_SAME_FOCAL_LENGTH
        #flags |= cv2.CALIB_FIX_K3
        #flags |= cv2.CALIB_FIX_K4
        #flags |= cv2.CALIB_FIX_K5


### To do: Modify everything below to accomodate two cameras ###

    def printResults(self):
        """Print calibration results"""
        if self.calibrated:
            with np.printoptions(precision=3, suppress=True):
                print("Camera matrix:\n", self.cameraMatrixL, "\n")
                print("Distortion coefficients:\n", self.distortionCoeffsL, "\n")
                print("Rotation vectors:\n", self.rotationVecsL, "\n")
                print("Translation vectors:\n", self.translationVecsL, "\n")
        else:
            print("Not calibrated yet")


    def exportResults(self, path="data/calibration.json"):
        """Export results as json for later retrieval"""
        results = {"cameraMatrix" : self.cameraMatrixL.tolist(), \
                   "distortionCoefficients" : self.distortionCoeffsL.tolist()}

        if self.leftCam:
            path = "".join([path.replace(".json", ""), "_left.json"])
        else:
            path = "".join([path.replace(".json", ""), "_right.json"])

        resultJson = open(path, "w")
        json.dump(results, resultJson, sort_keys=True, indent=4)
        resultJson.close()

        print("Exported as", path)
        

    def undistortImages(self):
        if self.calibrated:
            for fileName in self.images:
                image = cv2.imread(fileName)

                undistortedImage = cv2.undistort(image, self.cameraMatrix, self.distortionCoeffs, None, self.cameraMatrix)

                cv2.imshow('UndistortedImage', undistortedImage)
                cv2.waitKey(500)

            cv2.destroyAllWindows()
        
        else:
            print("Camera hasn't been calibrated yet")


    def findReprojectionError(self):
        mean_error = 0
        for i in range(len(self.objectPoints)):
            imgpoints2L, _ = cv2.projectPoints(self.objectPoints[i], \
                self.rotationVecsL[i], self.translationVecsL[i], \
                self.cameraMatrixL, self.distortionCoeffsL)
            error = cv2.norm(self.imagePointsL[i], imgpoints2L, cv2.NORM_L2)/len(imgpoints2L)
            mean_error += error
        print( "Total error: {}".format(mean_error/len(self.objectPoints)) )



if __name__=="__main__":
    print("Run using calibrateCamera.ipynb")