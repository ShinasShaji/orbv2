import json
import os
import time
import threading

import cv2
import numpy as np


class StereoCam:
    """Class to handle a camera pair as a synchronized stereo pair"""
    def __init__(self, leftID, rightID, width = 1280, height = 720, fps=2):
        self.leftCam = cv2.VideoCapture(leftID)
        self.rightCam = cv2.VideoCapture(rightID)

        # Set camera resolution
        self.leftCam.set(3, width)
        self.leftCam.set(4, height)
        self.rightCam.set(3, width)
        self.rightCam.set(4, height)

        # Set format to MJPG in FourCC format 
        self.leftCam.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
        self.rightCam.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))

        # Timing
        self.previousTime = time.time()
        self.frameTime = float(1/fps)

        # Capture details
        self.imgCounter = 0
        self.capturePath = "captures/" 

        # Disparity
        self.numDisparities = 6*16
        self.blockSize = 11
    
        self.minDisparity = 0
        self.windowSize = 6


    def checkOpen(self):
        """Check if cameras have initialized"""
        if self.leftCam.isOpened() and self.rightCam.isOpened():
            return True
        else:
            return False


    def getFrames(self):
        """Grab and retrieve frames"""
        for i in range(10):
            self.leftCam.grab()
            self.rightCam.grab()

        retLeft, self.leftImg = self.leftCam.retrieve()
        retRight, self.rightImg = self.rightCam.retrieve()

        if retLeft and retRight:
            return True
        else:
            return False


    def captureImages(self):
        """Capture and save images from both cameras"""
        for (cam, frame) in [("left", self.leftImg), ("right", self.rightImg)]:
            imgName = "".join([cam, "_{}.png".format(self.imgCounter)])
            cv2.imwrite(os.path.join(self.capturePath, imgName), frame)
            print("Captured {}".format(imgName))

        self.imgCounter += 1

    
    def camPreview_Internal(self):
        """Preview the camera outputs"""
        print("Initializing cameras for preview")
        if self.checkOpen():
            retVal = self.getFrames()

            while retVal:
                if ((time.time()-self.previousTime)>=self.frameTime):
                    self.previousTime = time.time()

                    retVal = self.getFrames()

                    cv2.imshow('Left', self.leftImg)
                    cv2.imshow('Right', self.rightImg)

                    key = cv2.waitKey(20)
                    if key == 27: # Exit on ESC
                        break
                    if key == 32: # Capture on SPACE
                        self.captureImages()

            cv2.destroyAllWindows()
            
        else:
            print('Could not initialize camera pair')

    
    def camPreview(self):
        previewThread = threading.Thread(target=self.camPreview_Internal)
        previewThread.start()


    def loadCalibration(self, path="data/calibration.json"):
        """Load calibration matrices from jsons"""
        path = {"left":"".join([path.replace(".json", ""), "_left.json"]), \
                "right":"".join([path.replace(".json", ""), "_right.json"])}

        try:
            # Index 0 is left, 1 is right
            self.cameraMatrix = []
            self.distortionMatrix = []

            for camera in ["left", "right"]:
                with open(path[camera], "r") as matrixJson:
                    calibrationDict = json.load(matrixJson)

                    self.cameraMatrix.append(np.array(calibrationDict["cameraMatrix"]))
                    self.distortionMatrix.append(np.array(calibrationDict["distortionCoefficients"]))

            print("Calibration loaded")

        except:
            print("Could not load calibration")

    
    def previewDisparity(self):
        self.createStereoSGBM()
        self.createStereoBM()

        if self.checkOpen():
            retVal = self.getFrames()

            while retVal:
                if ((time.time()-self.previousTime)>=self.frameTime):
                    self.previousTime = time.time()

                    retVal = self.getFrames()

                    self.computeDisparity()
                    cv2.imshow("leftDisparity", self.leftDisparityImg)

                    key = cv2.waitKey(20)
                    if key == 27: # Exit on ESC
                        break
                    if key == 32: # Capture on SPACE
                        self.captureImages()

            cv2.destroyAllWindows()
            
        else:
            print('Could not initialize camera pair')

    
    def createStereoSGBM(self):
        self.leftMatcherSGBM = cv2.StereoSGBM_create(
            minDisparity=self.minDisparity,
            numDisparities=self.numDisparities,
            blockSize=self.blockSize,
            P1=8 * 3 * self.windowSize ** 2,
            P2=32 * 3 * self.windowSize ** 2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
    

    def createStereoBM(self):
        self.leftMatcherBM = cv2.StereoBM_create(
            numDisparities=self.numDisparities,
            blockSize=self.blockSize
        )

    
    def computeDisparity(self):
        self.leftImgGray = cv2.cvtColor(self.leftImg, cv2.COLOR_BGR2GRAY)
        self.rightImgGray = cv2.cvtColor(self.rightImg, cv2.COLOR_BGR2GRAY)

        # Compute the left disparity map
        self.leftDisparityImg = self.leftMatcherBM.compute(self.leftImgGray, self.rightImgGray).astype(np.float32)/16



if __name__=="__main__":
    Stereo = StereoCam(leftID=2, rightID=0, width=1280, height=720, fps=2)
    Stereo.loadCalibration()
    Stereo.camPreview()
    #Stereo.previewDisparity()