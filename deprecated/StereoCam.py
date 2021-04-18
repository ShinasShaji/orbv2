import os
import threading
import time

import cv2
import numpy as np

import jsonHandler


class StereoCam:
    """Class to handle a camera pair as a synchronized stereo pair"""
    def __init__(self, leftID, rightID, width = 1280, height = 720, fps=2):
        self.leftCam = cv2.VideoCapture(leftID)
        self.rightCam = cv2.VideoCapture(rightID)

        # Set camera resolution
        self.imageSize = (width, height)
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

        # For event managed, threaded capture and preview
        self.imageEvent = threading.Event()
        self.quitEvent = threading.Event()
        self.captureRetVal = True

        # Capture details
        self.imgCounter = 0
        self.capturePath = "captures/" 

        # json data load flags
        self.monoCalibrationLoaded = False
        self.cameraPropertiesLoaded = False
        self.stereoCalibrationLoaded = False


    def checkOpen(self):
        """Check if cameras have initialized"""
        if self.leftCam.isOpened() and self.rightCam.isOpened():
            return True
        else:
            print('Could not initialize camera pair')
            return False


    def getFrames(self):
        """Grab and retrieve frames"""
        for i in range(10):
            self.leftCam.grab()
            self.rightCam.grab()

        retLeft, self.leftImg = self.leftCam.retrieve()
        retRight, self.rightImg = self.rightCam.retrieve()

        if retLeft and retRight:
            self.captureRetVal = True
            return True

        else:
            self.captureRetVal = False
            return False


    def captureImages(self):
        """Capture and save images from both cameras"""
        for (cam, frame) in [("left", self.leftImg), ("right", self.rightImg)]:
            imgName = "".join([cam, "_{}.png".format(self.imgCounter)])
            cv2.imwrite(os.path.join(self.capturePath, imgName), frame)
            print("Captured {}".format(imgName))

        self.imgCounter += 1
    

    def startCapture_Internal(self):
        print("Initializing cameras for preview")
        if not self.checkOpen():
            pass

        while self.getFrames():

            self.imageEvent.set()
            if self.quitEvent.isSet():
                break

            remainingTime = self.frameTime-(time.time()-self.previousTime)
            if remainingTime<=0:
                self.previousTime = time.time()

            else:
                time.sleep(remainingTime)

        
    def startCapture(self):
        """Runs startCapture_Internal() as a thread"""
        self.captureThread = threading.Thread(target=self.startCapture_Internal)
        self.captureThread.start()

    
    def eventManagedPreview_Internal(self):
        """Previews capture. Event managed thread"""
        while self.captureRetVal:
            if self.imageEvent.wait():
                cv2.imshow('Left', self.leftImg)
                cv2.imshow('Right', self.rightImg)

                self.imageEvent.clear()

                key = cv2.waitKey(20)
                if key == 27: # Exit on ESC
                    print("Exiting preview")
                    break
                if key == 32: # Capture on SPACE
                    self.captureImages()
            
        cv2.destroyAllWindows()
        self.quitEvent.set()

    
    def eventManagedPreview(self):
        """Runs eventManagedPreview_Internal() as a thread"""
        self.previewThread = threading.Thread(target=self.eventManagedPreview_Internal)
        self.previewThread.start()


    def camPreview(self):
        """Preview camera outputs"""
        self.startCapture()
        self.eventManagedPreview()


### To do: Load stereo calibration data and matrices from jsons ###


    def loadMonoCalibrationResults(self, path="data/monoCalibration.json"):
        """Loads mono calibration data from given json"""

        dataDict = jsonHandler.jsonToDict(path)

        # Left
        self.cameraMatrixL = np.array(dataDict["left"]["cameraMatrix"])
        self.distortionCoeffsL = np.array(dataDict["left"]["distortionCoeffs"])

        # Right
        self.cameraMatrixR = np.array(dataDict["right"]["cameraMatrix"])
        self.distortionCoeffsR = np.array(dataDict["right"]["distortionCoeffs"])

        print("Loaded mono calibration")

        self.monoCalibrationLoaded = True


    def loadCameraProperties(self, path="data/cameraProperties.json"):
        """Loads camera characteristics from json"""

        ### To do: Implement ###

        print("Loaded camera properties")

        self.cameraPropertiesLoaded = True

        pass


    def loadStereoCalibration(self, path="data/stereoCalibration.json"):
        """Loads stereo calibration data from given json"""

        dataDict = jsonHandler.jsonToDict(path)

        # Left
        self.cameraMatrixL = np.array(dataDict["left"]["cameraMatrix"])
        self.distortionCoeffsL = np.array(dataDict["left"]["distortionCoeffs"])

        # Right
        self.cameraMatrixR = np.array(dataDict["right"]["cameraMatrix"])
        self.distortionCoeffsR = np.array(dataDict["right"]["distortionCoeffs"])

        # Common
        self.stereoRotationMatrix = np.array(dataDict["stereoRotationMatrix"])
        self.stereoTranslationMatrix = np.array(dataDict["stereoTranslationMatrix"])
        self.essentialMatrix = np.array(dataDict["essentialMatrix"]) 
        self.fundamentalMatrix = np.array(dataDict["fundamentalMatrix"])
        self.rectifyScale = dataDict["alpha"]

        # Image size
        self.imageSizeL = dataDict["imageSizeL"]
        self.imageSizeR = dataDict["imageSizeR"]

        if self.imageSize!=self.imageSizeL or self.imageSize!=self.imageSizeR:
            print("Image size mismatch")

            self.stereoCalibrationLoaded = False

        else:
            print("Loaded stereo calibration")

            self.stereoCalibrationLoaded = True       



if __name__=="__main__":
    Stereo = StereoCam(leftID=2, rightID=0, width=1280, height=720, fps=2)
    Stereo.camPreview()
