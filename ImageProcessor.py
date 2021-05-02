import datetime
import multiprocessing
import os
import time

import cv2
import numpy as np

from helperScripts import jsonHelper
from StereoMatcher import StereoMatcher
from VoxelGrid import VoxelGrid


class ImageProcessor(multiprocessing.Process):
    """Class to handle image processing on capture stream"""
    def __init__(self, vertical=True):
        super(ImageProcessor, self).__init__()

        # Flags
        # Vertical or horizontal stereo rig
        self.vertical = vertical

        # Debug
        self.verbose = False

        # Capture link
        self.captureBufferReady = False
        self.captureEventReady = False

        # Calibration load
        self.monoCalibrationLoaded = False
        self.cameraPropertiesLoaded = False
        self.stereoCalibrationLoaded = False
        self.stereoRectifyLoaded = False


    ### Methods to link up with capture process

    def referenceCaptureBuffers(self, buffers, cvImageShape):
        """Create class references to passed capture buffers and 
        shape buffers into image arrays"""
        # Creating class references to buffers
        assert buffers is not None and cvImageShape is not None, \
                                            "Initialize capture buffers"
        self.leftImageBuffer = buffers[0]
        self.rightImageBuffer = buffers[1]
        self.captureTimeBuffer = buffers[2]

        self.cvImageShape = cvImageShape
        self.imageResolution = (self.cvImageShape[1], self.cvImageShape[0])

        # Creating arrays from memory buffers
        self.imageL = np.frombuffer(self.leftImageBuffer, \
                            dtype=np.uint8).reshape(self.cvImageShape)
        self.imageR = np.frombuffer(self.rightImageBuffer, \
                            dtype=np.uint8).reshape(self.cvImageShape)
        self.captureTime = np.frombuffer(self.captureTimeBuffer,\
                            dtype=np.float64)
        
        self.captureBufferReady = True

    
    def isCaptureBufferReady(self):
        """Check if capture buffers have been referenced"""
        if self.captureBufferReady:
            return True
        
        else:
            print("Capture buffers not referenced")
            return False


    def referenceCaptureEvents(self, events):
        """Create class references to passed capture events"""
        self.imageEvent = events[0]
        self.quitEvent = events[1]

        self.captureEventReady = True


    def isCaptureEventReady(self):
        """Check if capture events have been referenced"""
        if self.captureEventReady:
            return True
        
        else:
            print("Capture events not referenced")
            return False

    
    def isCapturePipelineReady(self):
        """Check if the capture pipeline is ready"""
        if not self.isCaptureBufferReady() or \
            not self.isCaptureEventReady():
            print("Capture pipeline is not ready")
            return False
        
        else:
            return True


    ### Methods to preview and save capture 

    def captureImages(self, imageFormat=".png", path="captures/"):
        """Capture and save images from both cameras"""
        if self.vertical:
            fileNames = [("top", self.imageL), ("bottom", self.imageR)]
        
        else:
            fileNames = [("left", self.imageL), ("right", self.imageR)]

        timeString = datetime.datetime.now().strftime("%d%m%y%H%M%S")

        for (camera, frame) in fileNames:
            imageName = "".join([camera, "_{}{}".format(timeString, \
                                                imageFormat)])
            cv2.imwrite(os.path.join(path, imageName), frame)
            print("Saved {} to {}".format(imageName, path))

    
    def pollCapture(self):
        """Wait for and acknowledge next frame pair loaded into buffer"""
        if self.imageEvent.wait():
            self.pickupTime = self.captureTime[0]
            self.imageEvent.clear()


    def previewCapture(self):
        """Previews capture. Event managed"""
        if not self.isCapturePipelineReady():
            return

        if self.vertical:
            windowNames = ["Top", "Bottom"]
        else:
            windowNames = ["Left", "Right"]

        while not self.quitEvent.is_set():
            self.pollCapture()
            
            if self.vertical:
                cv2.imshow(windowNames[0], cv2.rotate(self.imageL, \
                    cv2.ROTATE_90_CLOCKWISE))
                cv2.imshow(windowNames[1], cv2.rotate(self.imageR, \
                    cv2.ROTATE_90_CLOCKWISE))

            else:
                cv2.imshow(windowNames[0], self.imageL)
                cv2.imshow(windowNames[1], self.imageR)

            if self.verbose:
                print("Preview time: {:.5f}"\
                    .format(time.time()-self.pickupTime))

            key = cv2.waitKey(20)
            if key == 27: # Exit on ESC
                print("Exiting preview")
                self.quitEvent.set()
                break
            if key == 32: # Capture on SPACE
                self.captureImages()
            
        cv2.destroyAllWindows()


    def drawHorEpipolarLines(self, undistortImageL, undistortImageR):
        """Returns image with horizontal epipolar lines drawn to confirm 
        rectification"""
        epipolarImageL = undistortImageL.copy()
        epipolarImageR = undistortImageR.copy()

        for line in range(0, int(undistortImageL.shape[0]/20)): 
                epipolarImageL[line*20,:]= 255
                epipolarImageR[line*20,:]= 255

        return np.hstack([epipolarImageL, \
                                            epipolarImageR])


    def drawVertEpipolarLines(self, undistortImageL, undistortImageR):
        """Returns image with vertical epipolar lines drawn to confirm 
        rectification"""
        epipolarImageL = undistortImageL.copy()
        epipolarImageR = undistortImageR.copy()

        for line in range(0, int(undistortImageL.shape[1]/20)): 
                epipolarImageL[:,line*20]= 255
                epipolarImageR[:,line*20]= 255

        return np.vstack([epipolarImageL, \
                                            epipolarImageR])

    
    def previewUndistortCapture(self):
        """Previews capture, undistorted. Event managed"""
        if not self.isCapturePipelineReady():
            return

        self.loadMonoCalibrationResults()
        self.loadStereoCalibration()
        self.loadStereoRectify()
        self.initUndistortRectifyMap()

        if self.vertical:
            windowNames = ["Undistorted_Top", "Undistorted_Bottom"]
        else:
            windowNames = ["Undistorted_Left", "Undistorted_Right"]

        while not self.quitEvent.is_set():
            self.pollCapture()

            self.undistortRectifyRemap(self.imageL, self.imageR)

            if self.vertical:
                cv2.imshow(windowNames[0], cv2.rotate(self.undistortImageL, \
                    cv2.ROTATE_90_CLOCKWISE))
                cv2.imshow(windowNames[1], cv2.rotate(self.undistortImageR, \
                    cv2.ROTATE_90_CLOCKWISE))

            else:
                cv2.imshow(windowNames[0], self.undistortImageL)
                cv2.imshow(windowNames[1], self.undistortImageR)

            # Draw epipolar lines to check rectification
            cv2.imshow("Vertical_Epipolar", \
                self.drawVertEpipolarLines(self.undistortImageL, \
                                            self.undistortImageR))
            cv2.imshow("Horizontal_Epipolar", \
                self.drawHorEpipolarLines(self.undistortImageL, \
                                            self.undistortImageR))

            key = cv2.waitKey(20)
            if key == 27: # Exit on ESC
                print("Exiting preview")
                self.quitEvent.set()
                break
            
        cv2.destroyAllWindows()

    
    def previewDisparity(self):
        """Compute and preview disparity map"""
        if not self.isCapturePipelineReady():
            return

        self.loadMonoCalibrationResults()
        self.loadStereoCalibration()
        self.loadStereoRectify()

        self.stereoMatcher = StereoMatcher("SGBM", \
                vertical=self.vertical, createRightMatcher=False)
        self.voxelGrid = VoxelGrid(stereoMatcher=self.stereoMatcher)
        
        self.stereoMatcher.referenceImageProcessor(self)
        self.stereoMatcher.referenceVoxelGrid(self.voxelGrid)

        self.initUndistortRectifyMap()


        while not self.quitEvent.is_set():
            self.pollCapture()

            self.convertToGrayscale(self.imageL, self.imageR)
            self.undistortRectifyRemap(self.grayImageL, self.grayImageR)

            self.stereoMatcher.computeDisparity(\
                grayImageL=self.undistortImageL, \
                grayImageR=self.undistortImageR)

            self.stereoMatcher.clampDisparity()
            self.stereoMatcher.applyClosingFilter()

            if self.vertical:
                cv2.imshow("Disparity", \
                    cv2.rotate((self.stereoMatcher.disparityMapL\
                        -self.stereoMatcher.minDisparity)/\
                        self.stereoMatcher.numDisparities, \
                        cv2.ROTATE_90_CLOCKWISE))
            
            else:
                cv2.imshow("Disparity", (self.stereoMatcher.disparityMapL\
                    -self.stereoMatcher.minDisparity)/\
                    self.stereoMatcher.numDisparities)

            if self.vertical:
                cv2.imshow("Horizontal_Epipolar", \
                    self.drawHorEpipolarLines(self.undistortImageL, \
                        self.undistortImageR))
            else:
                cv2.imshow("Vertical_Epipolar", \
                    self.drawVertEpipolarLines(self.undistortImageL, \
                        self.undistortImageR))

            if self.verbose:
                print("Disparity compute time: {:.5f}"\
                    .format(time.time()-self.pickupTime))

            if self.stereoMatcher.tuneParameters(self):
                self.stereoMatcher.createMatcher()
            else:
                self.quitEvent.set()
                break

        cv2.destroyAllWindows()


    ### Methods to load calibrations and camera properties

    def loadMonoCalibrationResults(self, path="data/monoCalibration.json"):
        """Loads mono calibration data from given json"""

        dataDict = jsonHelper.jsonToDict(path)

        # Left
        self.cameraMatrixL = np.array(dataDict["left"]["cameraMatrix"])
        self.distortionCoeffsL = np.array(\
                                    dataDict["left"]["distortionCoeffs"])

        # Right
        self.cameraMatrixR = np.array(dataDict["right"]["cameraMatrix"])
        self.distortionCoeffsR = np.array(\
                                    dataDict["right"]["distortionCoeffs"])

        print("Loaded mono calibration")

        self.monoCalibrationLoaded = True


    def loadCameraProperties(self, path="data/cameraProperties.json"):
        """Loads camera characteristics from json"""

        dataDict = jsonHelper.jsonToDict(path)

        # Left
        self.fovXL = dataDict["left"]["fieldOfView"]["horizontal"]
        self.fovYL = dataDict["left"]["fieldOfView"]["vertical"]
        self.focalLengthL = dataDict["left"]["focalLength"]
        self.principalPointL = dataDict["left"]["principalPoint"]
        self.aspectRatioL = dataDict["left"]["aspectRatio"]

        # Right
        self.fovXR = dataDict["right"]["fieldOfView"]["horizontal"]
        self.fovYR = dataDict["right"]["fieldOfView"]["vertical"]
        self.focalLengthR = dataDict["right"]["focalLength"]
        self.principalPointR = dataDict["right"]["principalPoint"]
        self.aspectRatioR = dataDict["right"]["aspectRatio"]

        print("Loaded camera properties")

        self.cameraPropertiesLoaded = True


    def loadStereoCalibration(self, path="data/stereoCalibration.json"):
        """Loads stereo calibration data from given json"""

        dataDict = jsonHelper.jsonToDict(path)

        # Left
        self.cameraMatrixL = np.array(dataDict["left"]["cameraMatrix"])
        self.distortionCoeffsL = np.array(\
                                    dataDict["left"]["distortionCoeffs"])

        # Right
        self.cameraMatrixR = np.array(dataDict["right"]["cameraMatrix"])
        self.distortionCoeffsR = np.array(\
                                    dataDict["right"]["distortionCoeffs"])

        # Common
        self.stereoRotationMatrix = np.array(\
                                    dataDict["stereoRotationMatrix"])
        self.stereoTranslationMatrix = np.array(\
                                    dataDict["stereoTranslationMatrix"])

        self.essentialMatrix = np.array(dataDict["essentialMatrix"]) 
        self.fundamentalMatrix = np.array(dataDict["fundamentalMatrix"])

        # Image size
        self.grayImageSizeL = tuple(dataDict["grayImageSizeL"])
        self.grayImageSizeR = tuple(dataDict["grayImageSizeR"])

        self.imageSizeL = tuple(dataDict["imageSizeL"])
        self.imageSizeR = tuple(dataDict["imageSizeR"])

        print("Loaded stereo calibration")

        self.stereoCalibrationLoaded = True


    def loadStereoRectify(self, path="data/stereoRectify.json"):
        """Loads stereo rectification data from given json"""

        dataDict = jsonHelper.jsonToDict(path)

        # Left
        self.rotationMatrixL = np.array(\
                                    dataDict["left"]["rotationMatrix"])
        self.projectionMatrixL = np.array(\
                                    dataDict["left"]["projectionMatrix"])

        # Right
        self.rotationMatrixR = np.array(\
                                    dataDict["right"]["rotationMatrix"])
        self.projectionMatrixR = np.array(\
                                    dataDict["right"]["projectionMatrix"])

        # Common
        # Q matrix
        self.dispToDepthMatrix = np.array(\
                        dataDict["dispToDepthMatrix"], dtype=np.float32)

        print("Loaded stereo rectification data")

        self.stereoRectifyLoaded = True


    ### Disparity map generation
    
    def initUndistortRectifyMap(self):
        """Computes the joint undistortion and rectification transformation 
        and represents the result in the form of maps for remap()"""

        if not self.stereoRectifyLoaded:
            print("Stereo rectification data not loaded")
            return

        if not self.stereoCalibrationLoaded:
            print("Stereo calibration not loaded")
            return

        # Left
        self.undistortMapL = cv2.initUndistortRectifyMap(\
                self.cameraMatrixL, self.distortionCoeffsL, \
                self.rotationMatrixL, self.projectionMatrixL, \
                self.grayImageSizeL[::-1], cv2.CV_16SC2)  
        # cv2.CV_16SC2 format enables faster execution

        # Right
        self.undistortMapR = cv2.initUndistortRectifyMap(\
                self.cameraMatrixR, self.distortionCoeffsR, \
                self.rotationMatrixR, self.projectionMatrixR, \
                self.grayImageSizeR[::-1], cv2.CV_16SC2)

    
    def undistortRectifyRemap(self, imageL, imageR):
        """Calls remap on passed images using undistortMaps to undistort 
        and rectify image pair"""
        # Left
        self.undistortImageL = cv2.remap(\
            imageL, self.undistortMapL[0], self.undistortMapL[1], \
            cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

        # Right
        self.undistortImageR= cv2.remap(\
            imageR, self.undistortMapR[0], self.undistortMapR[1], \
            cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

    
    def convertToGrayscale(self, imageL, imageR):
        """Convert BGR images to GRAY"""
        self.grayImageL = cv2.cvtColor(imageL, cv2.COLOR_BGR2GRAY)
        self.grayImageR = cv2.cvtColor(imageR, cv2.COLOR_BGR2GRAY)


    ### Methods to set context and start processes

    def setContext(self, context):
        """Selects which methods to run on start() based on context"""
        self.context = context


    def run(self):
        """Runs when start() is called on this process"""
        if self.context=="preview":
            self.previewCapture()

        elif self.context=="previewDisparity":
            self.previewDisparity()

        elif self.context=="previewUndistort":
            self.previewUndistortCapture()



if __name__=="__main__":
    print("Handled by ProcessManager")
