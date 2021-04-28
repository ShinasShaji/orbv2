import datetime
import multiprocessing
import os
import time

import cv2
import numpy

from helperScripts import jsonHelper


class ImageProcessing(multiprocessing.Process):
    """Class to handle image processing on capture stream"""
    def __init__(self, vertical=True):
        super(ImageProcessing, self).__init__()

        # Capture details
        self.capturePath = "captures/"

        # Flags
        # Vertical or horizontal stereo rig
        self.vertical = vertical

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
        self.imageL = numpy.frombuffer(self.leftImageBuffer, \
                            dtype=numpy.uint8).reshape(self.cvImageShape)
        self.imageR = numpy.frombuffer(self.rightImageBuffer, \
                            dtype=numpy.uint8).reshape(self.cvImageShape)
        self.captureTime = numpy.frombuffer(self.captureTimeBuffer,\
                            dtype=numpy.float64)
        
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

    def captureImages(self):
        """Capture and save images from both cameras"""
        if self.vertical:
            fileNames = [("top", self.imageL), ("bottom", self.imageR)]
        
        else:
            fileNames = [("left", self.imageL), ("right", self.imageR)]

        timeString = datetime.datetime.now().strftime("%d%m%y%H%M%S")

        for (camera, frame) in fileNames:
            imageName = "".join([camera, "_{}.png".format(timeString)])
            cv2.imwrite(os.path.join(self.capturePath, imageName), frame)
            print("Captured {}".format(imageName))

    
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

            cv2.imshow(windowNames[0], self.imageL)
            cv2.imshow(windowNames[1], self.imageR)

            key = cv2.waitKey(20)
            if key == 27: # Exit on ESC
                print("Exiting preview")
                self.quitEvent.set()
                break
            if key == 32: # Capture on SPACE
                self.captureImages()
            
        cv2.destroyAllWindows()


    def drawUndistortHorEpipolarLines(self):
        """Returns image with horizontal epipolar lines drawn to confirm 
        rectification"""
        epipolarImageL = self.undistortImageL.copy()
        epipolarImageR = self.undistortImageR.copy()

        for line in range(0, int(self.undistortImageL.shape[0]/20)): 
                epipolarImageL[line*20,:]= (0,255,0)
                epipolarImageR[line*20,:]= (0,255,0)

        return numpy.hstack([epipolarImageL, \
                                            epipolarImageR])


    def drawUndistortVertEpipolarLines(self):
        """Returns image with vertical epipolar lines drawn to confirm 
        rectification"""
        epipolarImageL = self.undistortImageL.copy()
        epipolarImageR = self.undistortImageR.copy()

        for line in range(0, int(self.undistortImageL.shape[1]/20)): 
                epipolarImageL[:,line*20]= (0,255,0)
                epipolarImageR[:,line*20]= (0,255,0)

        return numpy.vstack([epipolarImageL, \
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

            self.undistortRectifyRemap()

            cv2.imshow(windowNames[0], self.undistortImageL)
            cv2.imshow(windowNames[1], self.undistortImageR)

            # Draw epipolar lines to check rectification
            cv2.imshow("Vertical_Epipolar", \
                                self.drawUndistortVertEpipolarLines())
            cv2.imshow("Horizontal_Epipolar", \
                                self.drawUndistortHorEpipolarLines())

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
        self.createStereoMatcher("SGBM")
        self.initUndistortRectifyMap()


        while not self.quitEvent.is_set():
            self.pollCapture()

            self.convertCaptureToGrayscale()
            self.undistortRectifyRemapGray()

            self.rotateGrayCC90()
            self.computeDisparityMapL()
            self.rotateDisparityC90()

            self.clampDisparity(2, 32)

            cv2.imshow("Disparity", self.disparityMap)

            print("Disparity compute time: {:.5f}"\
                .format(time.time()-self.pickupTime))

            key = cv2.waitKey(20)
            if key == 27: # Exit on ESC
                print("Exiting disparity preview")
                self.quitEvent.set()
                break

        cv2.destroyAllWindows()


    ### Methods to load calibrations and camera properties

    def loadMonoCalibrationResults(self, path="data/monoCalibration.json"):
        """Loads mono calibration data from given json"""

        dataDict = jsonHelper.jsonToDict(path)

        # Left
        self.cameraMatrixL = numpy.array(dataDict["left"]["cameraMatrix"])
        self.distortionCoeffsL = numpy.array(\
                                    dataDict["left"]["distortionCoeffs"])

        # Right
        self.cameraMatrixR = numpy.array(dataDict["right"]["cameraMatrix"])
        self.distortionCoeffsR = numpy.array(\
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
        self.cameraMatrixL = numpy.array(dataDict["left"]["cameraMatrix"])
        self.distortionCoeffsL = numpy.array(\
                                    dataDict["left"]["distortionCoeffs"])

        # Right
        self.cameraMatrixR = numpy.array(dataDict["right"]["cameraMatrix"])
        self.distortionCoeffsR = numpy.array(\
                                    dataDict["right"]["distortionCoeffs"])

        # Common
        self.stereoRotationMatrix = numpy.array(\
                                    dataDict["stereoRotationMatrix"])
        self.stereoTranslationMatrix = numpy.array(\
                                    dataDict["stereoTranslationMatrix"])

        self.essentialMatrix = numpy.array(dataDict["essentialMatrix"]) 
        self.fundamentalMatrix = numpy.array(dataDict["fundamentalMatrix"])

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
        self.rotationMatrixL = numpy.array(\
                                    dataDict["left"]["rotationMatrix"])
        self.projectionMatrixL = numpy.array(\
                                    dataDict["left"]["projectionMatrix"])

        # Right
        self.rotationMatrixR = numpy.array(\
                                    dataDict["right"]["rotationMatrix"])
        self.projectionMatrixR = numpy.array(\
                                    dataDict["right"]["projectionMatrix"])

        # Common
        # Q matrix
        self.dispToDepthMatrix = dataDict["dispToDepthMatrix"]

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
        # cv2.CV_16SC2: Format enables faster execution

        # Right
        self.undistortMapR = cv2.initUndistortRectifyMap(\
                self.cameraMatrixR, self.distortionCoeffsR, \
                self.rotationMatrixR, self.projectionMatrixR, \
                self.grayImageSizeR[::-1], cv2.CV_16SC2)

    
    def createStereoSGBM(self):
        """Create SGBM stereo matchers, implementing the modified 
        H. Hirschmuller algorithm"""
        # Parameters
        blockSize = 3
        minDisparity = 2
        numDisparities = 32 - minDisparity
        penalty1 = 8*3*blockSize**2
        penalty2 = 32*3*blockSize**2
        uniquenessRatio = 10
        speckleWindowSize = 50
        speckleRange = 2
        disp12MaxDiff = 5
        
        # Creating StereoSGBM matchers
        # Left
        self.stereoL = cv2.StereoSGBM_create(minDisparity=minDisparity, \
            numDisparities=numDisparities, blockSize=blockSize, \
            uniquenessRatio=uniquenessRatio, \
            speckleRange = speckleRange, \
            speckleWindowSize=speckleWindowSize, \
            disp12MaxDiff=disp12MaxDiff, \
            P1 = penalty1, P2 = penalty2)

        # Right
        self.stereoR = cv2.ximgproc.createRightMatcher(self.stereoL)

    
    def createStereoBM(self):
        """Create Stereo BM matchers"""
        # Parameters
        numDisparities = 96
        blockSize = 21

        self.stereoL = cv2.StereoBM_create(\
                                    numDisparities=numDisparities, \
                                    blockSize=blockSize)

        self.stereoR = cv2.ximgproc.createRightMatcher(self.stereoL)

    
    def createStereoMatcher(self, matcher=None):
        """Create StereoBM "BM" or StereoSGBM "SGBM" matcher objects"""
        assert matcher is not None, "No matcher specified"

        if matcher=="SGBM":
            self.createStereoSGBM()

        elif matcher=="BM":
            self.createStereoBM()

        self.kernel = numpy.ones((3,3),numpy.uint8)


    def createDisparityWLSFilter(self):
        """Create disparity map filter based on Weighted Least Squares 
        filter"""
        # Parameters
        lmbda = 80000
        sigma = 1.8
        visualMultiplier = 1.0

        # Creating filter
        self.wlsFilter = cv2.ximgproc.createDisparityWLSFilter(\
                                            matcher_left=self.stereoL)
        
        # Setting parameters
        self.wlsFilter.setLambda(lmbda)
        self.wlsFilter.setSigmaColor(sigma)

    
    def undistortRectifyRemap(self):
        """Calls remap on the images using undistortMaps to undistort and
        rectify image pair"""
        # Left
        self.undistortImageL = cv2.remap(\
            self.imageL, self.undistortMapL[0], self.undistortMapL[1], \
            cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

        # Right
        self.undistortImageR= cv2.remap(\
            self.imageR, self.undistortMapR[0], self.undistortMapR[1], \
            cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

    
    def undistortRectifyRemapGray(self):
        """Calls remap on the images using undistortMaps to undistort and
        rectify image pair"""
        # Left
        self.grayImageL = cv2.remap(\
            self.grayImageL, self.undistortMapL[0], self.undistortMapL[1], \
            cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

        # Right
        self.grayImageR = cv2.remap(\
            self.grayImageR, self.undistortMapR[0], self.undistortMapR[1], \
            cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

    
    def convertCaptureToGrayscale(self):
        """Convert captures BGR images to GRAY"""
        self.grayImageL = cv2.cvtColor(self.imageL, cv2.COLOR_BGR2GRAY)
        self.grayImageR = cv2.cvtColor(self.imageR, cv2.COLOR_BGR2GRAY)

    
    def convertUndistortToGrayscale(self):
        """Convert undistorted BGR images to GRAY"""
        self.grayImageL = cv2.cvtColor(self.undistortImageL, \
                                                    cv2.COLOR_BGR2GRAY)
        self.grayImageR = cv2.cvtColor(self.undistortImageR, \
                                                    cv2.COLOR_BGR2GRAY)
    

    def computeDisparityMapL(self):
        """Compute the left disparity map from current grayscale 
        images"""
        self.disparityMap = self.stereoL.compute(\
                                        self.grayImageL, self.grayImageR)
        self.disparityMapL = numpy.int16(self.disparityMap)

    
    def computeDisparityMapR(self):
        """Compute the left disparity map from current grayscale 
        images"""
        self.disparityMapR = numpy.int16(self.stereoR.compute(\
                                        self.grayImageR, self.grayImageL))

    
    def rotateGrayCC90(self):
        """Rotate current grayscale images by 90 degress counterclockwise"""
        self.grayImageL = cv2.rotate(self.grayImageL, \
                                        cv2.ROTATE_90_COUNTERCLOCKWISE)
        self.grayImageR = cv2.rotate(self.grayImageR, \
                                        cv2.ROTATE_90_COUNTERCLOCKWISE)

                            
    def computeDisparityMapLR(self):
        """Compute the left and right disparity maps from current 
        grayscale images"""
        self.computeDisparityMapL()
        self.computeDisparityMapR()


    def rotateDisparityC90(self):
        """Rotate current disparity image by 90 degrees clockwise"""
        try:
            self.disparityMap = cv2.rotate(self.disparityMap, \
                                        cv2.ROTATE_90_CLOCKWISE)
            self.disparityMapL = cv2.rotate(self.disparityMapL, \
                                        cv2.ROTATE_90_CLOCKWISE)
            self.disparityMapR = cv2.rotate(self.disparityMapR, \
                                        cv2.ROTATE_90_CLOCKWISE)
        
        except:
            pass

                                
    def rotateGrayC90(self):
        """Rotate current grayscale images by 90 degress clockwise"""
        self.grayImageL = cv2.rotate(self.grayImageL, \
                                        cv2.ROTATE_90_CLOCKWISE)
        self.grayImageR = cv2.rotate(self.grayImageR, \
                                        cv2.ROTATE_90_CLOCKWISE)


    def applyWLSFilterDisparity(self):
        """Apply created WLS disparity filter on disparity maps"""
        self.disparityMap = self.wlsFilter.filter(self.disparityMapL, \
                self.grayImageL, disparity_map_right=self.disparityMapR)     


    def clampDisparity(self, minDisparity, numDisparities):
        """Sets 0 for most distant object that can be detected"""
        self.disparityMap = ((self.disparityMap.astype(numpy.float32)\
                                    /16)-minDisparity)/numDisparities


    def applyClosingFilter(self):
        """Applies a closing filter to the disparity map"""
        self.disparityMap = cv2.morphologyEx(\
                    self.disparityMap, cv2.MORPH_CLOSE, self.kernel)    


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

        elif self.context=="undistortPreview":
            self.previewUndistortCapture()



if __name__=="__main__":
    print("Handled by ProcessManager")
