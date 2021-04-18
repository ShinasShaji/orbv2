import multiprocessing
import os

import cv2
import numpy

import jsonHandler


class ImageProcessing(multiprocessing.Process):
    """Class to handle image processing on capture stream"""
    def __init__(self):
        super(ImageProcessing, self).__init__()

        # Capture details
        self.imageCounter = 0
        self.capturePath = "captures/"

        # Flags
        # Capture link
        self.captureBufferReady = False
        self.captureEventReady = False

        # Calibration load
        self.monoCalibrationLoaded = False
        self.cameraPropertiesLoaded = False
        self.stereoCalibrationLoaded = False


    ### Methods to link up with capture process

    def referenceCaptureBuffers(self, buffers, cvImageShape):
        """Create class references to passed capture buffers and 
        shape buffers into image arrays"""
        # Creating class references to buffers
        assert buffers is not None and cvImageShape is not None, \
                                            "Initialize capture buffers"
        self.leftImageBuffer = buffers[0]
        self.rightImageBuffer = buffers[1]
        self.cvImageShape = cvImageShape
        self.imageSize = (self.cvImageShape[1], self.cvImageShape[0])

        # Creating arrays from memory buffers
        self.leftImage = numpy.frombuffer(self.leftImageBuffer, \
                            dtype=numpy.uint8).reshape(self.cvImageShape)
        self.rightImage = numpy.frombuffer(self.rightImageBuffer, \
                            dtype=numpy.uint8).reshape(self.cvImageShape)
        
        self.captureBufferReady = True

    
    def isCaptureBufferReady(self):
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
        if self.captureEventReady:
            return True
        
        else:
            print("Capture events not referenced")
            return False


    ### Methods to preview and save capture 

    def captureImages(self):
        """Capture and save images from both cameras"""
        for (cam, frame) in \
            [("left", self.leftImage), ("right", self.rightImage)]:

            imageName = "".join([cam, "_{}.png".format(self.imageCounter)])
            cv2.imwrite(os.path.join(self.capturePath, imageName), frame)
            print("Captured {}".format(imageName))

        self.imageCounter += 1


    def capturePreview(self):
        """Previews capture. Event managed"""
        if not self.isCaptureBufferReady() or \
            not self.isCaptureEventReady():
            pass

        while not self.quitEvent.is_set():
            if self.imageEvent.wait():
                cv2.imshow('Left', self.leftImage)
                cv2.imshow('Right', self.rightImage)

                self.imageEvent.clear()

                key = cv2.waitKey(20)
                if key == 27: # Exit on ESC
                    print("Exiting preview")
                    self.quitEvent.set()
                    break
                if key == 32: # Capture on SPACE
                    self.captureImages()
            
        cv2.destroyAllWindows()


    ### Methods to load calibrations and camera properties

    def loadMonoCalibrationResults(self, path="data/monoCalibration.json"):
        """Loads mono calibration data from given json"""

        dataDict = jsonHandler.jsonToDict(path)

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

        ### To do: Implement ###

        print("Loaded camera properties")

        self.cameraPropertiesLoaded = True

        pass


    def loadStereoCalibration(self, path="data/stereoCalibration.json"):
        """Loads stereo calibration data from given json"""

        dataDict = jsonHandler.jsonToDict(path)

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


    ### Methods to set context and start processes

    def setContext(self, context):
        """Selects which methods to run on start() based on context"""
        self.context = context


    def run(self):
        """Runs when start() is called on this process"""
        if self.context=="preview":
            self.capturePreview()



if __name__=="__main__":
    print("Handled by ProcessManager")
