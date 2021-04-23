import ctypes
import multiprocessing
import time

import cv2
import numpy


class StereoCapture(multiprocessing.Process):
    """Class to handle capture from stereo camera setup"""
    def __init__(self, cameraID=(2,0), imageSize=(1280/2,720/2), fps=4):
        super(StereoCapture, self).__init__()

        # Camera IDs and properties
        self.leftID = cameraID[0]
        self.rightID = cameraID[1]
        self.imageSize = imageSize
        self.fps = fps

        # Seting up capture objects
        self.leftCam = cv2.VideoCapture(self.leftID)
        self.rightCam = cv2.VideoCapture(self.rightID)

        # Set camera resolution
        # Left
        self.leftCam.set(3, self.imageSize[0])
        self.leftCam.set(4, self.imageSize[1])
        # Right
        self.rightCam.set(3, self.imageSize[0])
        self.rightCam.set(4, self.imageSize[1])

        # Set format to MJPG in FourCC format 
        self.leftCam.set(cv2.CAP_PROP_FOURCC, \
            cv2.VideoWriter_fourcc('M','J','P','G'))

        self.rightCam.set(cv2.CAP_PROP_FOURCC, \
            cv2.VideoWriter_fourcc('M','J','P','G'))

        # Timing
        self.previousTime = time.time()
        self.frameTime = float(1/self.fps)

        # Events
        self.imageEvent = multiprocessing.Event()
        self.quitEvent = multiprocessing.Event()

        # Flags
        self.bufferReady = False


    def checkOpen(self):
        """Check if cameras have initialized"""
        if self.leftCam.isOpened() and self.rightCam.isOpened():
            return True
        else:
            raise IOError("Could not initialize camera pair")

    
    def createBuffers(self):
        """Create and shape buffers into image arrays"""
        # Check if camera pair has initialized
        if not self.checkOpen():
            return
        
        print("Initialized camera pair")

        # Finding image shape
        retVal, leftImage = self.leftCam.read()
        self.cvImageShape = leftImage.shape
        self.cvImageSize = leftImage.size

        # Creating shared memory buffers
        self.leftImageBuffer = multiprocessing.Array(ctypes.c_uint8, \
            self.cvImageSize, lock=False)
        self.rightImageBuffer = multiprocessing.Array(ctypes.c_uint8, \
            self.cvImageSize, lock=False)
        self.captureTimeBuffer = multiprocessing.Array(ctypes.c_double, \
            1, lock=False)

        # Creating arrays from memory buffers
        self.imageL = numpy.frombuffer(self.leftImageBuffer, \
                            dtype=numpy.uint8).reshape(self.cvImageShape)
        self.imageR = numpy.frombuffer(self.rightImageBuffer, \
                            dtype=numpy.uint8).reshape(self.cvImageShape)
        self.captureTime = numpy.frombuffer(self.captureTimeBuffer,\
                            dtype=numpy.float64)

        self.bufferReady = True
        print("Initialized capture buffers")

    
    def isBufferReady(self):
        """Check if buffers have been created"""
        if self.bufferReady:
            return True
        
        else:
            print("Buffers not initialized")
            return False

    
    def getCVImageShape(self):
        if self.bufferReady:
            return self.cvImageShape
        
        else:
            print("Buffers not initialized")
            return None

    
    def getBuffers(self):
        """Returns references of left, right internal buffers"""
        if self.isBufferReady():
            return (self.leftImageBuffer, self.rightImageBuffer, \
                    self.captureTimeBuffer)
        
        else:
            return None

    
    def getCaptureEvents(self):
        """Returns references of image, quit events"""
        return (self.imageEvent, self.quitEvent)


    def getFrames(self):
        """Grab and retrieve frames"""
        self.previousTime = time.time()

        for i in range(3):
            self.leftCam.grab()
            self.rightCam.grab()
        
        self.captureTime[0] = time.time()

        # Write images to buffer instead of assigning as array
        retLeft, self.imageL[:] = self.leftCam.retrieve()
        retRight, self.imageR[:] = self.rightCam.retrieve()

        if retLeft and retRight:
            self.actualFrameTime = time.time() - self.previousTime
            self.remainingTime = self.frameTime - self.actualFrameTime

            self.imageEvent.set()
            return True

        else:
            return False


    def capture(self):
        """Capture images from camera pair"""
        # Check if camera and buffers have initialized
        if not self.checkOpen() or not self.isBufferReady():
            return
            
        # Capture frames from camera pair
        while self.getFrames():

            if self.quitEvent.is_set():
                break     
                
            print("Capture time: {:.5f}  Frame time: {:.5f}  "\
                .format(self.captureTime[0], self.actualFrameTime), end="")
            print("Remaining time: {:.5f}  Frame is:"\
                .format(self.remainingTime), end=" ")

            if self.remainingTime<0:
                print("late")

            else:
                print("on time")
                time.sleep(self.remainingTime) 
        
        self.leftCam.release()
        self.rightCam.release()

        
    def run(self):
        """Runs when start() is called on this process"""
        self.capture()



if __name__=="__main__":
    print("Handled by ProcessManager")
