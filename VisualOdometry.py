import ctypes
import multiprocessing
from collections import deque

import cv2
import numpy as np

from helperScripts.TimeKeeper import TimeKeeper


class VisualOdometry(multiprocessing.Process):
    """Class to handle visual odometry routines"""
    def __init__(self):
        super(VisualOdometry, self).__init__()

        # Keep current and previous image in stack for odometry 
        self.imageStackL = deque(maxlen=2)
        # Keypoints and descriptors for the images
        self.keypointStackL = deque(maxlen=2)
        self.descriptorStackL = deque(maxlen=2)
        
        # Debug
        self.verbose = True

        # Capture link
        self.captureBufferReady = False
        self.captureEventReady = False

        # Visual odometry buffers
        self.stateBufferReady = False

        
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


    def createStateBuffers(self, bufferLength=7):
        """Create buffers for state (position and rotation)"""
        # Initialize deques for copying into shared arrays
        self.trajectoryStack = deque(maxlen=bufferLength)
        self.rotationStack = deque(maxlen=bufferLength)

        # Creating shared memory buffers
        # Trajectory state buffer of nx4 size (x,y,z,time)
        self.trajectoryBuffer = multiprocessing.Array(ctypes.c_double, \
            bufferLength*4, lock=False)
        # Rotation state buffer of nx3x3 size
        self.rotationBuffer = multiprocessing.Array(ctypes.c_double, \
            bufferLength*3*3, lock=False)

        # Creating arrays from shared memory buffers
        self.trajectoryWrapper = np.frombuffer(self.trajectoryBuffer, \
                        dtype=np.float64).reshape((bufferLength, 4))
        self.rotationWrapper = np.frombuffer(self.rotationBuffer, \
                        dtype=np.float64).reshape((bufferLength, 3, 3))

        self.stateBufferReady = True
        print("Initialized visual odometry state buffers")

    
    def isCaptureBufferReady(self):
        """Check if capture buffers have been referenced"""
        if self.captureBufferReady:
            return True
        
        else:
            print("Capture buffers not referenced")
            return False

    
    def referenceCaptureEvents(self, events):
        """Create class references to passed capture events"""
        self.visualOdometryEvent = events[0]
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
    

    def isStateBufferReady(self):
        """Check if the state buffers have been created"""
        if self.stateBufferReady:
            return True
        
        else:
            print("State buffers not initialized")

    
    def isVisualOdometryPipelineReady(self):
        """Check if the visual odometry pipeline is ready"""
        if not self.isCapturePipelineReady() or \
            not self.isStateBufferReady():
            print("Visual odometry pipeline is not ready")
            return False
        
        else:
            return True


    def pollCapture(self):
        """Wait for and acknowledge next frame pair loaded into buffer"""
        if self.visualOdometryEvent.wait():
            self.pickupTime = self.captureTime[0]
            # Add latest frame to index 0; latest first
            self.imageStackL.appendleft(self.imageL)

            self.visualOdometryEvent.clear()

    
    def createExtractorORB(self):
        """Create an ORB feature extractor"""
        self.orb = cv2.ORB_create()
            

    def extractFeaturesORB(self):
        """Extract features from latest frame using the initialized ORB
        feature detector"""
        keypoints = self.orb.detect(self.imageStackL[0], None)
        keypoints, descriptors = self.orb.compute(\
                                        self.imageStackL[0], keypoints)

        self.keypointStackL.appendleft(keypoints)
        self.descriptorStackL.appendleft(descriptors)


    def estimateStateRunning(self):
        """Keep a running estimate of the state"""
        if not self.isVisualOdometryPipelineReady():
            return
        
        self.createExtractorORB()
        self.pollCapture()
        self.extractFeaturesORB()

        while not self.quitEvent.is_set():
            self.pollCapture()
