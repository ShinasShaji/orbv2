import ctypes
import multiprocessing
from collections import deque
import time

import cv2
import numpy as np

from helperScripts.TimeKeeper import TimeKeeper


class VisualOdometry(multiprocessing.Process):
    """Class to handle visual odometry routines"""
    def __init__(self, bufferLength=7):
        super(VisualOdometry, self).__init__()

        # Buffer length for setting up shared memory buffers
        self.bufferLength = bufferLength

        # Keypoints and descriptors for the images
        self.keypointStackL = deque(maxlen=2)
        self.descriptorStackL = deque(maxlen=2)
        self.match = None

        # Camera properties
        self.cameraMatrixL = None
        self.rotationOffsetL = None
        
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


    def createStateBuffers(self):
        """Create buffers for state (position and rotation)"""
        # Creating shared memory buffers
        # Rotation state buffer of nx3x3 size
        self.rotationBuffer = multiprocessing.Array(ctypes.c_double, \
                        self.bufferLength*3*3, lock=False)
        # Trajectory state buffer of nx4 size (x,y,z,time)
        self.positionBuffer = multiprocessing.Array(ctypes.c_double, \
                        self.bufferLength*4, lock=False)

        # Creating arrays from shared memory buffers
        self.rotationEstimateWrapper = np.frombuffer(self.rotationBuffer, \
                        dtype=np.float64).reshape((self.bufferLength, 3, 3))
        self.positionEstimateWrapper = np.frombuffer(self.positionBuffer, \
                        dtype=np.float64).reshape((self.bufferLength, 4))

        self.flushStateBuffers()

        self.stateBufferReady = True
        print("Initialized visual odometry state buffers")


    def flushStateBuffers(self):
        """Write zeros to position and rotation buffers"""
        self.rotationEstimateWrapper[:] = 0
        self.positionEstimateWrapper[:] = 0

    
    def writeStateStacksToBuffers(self):
        """Write contents of state stacks to the state buffers"""
        self.rotationBuffer[:len(self.rotationEstimateStack)] = \
                                            self.rotationEstimateStack
        self.positionBuffer[:len(self.positionEstimateStack)] = \
                                            self.positionEstimateStack

    
    def getStateBuffers(self):
        """Return references of internal position, rotation state buffers"""
        if self.isStateBufferReady():
            return (self.rotationBuffer, self.positionBuffer)

        else:
            return None
        

    def getBufferLength(self):
        """Return length of state buffers"""
        if self.stateBufferReady:
            return self.bufferLength

        else:
            print("State buffers not initialized")
            return None

    
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
        """Check if state buffers have been initialized"""
        if self.stateBufferReady:
            return True
        
        else:
            print("State buffers not initialized")
            return False

    
    def isVisualOdometryPipelineReady(self):
        """Check if the visual odometry pipeline is ready"""
        if not self.isCapturePipelineReady() or \
            not self.isStateBufferReady():
            print("Visual odometry pipeline is not ready")
            return False
        
        else:
            return True

        
    def setCameraMatrixL(self, cameraMatrixL):
        """Set left camera matrix"""
        self.cameraMatrixL = cameraMatrixL


    def setRotationOffsetL(self, rotationOffsetL):
        """Set rotation offset of left camera from body reference"""
        self.rotationOffsetL = rotationOffsetL


    def setPositionOffsetL(self, positionOffsetL):
        """Set position offset of left camera from body reference"""
        self.positionOffsetL = positionOffsetL


    def pollCapture(self):
        """Wait for and acknowledge next frame pair loaded into buffer"""
        if self.visualOdometryEvent.wait():
            self.pickupTime = self.captureTime[0]

            self.visualOdometryEvent.clear()

    
    def initializeState(self):
        """Initialize state estimates"""
        # Initialize deques to hold a running list of estimates
        self.rotationEstimateStack = deque(maxlen=self.bufferLength)
        self.positionEstimateStack = deque(maxlen=self.bufferLength)

        # Append to beginning of deque; newest first, oldest last
        self.rotationEstimateStack.appendleft(\
                    self.rotationOffsetL.copy().T)
        self.positionEstimateStack.appendleft(\
                    np.hstack((self.positionOffsetL, time.time())))

        # Initializing state buffers
        self.flushStateBuffers()
        self.writeStateStacksToBuffers()

        # Initialize state estimates
        self.R = self.rotationOffsetL.copy().T
        self.T = np.array([self.positionOffsetL.copy()]).T
        self.RT = np.hstack([self.R, self.T])
        self.RT = np.vstack([self.RT, np.zeros([1, 4])])
        self.RT[-1, -1] = 1

    
    def createExtractorORB(self):
        """Create an ORB feature extractor"""
        self.orb = cv2.ORB_create()

    
    def createMatcherBF(self):
        """Create a brute force matcher for matching ORB features 
        (cv2.NORM_HAMMING)"""
        self.bfMatcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
            

    def extractFeaturesORB(self):
        """Extract features from latest frame using the initialized ORB
        feature detector and append to stack"""
        keypoints = self.orb.detect(self.imageL, None)
        keypoints, descriptors = self.orb.compute(self.imageL, keypoints)

        # Append to end of deque; newest last, oldest first
        self.keypointStackL.append(keypoints)
        self.descriptorStackL.append(descriptors)

    
    def matchFeaturesORB(self, bestNMatches=150):
        """Match ORB features from stack"""
        matches = self.bfMatcher.match(self.descriptorStackL[0], \
                                    self.descriptorStackL[1])
        matches = sorted(matches, key=lambda x:x.distance)
        self.matches = matches[:bestNMatches]
    
    
    def estimateMotion(self):
        """Estimate motion between a pair of subsequent image frames"""
        imagePoints0 = []
        imagePoints1 = []

        for match in self.matches:
            train_idx = match.trainIdx
            query_idx = match.queryIdx

            p1x, p1y = self.keypointStackL[0][query_idx].pt 
            imagePoints0.append([p1x,p1y])

            p2x,p2y = self.keypointStackL[1][train_idx].pt 
            imagePoints1.append([p2x,p2y])

        imagePoints0 = np.array(imagePoints0)
        imagePoints1 = np.array(imagePoints1)

        essentialMatrix, mask = cv2.findEssentialMat(\
                        imagePoints0, imagePoints1, self.cameraMatrixL, \
                        cv2.RANSAC, 0.999, 1.0)

        retVal, self.rotationMatrix, self.translationVector, mask = \
                    cv2.recoverPose(essentialMatrix, \
                        imagePoints0, imagePoints1, self.cameraMatrixL, \
                        mask=mask)


    def updateStateEstimate(self):
        """Update state estimate with estimated motion"""
        pass


    def estimateStateRunning(self):
        """Keep a running estimate of the state"""
        if not self.isVisualOdometryPipelineReady():
            return
        
        self.initializeState()
        self.createExtractorORB()
        self.pollCapture()
        self.extractFeaturesORB()

        while not self.quitEvent.is_set():
            self.pollCapture()
            self.extractFeaturesORB()
            self.matchFeaturesORB()



if __name__=="__main__":
    print("Handled by ProcessManager")
