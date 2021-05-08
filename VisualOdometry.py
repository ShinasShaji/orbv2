import multiprocessing

import cv2
import numpy as np

from helperScripts.TimeKeeper import TimeKeeper


class VisualOdometry(multiprocessing.Process):
    """Class to handle visual odometry routines"""
    def __init__(self):
        super(VisualOdometry, self).__init__()
        
        # Debug
        self.verbose = True

        # Capture link
        self.captureBufferReady = False
        self.captureEventReady = False


    ### Methods to link up with capture process
        
    def referenceCaptureBuffers(self, buffers):
        """Create class references to passed capture buffers"""

        assert buffers is not None, "Initialize capture buffers"

        self.leftImageBuffer = buffers[0]
        self.rightImageBuffer = buffers[1]
        self.captureTimeBuffer = buffers[2]

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