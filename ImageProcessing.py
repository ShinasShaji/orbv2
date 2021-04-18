import multiprocessing
import os

import cv2
import numpy


class ImageProcessing(multiprocessing.Process):
    """Class to handle image processing on capture stream"""
    def __init__(self):
        super(ImageProcessing, self).__init__()

        # Capture details
        self.imageCounter = 0
        self.capturePath = "captures/"

        # Flags
        self.captureBufferReady = False
        self.captureEventReady = False


    def referenceCaptureBuffers(self, buffers, cvImageShape):
        """Create class references to passed capture buffers and 
        shape buffers into image arrays"""
        # Creating class references to buffers
        assert buffers is not None and cvImageShape is not None, \
                                            "Initialize capture buffers"
        self.leftImageBuffer = buffers[0]
        self.rightImageBuffer = buffers[1]
        self.cvImageShape = cvImageShape

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


    def selectContext(self, context):
        """Selects which methods to run on start() based on context"""
        self.context = context


    def run(self):
        """Runs when start() is called on this process"""
        if self.context=="preview":
            self.capturePreview()



if __name__=="__main__":
    print("Import to use")
