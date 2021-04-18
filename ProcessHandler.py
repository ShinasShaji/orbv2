from StereoCapture import StereoCapture
from ImageProcessing import ImageProcessing

class ProcessHandler():
    def __init__(self):
        self.capture = StereoCapture()
        self.processing = ImageProcessing()

    def preview(self):
        """Run processes for preview"""
        self.capture.createBuffers()

        self.processing.referenceCaptureBuffers(\
            self.capture.getBuffers(), self.capture.getCVImageShape())

        self.processing.referenceCaptureEvents(\
            self.capture.getCaptureEvents())

        self.capture.start()
        self.processing.start()



if __name__=="__main__":
    processes = ProcessHandler()
    processes.preview()