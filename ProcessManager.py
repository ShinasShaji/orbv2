from ImageProcessor import ImageProcessor
from StereoCapture import StereoCapture


class ProcessManager():
    def __init__(self):
        self.stereoCapture = StereoCapture()
        self.imageProcessor = ImageProcessor()

        self.possibleContexts = ["preview", "previewDisparity", \
            "previewUndistort"]


    def prepareCapturePipeline(self):
        """References bufers and events required for capture"""
        self.stereoCapture.createBuffers()

        self.imageProcessor.referenceCaptureBuffers(\
            self.stereoCapture.getBuffers(), \
            self.stereoCapture.getCVImageShape())

        self.imageProcessor.referenceCaptureEvents(\
            self.stereoCapture.getCaptureEvents())


    def runProcesses(self, context=None):
        """Start processes based on context"""
        assert context is not None, "No context provided"
        
        if context in self.possibleContexts:
            self.context = context

        else:
            print("Invalid context")
            return

        # Capture pipeline is prepared regardless of context
        self.prepareCapturePipeline()

        # Below are run based on context
        if self.context=="preview":
            # Communicating context to objects
            self.imageProcessor.setContext(self.context)

            # Starting processes
            self.stereoCapture.start()
            self.imageProcessor.start()

            self.stereoCapture.join()
            self.imageProcessor.join()

        elif self.context=="previewDisparity":
            # Communicating context to objects
            self.imageProcessor.setContext(self.context)

            # Starting processes
            self.stereoCapture.start()
            self.imageProcessor.start()

            self.stereoCapture.join()
            self.imageProcessor.join()

        elif self.context=="previewUndistort":
            # Communicating context to objects
            self.imageProcessor.setContext(self.context)

            # Starting processes
            self.stereoCapture.start()
            self.imageProcessor.start()

            self.stereoCapture.join()
            self.imageProcessor.join()



if __name__=="__main__":
    processManager = ProcessManager()
    processManager.runProcesses("previewDisparity")
