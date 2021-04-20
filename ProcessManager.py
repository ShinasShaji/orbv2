from ImageProcessing import ImageProcessing
from StereoCapture import StereoCapture


class ProcessManager():
    def __init__(self):
        self.stereoCapture = StereoCapture()
        self.imageProcessing = ImageProcessing()


    def prepareCapturePipeline(self):
        """References bufers and events required for capture"""
        self.stereoCapture.createBuffers()

        self.imageProcessing.referenceCaptureBuffers(\
            self.stereoCapture.getBuffers(), \
            self.stereoCapture.getCVImageShape())

        self.imageProcessing.referenceCaptureEvents(\
            self.stereoCapture.getCaptureEvents())


    def runProcesses(self, context=None):
        """Start processes based on context"""
        assert context is not None, "No context provided"
        self.context = context

        # Capture pipeline is prepared regardless of context
        self.prepareCapturePipeline()

        # Below are run based on context
        if self.context=="preview":
            # Communicating context to objects
            self.imageProcessing.setContext(self.context)

            # Starting processes
            self.stereoCapture.start()
            self.imageProcessing.start()

            self.stereoCapture.join()
            self.imageProcessing.join()

        if self.context=="previewDisparity":
            # Communicating context to objects
            self.imageProcessing.setContext(self.context)

            # Starting processes
            self.stereoCapture.start()
            self.imageProcessing.start()

            self.stereoCapture.join()
            self.imageProcessing.join()

        else:
            print("Invalid context")



if __name__=="__main__":
    processManager = ProcessManager()
    processManager.runProcesses("previewDisparity")
