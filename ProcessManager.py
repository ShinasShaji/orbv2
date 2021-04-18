from ImageProcessing import ImageProcessing
from StereoCapture import StereoCapture


class ProcessManager():
    def __init__(self):
        self.capture = StereoCapture()
        self.processing = ImageProcessing()


    def prepareCapturePipeline(self):
        self.capture.createBuffers()

        self.processing.referenceCaptureBuffers(\
            self.capture.getBuffers(), self.capture.getCVImageShape())

        self.processing.referenceCaptureEvents(\
            self.capture.getCaptureEvents())


    def runProcesses(self, context=None):
        """Start processes based on context"""
        assert context is not None, "No context provided"
        self.context = context

        # Capture pipeline is prepared regardless of context
        self.prepareCapturePipeline()

        # Below are run based on context
        if context=="preview":
            # Communicating context to objects
            self.processing.setContext(self.context)

            # Starting processes
            self.capture.start()
            self.processing.start()

            self.capture.join()
            self.processing.join()

        else:
            print("Invalid context")



if __name__=="__main__":
    processes = ProcessManager()
    processes.runProcesses("preview")
