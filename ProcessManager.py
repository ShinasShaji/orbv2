from StereoCapture import StereoCapture
from ImageProcessing import ImageProcessing

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


    def runProcesses(self, context):
        """Start processes based on context"""
        # Capture pipeline is prepared regardless of context
        self.prepareCapturePipeline()

        self.context = context

        # Below are run based on context
        if context=="preview":
            # Communicating context to objects
            self.processing.selectContext(self.context)

            # Starting processes
            self.capture.start()
            self.processing.start()



if __name__=="__main__":
    processes = ProcessManager()
    processes.runProcesses("preview")