from ImageProcessor import ImageProcessor
from StereoCapture import StereoCapture
from VisualOdometry import VisualOdometry


class ProcessManager():
    def __init__(self):
        self.stereoCapture = StereoCapture()
        self.imageProcessor = ImageProcessor()
        self.visualOdometry = VisualOdometry()

        self.possibleContexts = ["preview", "previewDisparity", \
            "previewUndistort", "assistedVoxel"]


    def prepareCapturePipeline(self):
        """References bufers and events required for capture"""
        self.stereoCapture.createBuffers()

        self.imageProcessor.referenceCaptureBuffers(\
            self.stereoCapture.getBuffers(), \
            self.stereoCapture.getCVImageShape())

        self.imageProcessor.referenceCaptureEvents(\
            self.stereoCapture.getImageProcessorEvents())

        
    def prepareVisualOdometryPipeline(self):
        """References bufers and events required for visual odometry"""
        self.visualOdometry.referenceCaptureBuffers(\
            self.stereoCapture.getBuffers(), \
            self.stereoCapture.getCVImageShape())

        self.visualOdometry.referenceCaptureEvents(\
            self.stereoCapture.getVisualOdometryEvents())
        
        self.visualOdometry.setCameraMatrixL(\
            self.imageProcessor.getCameraMatrixL())

        self.imageProcessor.stereoMatcher.voxelGrid.referenceStateBuffers(\
            self.visualOdometry.getStateBuffers(), \
            self.visualOdometry.getBufferLength())

        self.imageProcessor.stereoMatcher.voxelGrid.\
            referenceStateEvent(\
            self.visualOdometry.getStateEvent())


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

        elif self.context=="assistedVoxel":
            """Voxel mapping assisted by Visual Odometry"""
            # Prepare visual odometry pipeline
            self.prepareVisualOdometryPipeline()

            # Communicating context to objects
            self.imageProcessor.setContext(self.context)

            # Starting processes
            self.stereoCapture.start()
            self.imageProcessor.start()
            self.visualOdometry.start()

            self.stereoCapture.join()
            self.imageProcessor.join()
            self.visualOdometry.join()



if __name__=="__main__":
    processManager = ProcessManager()
    processManager.runProcesses("previewDisparity")
