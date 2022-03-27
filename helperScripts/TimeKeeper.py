import time


class TimeKeeper:
    """Class to keep time of operations"""
    def __init__(self):
        pass

    
    def startPerfCounter(self):
        """Start an internal counter for evaluating performance. Stop with
        stopPerfCounter()"""
        self.perfCounter = [time.perf_counter(), 0]


    def returnPerfCounter(self):
        """Stop internal perf_counter and return elapsed time"""
        self.perfCounter[1] = time.perf_counter()

        return self.perfCounter[1]-self.perfCounter[0]

    def printPerfCounter(self):
        """Stop internal perf_counter and print elapsed time"""
        self.perfCounter[1] = time.perf_counter()

        print("Completed in {:.6f} seconds"\
                        .format(self.perfCounter[1]-self.perfCounter[0]))      

    def getElapsedTime(self):
        """Return elapsed time of the last session"""
        return self.perfCounter[1]-self.perfCounter[0]
  