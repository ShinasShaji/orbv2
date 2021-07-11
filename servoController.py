import time

from pyPS4Controller.controller import Controller


class ServoController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.interval = 0.2

        self.L3State = [0,0]
        self.prevL3Time = time.perf_counter()
        # Analog controls going to rest should override timer
        self.L3Override = False


    def on_L3_left(self, value):
        self.updateState("L3", direction="left", value=value)


    def on_L3_right(self, value):
        self.updateState("L3", direction="right", value=value)

    
    def on_L3_x_at_rest(self):
        self.updateState("L3", direction="left", value=0)
        self.L3Override = True
        
    
    def on_L3_up(self, value):
        self.updateState("L3", direction="up", value=value)

    
    def on_L3_down(self, value):
        self.updateState("L3", direction="down", value=value)


    def on_L3_y_at_rest(self):
        self.updateState("L3", direction="up", value=0)
        self.L3Override = True


    def updateState(self, control, direction=None, value=None):
        """Update state with current values"""
        if control == "L3":
            if direction in ["up", "down"]:
                self.L3State[1] = value
            elif direction in ["left", "right"]:
                self.L3State[0] = value
            
            if self.L3State == [0,0]:
                self.L3Override = True

            self.printState("L3")


    def printState(self, control):
        """Print controller state at defined intervals"""
        currentTime = time.perf_counter()
        if currentTime-self.prevL3Time > self.interval or self.L3Override:
            if control == "L3":
                print("L3", self.L3State)
            
            self.prevL3Time = currentTime
            self.L3Override = False



if __name__ == "__main__":
    servoController = ServoController(interface="/dev/input/js0", \
        connecting_using_ds4drv=False)
    servoController.listen()
