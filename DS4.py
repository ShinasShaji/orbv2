import threading
import time

from pyPS4Controller.controller import Controller

from helperScripts.Arduino import Arduino


class DS4(Controller):
    def __init__(self, serialOutput = False, **kwargs):
        Controller.__init__(self, **kwargs)
        self.INTERVAL = 0.01
        self.MAXVALUE = 32767
        self.INITSTATE = [0.0, 0.0]
        
        self.currentTime = time.perf_counter()

        # L3
        self.L3State = self.INITSTATE
        self.prevL3Time = self.currentTime
        self.L3Override = False

        # R3
        self.R3State = self.INITSTATE
        self.prevR3Time = self.currentTime
        self.R3Override = False

        # Triggers, L2 and R2
        self.triggerState = self.INITSTATE
        self.prevTriggerTime = self.currentTime
        self.triggerOverride = False

        self.serialOutput = serialOutput
        if self.serialOutput:
            # Connect to Arduino
            self.arduino = Arduino()
            self.arduino.attemptConnection()
            self.prevTxTime = self.currentTime
            self.txInterval = 0.1

            # Create thread to periodically write state to serial
            self.serialWriteThread = threading.Thread(target=self.writeStateToSerial)
            self.serialWriteThread.start()

        # Starting listener
        self.listen()


    # L3
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
        self.R3Override = True


    # R3
    def on_R3_left(self, value):
        self.updateState("R3", direction="left", value=value)


    def on_R3_right(self, value):
        self.updateState("R3", direction="right", value=value)

    
    def on_R3_x_at_rest(self):
        self.updateState("R3", direction="left", value=0)
        self.L3Override = True
        
    
    def on_R3_up(self, value):
        self.updateState("R3", direction="up", value=value)

    
    def on_R3_down(self, value):
        self.updateState("R3", direction="down", value=value)


    def on_R3_y_at_rest(self):
        self.updateState("R3", direction="up", value=0)
        self.R3Override = True


    # Triggers, L2 and R2
    def on_L2_press(self, value):
        self.updateState("L2", value=value+self.MAXVALUE)

    
    def on_L2_release(self):
        self.updateState("L2", value=0)
        self.triggerOverride = True

    
    def on_R2_press(self, value):
        self.updateState("R2", value=value+self.MAXVALUE)

    
    def on_R2_release(self):
        self.updateState("R2", value=0)
        self.triggerOverride = True


    # State management
    def updateState(self, control, direction=None, value=None):
        """Update state with current values"""
        if control == "L3":
            if direction in ["up", "down"]:
                self.L3State[1] = value/self.MAXVALUE
            elif direction in ["left", "right"]:
                self.L3State[0] = value/self.MAXVALUE
            
            if self.L3State == self.INITSTATE:
                self.L3Override = True

            self.printState("L3")

        elif control == "R3":
            if direction in ["up", "down"]:
                self.R3State[1] = value/self.MAXVALUE
            elif direction in ["left", "right"]:
                self.R3State[0] = value/self.MAXVALUE
            
            if self.R3State == self.INITSTATE:
                self.R3Override = True

            self.printState("R3")

        elif control in ["L2", "R2"]:
            if control == "L2":
                self.triggerState[0] = value/(2*self.MAXVALUE)
            elif control == "R2":
                self.triggerState[1] = value/(2*self.MAXVALUE)
            
            if self.triggerState == self.INITSTATE:
                self.triggerOverride = True

            self.printState("Trigger")


    def printState(self, control):
        """Print controller state at defined intervals"""
        self.currentTime = time.perf_counter()
        
        if control == "L3" and (self.currentTime-self.prevL3Time > self.INTERVAL or self.L3Override):
            print("L3", self.L3State)
            
            self.prevL3Time = self.currentTime
            self.L3Override = False

        elif control == "R3" and (self.currentTime-self.prevR3Time > self.INTERVAL or self.R3Override):
            print("R3", self.R3State)
            
            self.prevR3Time = self.currentTime
            self.R3Override = False

        elif control == "Trigger" and (self.currentTime-self.prevR3Time > self.INTERVAL or self.R3Override):
            print("Trigger", self.triggerState)
            
            self.prevTriggerTime = self.currentTime
            self.triggerOverride = False


    def writeStateToSerial(self):
        """Write current state of controller inputs to serial"""
        try:
            while self.arduino.connected:
                currentTime = time.perf_counter()
                timeElapsed = currentTime - self.prevTxTime

                if timeElapsed > self.txInterval:
                    content = "{:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f}".format(\
                            self.L3State[0],        self.L3State[1], \
                            self.R3State[0],        self.R3State[1], \
                            self.triggerState[0],   self.triggerState[1])
                
                    self.arduino.writeToSerial(content)
                    self.prevTxTime = currentTime
                
                else:
                    time.sleep(self.txInterval - timeElapsed)

        except KeyboardInterrupt:
            self.arduino.closeConnection()



if __name__ == "__main__":
    ds4 = DS4(serialOutput = False, interface="/dev/input/js0", connecting_using_ds4drv=False)
