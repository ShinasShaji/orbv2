import os
import threading
import time

from pyPS4Controller.controller import Controller

from helperScripts.Arduino import Arduino


class DS4(Controller):
    def __init__(self, serialOutput = False, **kwargs):
        Controller.__init__(self, **kwargs)
        self.INTERVAL = 0.01
        self.MAXVALUE = 32767
        
        self.currentTime = time.perf_counter()

        # Print control state on change
        self.verbose = True

        # State array; L3(x2), R3(x2), L2, R2
        self.state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
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
        
    
    def on_L3_up(self, value):
        self.updateState("L3", direction="up", value=value)

    
    def on_L3_down(self, value):
        self.updateState("L3", direction="down", value=value)


    def on_L3_y_at_rest(self):
        self.updateState("L3", direction="up", value=0)


    # R3
    def on_R3_left(self, value):
        self.updateState("R3", direction="left", value=value)


    def on_R3_right(self, value):
        self.updateState("R3", direction="right", value=value)

    
    def on_R3_x_at_rest(self):
        self.updateState("R3", direction="left", value=0)
        
    
    def on_R3_up(self, value):
        self.updateState("R3", direction="up", value=value)

    
    def on_R3_down(self, value):
        self.updateState("R3", direction="down", value=value)


    def on_R3_y_at_rest(self):
        self.updateState("R3", direction="up", value=0)


    # Triggers, L2 and R2
    def on_L2_press(self, value):
        self.updateState("L2", value=value+self.MAXVALUE)

    
    def on_L2_release(self):
        self.updateState("L2", value=0)

    
    def on_R2_press(self, value):
        self.updateState("R2", value=value+self.MAXVALUE)

    
    def on_R2_release(self):
        self.updateState("R2", value=0)


    # State management
    def updateState(self, control, direction=None, value=None):
        """Update state with current values"""
        if control == "L3":
            if direction in ["up", "down"]:
                self.state[1] = value/self.MAXVALUE
            elif direction in ["left", "right"]:
                self.state[0] = value/self.MAXVALUE

            self.printState("L3")

        elif control == "R3":
            if direction in ["up", "down"]:
                self.state[3] = value/self.MAXVALUE
            elif direction in ["left", "right"]:
                self.state[2] = value/self.MAXVALUE

            self.printState("R3")

        elif control in ["L2", "R2"]:
            if control == "L2":
                self.state[4] = value/(2*self.MAXVALUE)
            elif control == "R2":
                self.state[5] = value/(2*self.MAXVALUE)

            self.printState("Trigger")


    def printState(self, control):
        """Print controller state"""  
        if self.verbose: 
            if control == "L3":
                print("L3", self.state[:2])
            
            elif control == "R3":
                print("R3", self.state[2:4])
            
            elif control == "Trigger":
                print("Trigger", self.state[4:])
            

    def writeStateToSerial(self):
        """Write current state of controller inputs to serial"""
        try:
            while self.arduino.connected:
                self.currentTime = time.perf_counter()
                timeElapsed = self.currentTime - self.prevTxTime

                if timeElapsed > self.txInterval:
                    content = "ds4 {:.1f} {:.1f} {:.1f} {:.1f} {:.1f} {:.1f}".format(\
                            self.state[0], self.state[1], \
                            self.state[2], self.state[3], \
                            self.state[4], self.state[5])

                    self.arduino.writeToSerial(content)

                    self.prevTxTime = self.currentTime
                
                else:
                    time.sleep(self.txInterval - timeElapsed)

        except KeyboardInterrupt:
            self.arduino.closeConnection()



if __name__ == "__main__":
    if os.name == "posix":
        ds4 = DS4(serialOutput=True, interface="/dev/input/js0", connecting_using_ds4drv=False)

    else:
        print("The script does not support this platform")
