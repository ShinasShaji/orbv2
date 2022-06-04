import os
import sys
import threading
import time

import numpy as np
from pyPS4Controller.controller import Controller

from helperScripts.Arduino import Arduino


class DS4(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.MAXVALUE = 32767
        self.SCALEDMIDVALUE = 50.0
        
        self.currentTime = time.perf_counter()
        
        self.exitFlag = False

        # Print control state on change
        self.verbose = False

        # State array; L3(x2), R3(x2), L2, R2, Square
        self.state = [self.SCALEDMIDVALUE, self.SCALEDMIDVALUE, \
                      self.SCALEDMIDVALUE, self.SCALEDMIDVALUE, \
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


    # L3
    def on_L3_left(self, value):
        self.updateState("L3", direction="left", value=value+self.MAXVALUE)


    def on_L3_right(self, value):
        self.updateState("L3", direction="right", value=value+self.MAXVALUE)

    
    def on_L3_x_at_rest(self):
        self.updateState("L3", direction="left", value=self.MAXVALUE)
        
    
    def on_L3_up(self, value):
        self.updateState("L3", direction="up", value=value+self.MAXVALUE)

    
    def on_L3_down(self, value):
        self.updateState("L3", direction="down", value=value+self.MAXVALUE)


    def on_L3_y_at_rest(self):
        self.updateState("L3", direction="up", value=self.MAXVALUE)


    # R3
    def on_R3_left(self, value):
        self.updateState("R3", direction="left", value=value+self.MAXVALUE)


    def on_R3_right(self, value):
        self.updateState("R3", direction="right", value=value+self.MAXVALUE)

    
    def on_R3_x_at_rest(self):
        self.updateState("R3", direction="left", value=self.MAXVALUE)
        
    
    def on_R3_up(self, value):
        self.updateState("R3", direction="up", value=value+self.MAXVALUE)

    
    def on_R3_down(self, value):
        self.updateState("R3", direction="down", value=value+self.MAXVALUE)


    def on_R3_y_at_rest(self):
        self.updateState("R3", direction="up", value=self.MAXVALUE)


    # Triggers, L2 and R2
    def on_L2_press(self, value):
        self.updateState("L2", value=value+self.MAXVALUE)

    
    def on_L2_release(self):
        self.updateState("L2", value=0)

    
    def on_R2_press(self, value):
        self.updateState("R2", value=value+self.MAXVALUE)

    
    def on_R2_release(self):
        self.updateState("R2", value=0)

    
    def on_square_press(self):
        self.updateState("Square", value = 1)


    def on_square_release(self):
        self.updateState("Square", value = 0)


    def on_x_press(self):
        self.updateState("Cross", value = 1)

    
    def on_x_release(self):
        self.updateState("Cross", value = 0)


    def on_triangle_press(self):
        self.updateState("Triangle", value = 1)

    
    def on_triangle_release(self):
        self.updateState("Triangle", value = 0)


    def on_options_press(self):
        """Exit on options press"""
        self.updateState("Options", value = 1)

        time.sleep(self.txInterval)
        self.exitFlag = True

        exit()


    # State management
    def updateState(self, control, direction=None, value=None):
        """Update state with current values"""
        if control == "L3":
            value = value * self.SCALEDMIDVALUE

            if direction in ["up", "down"]:
                self.state[1] = value/self.MAXVALUE
            elif direction in ["left", "right"]:
                self.state[0] = value/self.MAXVALUE

            self.printState("L3")

        elif control == "R3":
            value = value * self.SCALEDMIDVALUE

            if direction in ["up", "down"]:
                self.state[3] = value/self.MAXVALUE
            elif direction in ["left", "right"]:
                self.state[2] = value/self.MAXVALUE

            self.printState("R3")

        elif control in ["L2", "R2"]:
            value = value * self.SCALEDMIDVALUE

            if control == "L2":
                self.state[4] = value/(2*self.MAXVALUE)
            elif control == "R2":
                self.state[5] = value/(2*self.MAXVALUE)

            self.printState("Trigger")

        elif control == "Square":
            self.state[6] = value

            self.printState("Square")

        elif control == "Cross":
            self.state[7] = value

            self.printState("Cross")

        elif control == "Triangle":
            self.state[8] = value

            self.printState("Triangle")

        elif control == "Options":
            self.state[9] = value

            self.printState("Options")


    def printState(self, control):
        """Print controller state"""  
        if self.verbose: 
            if control == "L3":
                print("L3", self.state[:2])
            
            elif control == "R3":
                print("R3", self.state[2:4])
            
            elif control == "Trigger":
                print("Trigger", self.state[4:6])

            elif control == "Square":
                print("Square", self.state[6])

            elif control == "Cross":
                print("Cross", self.state[7])

            elif control == "Triangle":
                print("Triangle", self.state[8])

            elif control == "Options":
                print("Options", self.state[9])


    def extractStates(self, message):
        """Extracts servo states from received serial message"""
        message = str(message).split()

        # Servo states
        if "s" in message:
            index = message.index("s")
            try:
                self.servoState = np.array([message[index+1], message[index+2], \
                                            message[index+3], message[index+4]], \
                                            dtype=np.int16)
            except:
                self.servoState = None

        # Kinematics states
        if "k" in message:
            index = message.index("k")
            try:
                self.kinematicsState = np.array([message[index+1], message[index+2], \
                                            message[index+3], message[index+4]], \
                                            dtype=np.int16)
            except:
                self.kinematicsState = None
            

    def controllerSerial(self):
        """Writes current controller state to serial. Run as thread"""
        while self.arduino.connected:
            self.currentTime = time.perf_counter()
            timeElapsed = self.currentTime - self.prevTxTime

            if timeElapsed > self.txInterval:
                content = "d {:.0f} {:.0f} {:.0f} {:.0f} {:.0f} {:.0f} {:.0f} {:.0f} {:.0f} {:.0f}".format(\
                        self.state[0], self.state[1], \
                        self.state[2], self.state[3], \
                        self.state[4], self.state[5], \
                        self.state[6], self.state[7],
                        self.state[8], self.state[9])

                self.arduino.writeToSerial(content)

                self.extractStates(self.arduino.readFromSerial())

                self.prevTxTime = self.currentTime
                
            else:
                time.sleep(self.txInterval - timeElapsed)

            if self.exitFlag:
                self.arduino.closeConnection()


    def setContext(self, context):
        """Selects which methods to run on run() based on context"""
        self.context = context

    
    def run(self):
        """Run methods based on set context. Not a thread/process; call run()"""
        if self.context == "preview":
            self.verbose = True
            self.listen()
    
        elif self.context == "control":
            # Connect to Arduino
            self.arduino = Arduino()
            self.arduino.attemptConnection()
            self.prevTxTime = self.currentTime
            self.txInterval = 0.1                                         

            self.servoState = None
            self.kinematicsState = None

            # Create thread to periodically write state to serial
            self.controllerSerialThread = threading.Thread(target=self.controllerSerial)
            self.controllerSerialThread.start()

            self.listen()
                


if __name__ == "__main__":
    if os.name == "posix":
        ds4 = DS4(interface="/dev/input/js0", connecting_using_ds4drv=False)
        ds4.setContext("control")

        try:
            ds4.run()

        except KeyboardInterrupt:
            print("Closing down...")
            sys.exit()

    else:
        print("This platform is unsupported")
