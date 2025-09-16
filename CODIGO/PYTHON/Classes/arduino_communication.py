import serial
import numpy as np

class ArduinoCommunication:
    def __init__(self):
        self.arduino = serial.Serial(port='COM6', baudrate=31250, timeout=.1)

    def closeCommunication(self):
        self.arduino.close()

    def write(self, data):
        self.arduino.write(data.encode('utf-8'))
        self.arduino.flush()

    def read(self):
        data = self.arduino.readline()
        data = data.decode('utf-8')[:-1]
        response = np.array(data.split())
        return response

    def createCommand(self, vector):
        command = ""
        delimitator = ","
        for i in range(0, len(vector)):
            command += str(vector[i]) + delimitator    
        return command

    def sendCommand(self, vector):
        command = self.createCommand(vector) + "\n"
        print(command)
        self.write(command)
