import serial

class Serial:
    def __init__(self):
        self.ser = None

    def set(self, commPort):
        self.ser = serial.Serial(commPort, 9600, timeout = 1)
        print("port set")

    def reset(self):
        self.ser = None

    def getOutput(self):
        print(self.ser)
        return self.ser.read_until(expected = B'END') if self.ser else None

    def linActMotor(self, direction):
        if self.ser:
            if direction == "forward":
                self.ser.write(b'LF')
            elif direction == "backward":
                self.ser.write(b'LR')
            else:
                self.ser.write(b'LO')

    def drillMotor(self, direction):
        if self.ser:
            if direction == "forward":
                self.ser.write(b'DF')
            elif direction == "backward":
                self.ser.write(b'DR')
            else:
                self.ser.write(b'DO')

