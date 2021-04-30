import os
import serial,glob,time
import serial.tools.list_ports

def StartSerialPort():
    #clear screen
    os.system('cls' if os.name == 'nt' else 'clear')

    ser = serial.Serial()
    ser.timeout = 3

    print('Searching for Serial Ports...')
    ports = serial.tools.list_ports.comports(include_links=False)
    for port in ports :
        print('Found '+ port.device)

    for port in ports:
        if "usb" in port.device and port.device != "/dev/ttyUSB0" :
            ser.port = port.device
            break
    ser.port = "/dev/ttyUSB0"
    
    ser.open()
    return ser







class Motor(): 
    def __init__(self,serial,Name):
        self.ser = serial
        self.MotorName = Name
        
    def readEncoder(self):
        message = "{Name},ENCODER_REQUEST,".format(self.MotorName)
        self.ser.write(message.encode('UTF-8'))
        line = self.ser.readline()
        result = ...
        return -result


    def resetEncoder(self):
        message = "{Name},RESET_ENCODER,".format(self.MotorName)
        self.ser.write(message.encode('UTF-8'))
    
    def setTargetPos(self,pos):
        message = "{Name},SET_POS,{pos};".format(Name=self.MotorName,pos=-pos)
        self.ser.write(message.encode('UTF-8'))
        


    