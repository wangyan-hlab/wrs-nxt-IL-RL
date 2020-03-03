import serial
ser = serial.Serial(port="COM8", baudrate=115200, timeout=1)
# ser.write("ME\r\n")
# print ser.read()
# ser.write("RV\r\n")
# print ser.read()