# Echo client program
import socket

HOST = "10.2.0.50" # The UR IP address
PORT = 30002 # UR secondary client
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

f = open ("peginholespiral.script", "rb")   #Robotiq Gripper
# f = open ("tst.script", "rb")  #Robotiq FT sensor

l = f.read(1024)
while (l):
    s.send(l)
    l = f.read(1024)
s.send("\n")
s.close()