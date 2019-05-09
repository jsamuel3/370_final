import socket
import sys
from ctypes import *
from getkey import getkey,keys


UDP_IP = "192.168.43.8"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.connect((UDP_IP, UDP_PORT))

class cmdPacket(Structure):
	_fields_ = [("velocity", c_double),("theta", c_double),("mode", c_int)]
	
class returnPacket(Structure):
    _fields_ = [("odo", c_double * 3), ("imu", c_double * 6), ("heading", c_double)]

# mode 0 = return x,y,head
# mode 1 = driving command
# mode 2 = cardinal command
# mode 3 = stop

while True:
	key = getkey()
	
	if ( keys.UP == key ) : #forward
		print("fwd")
		sock.send( cmdPacket( 10, 0, 1 ) )
		
	elif ( keys.DOWN == key ):
		print("back")
		sock.send( cmdPacket( 0, 0, 1 ) )
		
	elif ( keys.LEFT == key ):
		print("left")
		sock.send( cmdPacket( 0, -15, 1 ) )
		
	elif ( keys.RIGHT == key ):
		print("right")
		sock.send( cmdPacket( 0, 15, 1 ) )
		
	elif ( 'w' == key ):
		print("north")
		sock.send( cmdPacket( 0, 0, 2 ) )
		
	elif ( 's' == key ):
		print("south")
		sock.send( cmdPacket( 0, 180, 2 ) )
		
	elif ( 'a' == key ):
		print("west")
		sock.send( cmdPacket( 0, 270, 2 ) )
		
	elif ( 'd' == key ):
		print("east")
		sock.send( cmdPacket( 0, 90, 2 ) )
			
	elif ( ' ' == key ):
                print("report")
                sock.send(cmdPacket(0, 0, 0))
                buffer = sock.recv(sizeof(returnPacket))
                tempPacket = returnPacket.from_buffer_copy(buffer)
                print(tempPacket.odo[0], tempPacket.odo[1], tempPacket.odo[2])
                print(tempPacket.imu[0], tempPacket.imu[1], tempPacket.imu[2], tempPacket.imu[3], tempPacket.imu[4], tempPacket.imu[5])
                print(tempPacket.heading)
	else:
		print("unrecognized")
