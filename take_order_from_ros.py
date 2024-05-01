#!/usr/bin/env python3

import serial
import time
import rospy
from std_msgs.msg import String 

order=0

# Configure the serial port
SerialObj = serial.Serial('/dev/ttyUSB0')  	 # Change 'COM8' to match your device
SerialObj.baudrate = 115200           	 # Set Baud rate to 9600
SerialObj.bytesize = 8             		 # Number of data bits = 8
SerialObj.parity = 'N'              		 # No parity
SerialObj.stopbits = 1              		 # Number of Stop bits = 1

		
def take_order_callback(msg):
	if msg.data == 'sure':
		print(msg.data)
		order=1 		# Send the object 
	elif msg.data == 'here you are':
		print(msg.data)
		order=2 		# deliver the object 
			
	# Create a bytes object containing the byte to send
	byte_to_send = bytes([order])

	# Write the byte to the serial port
	BytesWritten = SerialObj.write(byte_to_send)

	# Print the number of bytes written
	print('BytesWritten = ', BytesWritten)# Print the number of bytes written
	
	


rospy.init_node('sub', anonymous=True)

rospy.Subscriber('chatbot_response', String, take_order_callback)



#face = int(input())  # Example value for the byte
rospy.spin()
SerialObj.close()  # Close the serial port
