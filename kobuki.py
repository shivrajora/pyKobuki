#!/usr/bin/env pythoni
# coding:utf-8
#
# kobuki.py

from serial import Serial
import numpy as np
import cv2
import time
import csv
import os
from datetime import datetime
from primesense import openni2#, niite2
from primesense import _openni2 as c_api

# Path to OpenNI redistribution OpenNI2.so
dist = '/usr/lib/'

class kobuki :
	# initialize Kobuki. OpenCV and 3D camera
	def __init__(self, dev_path):
		#initialize OpenNI2
		openni2.initialize(dist) #
		if (openni2.is_initialized()):
			print "openNI2 initialized"
		else:
			print "openNI2 not initialized"

		## Register the device
		self.dev = openni2.Device.open_any()

		## Create the streams stream
		self.rgb_stream = self.dev.create_color_stream()
		self.depth_stream = self.dev.create_depth_stream()

		## Configure the depth_stream -- changes automatically based on bus speed
		#print 'Depth video mode info', depth_stream.get_video_mode() # Checks depth video configuration
		self.depth_stream.set_video_mode(c_api.OniVideoMode(pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM, resolutionX=320, resolutionY=240, fps=30))

		## Check and configure the mirroring -- default is True
		## Note: I enabled mirroring
		# print 'Mirroring info1', depth_stream.get_mirroring_enabled()
		#depth_stream.set_mirroring_enabled(False)
		#rgb_stream.set_mirroring_enabled(False)

		## Start the streams
		self.rgb_stream.start()
		self.depth_stream.start()

		## Synchronize the streams
		self.dev.set_depth_color_sync_enabled(True) # synchronize the streams

		## IMPORTANT: ALIGN DEPTH2RGB (depth wrapped to match rgb stream)
		self.dev.set_image_registration_mode(openni2.IMAGE_REGISTRATION_DEPTH_TO_COLOR)
		print("ASUS Xtion Pro Initialized")

		#initialize Serial communication
		self.serial = Serial(dev_path, 115200)
		print("Serial connection to Kobuki initialized")
		
		# Set save path here
		self.save_path = 'data/'

		#Initialize CSV file
		self.csv_file = open(self.save_path+'data.csv', "wb")
		self.csv_writer = csv.writer(self.csv_file, delimiter=' ', quotechar='|', quoting=csv.QUOTE_NONE)

		# throttle and steering variables
		self.thr = 0
                self.steer = 50

	def get_rgb(self):
		"""
		Returns numpy 3L ndarray to represent the rgb image.
		"""
		self.bgr   = np.fromstring(self.rgb_stream.read_frame().get_buffer_as_uint8(),dtype=np.uint8).reshape(240,320,3)
		self.rgb   = cv2.cvtColor(self.bgr,cv2.COLOR_BGR2RGB)
		return self.rgb

	def get_depth(self):
		"""
		Returns numpy ndarrays representing the raw and ranged depth images.
		Outputs:
		dmap:= distancemap in mm, 1L ndarray, dtype=uint16, min=0, max=2**12-1
		d4d := depth for dislay, 3L ndarray, dtype=uint8, min=0, max=255    
		Note1: 
			fromstring is faster than asarray or frombuffer
		Note2:     
			.reshape(120,160) #smaller image for faster response 
			OMAP/ARM default video configuration
			.reshape(240,320) # Used to MATCH RGB Image (OMAP/ARM)
			Requires .set_video_mode
		"""
		self.dmap = np.fromstring(self.depth_stream.read_frame().get_buffer_as_uint16(),dtype=np.uint16).reshape(240,320)  # Works & It's FAST
		self.d4d = np.uint8(self.dmap.astype(float) *255/ 2**12-1) # Correct the range. Depth images are 12bits
		self.d4d = 255 - cv2.cvtColor(self.d4d,cv2.COLOR_GRAY2RGB)
		return self.dmap, self.d4d

	# send a command to Kobuki
	def send(self, commands) :
		sub_payloads = commands
		payload = []
		for sub_payload in sub_payloads :
			payload.append(sub_payload)
		header = [0xAA, 0x55]
		
		body = [len(payload)] + payload

		checksum = 0
		for x in body:
			checksum ^= x
				
		packets = header+body+[checksum]
		#print(packets)
		self.serial.write(''.join(map(chr, packets)))
	
	# destructor
	def __del__(self):
		self.stop()
		self.serial.close()
		print("Serial connection to Kobuki closed")

		## Release resources 
		self.csv_file.close()
		cv2.destroyAllWindows()
		print("OpenCV terminated")		

		self.rgb_stream.stop()
		self.depth_stream.stop()
		openni2.unload()
		print ("OpenNI Terminated")

	# Prepare byte string for speed and turn radius of Kobuki
        def base_control(self, speed, radius) :
                speed_lsb = 0xff & speed
                speed_msb = 0xff & (speed>>8)
                radius_lsb = 0xff & radius
                radius_msb = 0xff & (radius>>8)

                return [0x01, 0x04, speed_lsb, speed_msb, radius_lsb, radius_msb]

	# Stop the Kobuki
	def stop(self):
		self.send(self.base_control(0,0))

	# Map throttle values to values input by Kobuki
	'''
	def drive(self, thr, steer) :
		#map throttle (0-100) to speed value
		if(thr>100) :
			print("Throttle limit exceeded")
			thr = 100
		if(thr<0) :
			print("Throttle value out of bounds, setting to zero")
			thr = 0
		
		speed = thr*3;
		
		#map steering (0-100)i
		if(steer>100) :
                        print("Steering limit exceeded, setting to 100%")
                        radius = 100
                if(steer<0) :
                        print("Steer value out of bounds, setting to zero")
                        steer = 0

		radius = -300+steer*6

		#print("throttle : ", thr, "steer : ", steer)
		
		#drive robot
		self.send(self.base_control(speed,radius))

	'''
	
	def drive(self, thr, steer) :
		if (thr == 1) :
			speed = 300

		elif (thr == 0) :
			speed = 0

		else :
			print("Wrong throttle value, setting to zero")
			speed = 0


		if (steer == 0.5) :
			radius = 0

		elif (steer == 0) :
			radius = 300

		elif (steer == 1)
			radius = -300

		else :
			print("Wrong steering values, setting to 0.5")

		self.send(self.base_control(speed,radius))
	

	# Save images+steering+throttle data
	def save_data(self, steer, thr) :
		# Get RGB + Depth images
		rgb = self.get_rgb()
                _,d4d = self.get_depth()

		# Prepare strings for save paths
		time_str = self.save_path+datetime.now().strftime("%Y_%m_%d_%H_%M_%S_")+str(time.time()).replace(".","_")
		rgb_str = time_str+"_rgb.jpg"
		depth_str = time_str+"_depth.jpg"
	
		# Write images
		cv2.imwrite(rgb_str, rgb)
		cv2.imwrite(depth_str, d4d)

		# Write throttle+steer into CSV
		steer_left = (steer == 0)
		steer_right = (steer == 1)
		steer_straight = (steer == 0.5)
		self.csv_writer.writerow([rgb_str]+[depth_str]+[str(thr)]+[str(steer_left)]+[str(steer_straight)]+[str(steer_right)])
		
	# Main run loop
	def run(self) :

		# Create window for robot control input
		cv2.namedWindow('display', cv2.WINDOW_NORMAL)
		
		# step values. Change to increase/decrease acceleration
		#thr_step = 2i
		#steer_step = 1

		# Start measuring time here
		start_time = time.time()

		# Main loop
		'''
		while(True) :
			char = '\0'
			char = cv2.waitKey(10) & 255
			if(char == 27) :
				print("\tEscape key detected!")
                		break

		        elif char == ord('w'):
                		self.thr = self.thr+thr_step
				self.steer = 50
                		if(self.thr>100) :
                        		self.thr = 100
        
        		elif char == ord('a') :
                		if(self.steer <= 50) :
                        		self.steer = 100
                		self.steer = self.steer - steer_step
                		if(self.steer<80) :
                        		self.steer = 80

		        elif char == ord('d') :
                		if(self.steer>=50) :
		                        self.steer = 0
                		self.steer = self.steer+steer_step
                		if(self.steer>20) :
                        		self.steer = 20

        		elif char == ord('s') :
				self.steer = 50
                		self.thr = self.thr-thr_step
                		if(self.thr<0) :
                        		self.thr = 0
			'''

			while (True) :
			if(char == 27) :
                                print("\tEscape key detected!")
                                break

                        elif char == ord('w'):
                                self.thr = 1
                                self.steer = 0.5
                                #if(self.thr>100) :
                                #        self.thr = 100
        
                        elif char == ord('a') :
				self.thr = 1
                       		steer = 0

                        elif char == ord('d') :
                                steer = 1
				
                        elif char == ord('s') :
                                self.steer = 0.5
				self.thr = 0
		
			self.drive(self.thr,self.steer)

			end_time = time.time()
			if((end_time-start_time)>0.1) :
				self.save_data(self.steer, self.thr)
				start_time = start_time + 0.1

		# Stop robot and exit
		self.stop()

if __name__ == '__main__' :
	os.system("rm -r data/")
	os.system("mkdir data/")
	kob = kobuki('/dev/ttyUSB0')
	kob.run()

	# make videos
	os.system("sh make_video.sh")
