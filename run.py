import kobuki
import time
from datetime import datetime
import cv2
import numpy as np

kob = kobuki.Kobuki('/dev/ttyUSB0')
thr = 0
steer = 50
thr_step = 2
steer_step = 1

#print(time.time())
kob.run()

'''
while(True) :
	if(char == ord('q')) :
		break

	elif char == curses.KEY_UP :
		thr = thr+thr_step
		if(thr>100) :
			thr = 100
		kob.drive(thr,50)
	
	elif char == curses.KEY_LEFT :
		if(steer <= 50) :
			steer = 100
		steer = steer - steer_step
		if(steer<80) :
			steer = 80
		kob.drive(thr,steer)

	elif char == curses.KEY_RIGHT :
		if(steer>=50) :
			steer = 0
		steer = steer+steer_step
		if(steer>20) :
			steer = 20
		kob.drive(thr,steer)

	#elif char == curses.KEY_DOWN :
	else :
		thr = thr-thr_step
		if(thr<0) :
			thr = 0
                kob.drive(thr,50)



#print(time.time())

kob.stop()
'''
'''
## main loop
s=0
done = False
while not done:
	key = cv2.waitKey(1) & 255
	## Read keystrokes
	if key == 27: # terminate
		print "\tESC key detected!"
		done = True
	elif chr(key) =='s': #screen capture
		print "\ts key detected. Saving image {}".format(s)
		cv2.imwrite("ex4_"+str(s)+'.png', canvas)
		#s+=1 # uncomment for multiple captures
	#if

	## Streams
	#RGB
	rgb = kob.get_rgb()

	#DEPTH
	_,d4d = kob.get_depth()

	# canvas
	canvas = np.hstack((rgb,d4d))
	## Display the stream syde-by-side
	cv2.imshow('depth || rgb', canvas )
# end while
'''
#kob.save_data(0,0)
#kob.save_data(100,0)
