# L3_color_tracking.py
# This program was designed to have SCUTTLE following a target using a USB camera input

import cv2              # For image capture and processing
import numpy as np      
import L2_speed_control as sc
import L2_inverse_kinematics as ik
import L2_kinematics as kin
import netifaces as ni
from time import sleep
from math import radians, pi
import L2_vector as vec
import L1_lidar as lid

def wander(): 
    cord = vec.getNearest()
    if (cord[0] < .5 and cord[1] > -60 and cord[1] < 60):
        sc.driveOpenLoop(ik.getPdTargets([-0.25, 0])) 
        sleep(1)
        sc.driveOpenLoop(ik.getPdTargets([0, 0.9]))
        sleep(.5)
        print("Turning")
    else:
        sc.driveOpenLoop(ik.getPdTargets([0.25, 0])) 
        sleep(.5)


# Gets IP to grab MJPG stream
def getIp():
    for interface in ni.interfaces()[1:]:   #For interfaces eth0 and wlan0
    
        try:
            ip = ni.ifaddresses(interface)[ni.AF_INET][0]['addr']
            return ip
            
        except KeyError:                    #We get a KeyError if the interface does not have the info
            continue                        #Try the next interface since this one has no IPv4
        
    return 0
    
#    Camera
stream_ip = getIp()
if not stream_ip: 
    print("Failed to get IP for camera stream")
    exit()

camera_input = 'http://' + stream_ip + ':8090/?action=stream'   # Address for stream

size_w  = 240   # Resized image width. This is the image width in pixels.
size_h = 160	# Resized image height. This is the image height in pixels.

fov = 1         # Camera field of view in rad (estimate)

v1_min = [180,235,145]      # Minimum H value
v2_min = [145,70,0]     # Minimum S value
v3_min = [0,0,50]    # Minimum V value

v1_max = [235,250,155]    # Maximum H value
v2_max = [230,160,255]    # Maximum S value
v3_max = [25,0,255]    # Maximum V value


target_width = 20      # Target pixel width of tracked object
angle_margin = 0.12      # Radians object can be from image center to be considered "centered"
width_margin = 3       # Minimum width error to drive forward/back

def main():
    state = 1
    # Try opening camera with default method
    try:
        camera = cv2.VideoCapture(0)    
    except: pass

    # Try opening camera stream if default method failed
    if not camera.isOpened():
        camera = cv2.VideoCapture(camera_input)    

    camera.set(3, size_w)                       # Set width of images that will be retrived from camera
    camera.set(4, size_h)                       # Set height of images that will be retrived from camera
    alignedCount = 0
    try:
        while True:
            sleep(.05)                                          

            ret, image = camera.read()  # Get image from camera

            # Make sure image was grabbed
            if not ret:
                print("Failed to retrieve image!")
                break

            image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)              # Convert image to HSV

            height, width, channels = image.shape                       # Get shape of image

            thresh = cv2.inRange(image, (v1_min[state], v2_min[state], v3_min[state]), (v1_max[state], v2_max[state], v3_max[state]))   # Find all pixels in color range

            kernel = np.ones((5,5),np.uint8)                            # Set kernel size
            mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)     # Open morph: removes noise w/ erode followed by dilate
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)      # Close morph: fills openings w/ dilate followed by erode
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)[-2]                        # Find closed shapes in image
            
            if len(cnts) and len(cnts) < 3:                             # If more than 0 and less than 3 closed shapes exist

                c = max(cnts, key=cv2.contourArea)                      # return the largest target area
                x,y,w,h = cv2.boundingRect(c)                           # Get bounding rectangle (x,y,w,h) of the largest contour
                center = (int(x+0.5*w), int(y+0.5*h))                   # defines center of rectangle around the largest target area
                angle = round(((center[0]/width)-0.5)*fov, 3)           # angle of vector towards target center from camera, where 0 deg is centered

                wheel_measured = kin.getPdCurrent()                     # Wheel speed measurements

                # If robot is facing target
                if abs(angle) < angle_margin:                                 
                    e_width = target_width - w                          # Find error in target width and measured width

                    # If error width is within acceptable margin
                    if abs(e_width) < width_margin:
                        sc.driveOpenLoop(np.array([0.,0.]))             # Stop when centered and aligned
                        print("Aligned! ",w)
                        alignedCount += 1
                        if (state == 0):
                            if(alignedCount >= 10) :
                                sc.driveOpenLoop(ik.getPdTargets([0, 0.5]))
                                sleep(.45)
                                sc.driveOpenLoop(ik.getPdTargets([0.25, 0]))
                                sleep(3)
                                alignedCount = 0
                                print("Collect 0")
                        if (state == 1):
                            if(alignedCount >= 10) :
                                #sc.driveOpenLoop(ik.getPdTargets([0, 0.5]))
                                #sleep(.45)
                                sc.driveOpenLoop(ik.getPdTargets([0.25, 0]))
                                sleep(3)
                                alignedCount = 0
                                print("Collect 1")
                        if (state == 2):
                            if(alignedCount >= 10) :
                                sc.driveOpenLoop(ik.getPdTargets([0, -0.5]))
                                sleep(.45)
                                sc.driveOpenLoop(ik.getPdTargets([0.25, 0]))
                                sleep(3)
                                alignedCount = 0
                                print("Collect 0")


                        print("collected")    
                        continue
                    else : 
                        alignedCount = 0

                    fwd_effort = e_width/target_width                   
                    
                    wheel_speed = ik.getPdTargets(np.array([0.8*fwd_effort, -0.5*angle]))   # Find wheel speeds for approach and heading correction
                    sc.driveClosedLoop(wheel_speed, wheel_measured, 0)  # Drive closed loop
                    print("Angle: ", angle, " | Target L/R: ", *wheel_speed, " | Measured L\R: ", *wheel_measured)
                    continue

                wheel_speed = ik.getPdTargets(np.array([0, -1.1*angle]))    # Find wheel speeds for only turning

                sc.driveClosedLoop(wheel_speed, wheel_measured, 0)          # Drive robot
                print("Angle: ", angle, " | Target L/R: ", *wheel_speed, " | Measured L\R: ", *wheel_measured)

            else:
                
                wander()

                state += 1
                if (state > 2):
                    state = 0
                print("No targets")
                print(state)
                sc.driveOpenLoop(np.array([0.,0.]))         # stop if no targets detected

                
    except KeyboardInterrupt: # condition added to catch a "Ctrl-C" event and exit cleanly
        pass

    finally:
    	print("Exiting Color Tracking.")

if __name__ == '__main__':
    main()

