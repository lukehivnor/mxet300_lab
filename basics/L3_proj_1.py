import L1_encoder
import L1_lidar
import L2_inverse_kinematics
import L2_kinematics
import L2_speed_control
import L2_vector
import L1_log
import lidar_driving
import numpy as np
from time import sleep
from math import radians, pi
import cv2              # For image capture and processing
import numpy as np      
    
import netifaces as ni
from time import sleep
from math import radians, pi

np.set_printoptions(precision=3)

from gpiozero import PWMOutputDevice as pwm # for driving motors, LEDs, etc
import time                                 # for keeping time
import numpy as np                          # for handling arrays

frq = 150                                   # motor driving frequency
# Broadcom (BCM) pin numbering for RasPi is as follows: PHYSICAL:       NAME:
rotor_A = pwm(9,frequency=frq, initial_value=0)         #pin21           GPIO9
rotor_B = pwm(25,frequency=frq, initial_value=0)        #pin22           GPIO25



def computePWM(speed):              # take an argument in range [-1,1]
    if speed == 0:
        x = np.array([0,0])         # set all PWM to zero
    else:
        x = speed + 1.0             # change the range to [0,2]
        chA = 0.5 * x               # channel A sweeps low to high
        chB = 1 - (0.5 * x)         # channel B sweeps high to low
        x = np.array([chA, chB])    # store values to an array
        x = np.round(x,2)           # round the values
    return(x)

def send_rotor(mySpeed):
    myPWM = computePWM(mySpeed)
    rotor_B.value = myPWM[0]
    rotor_A.value = myPWM[1]

def rotor():
    send_rotor(1)
    pass

def getIp():
    for interface in ni.interfaces()[1:]:   #For interfaces eth0 and wlan0
    
        try:
            ip = ni.ifaddresses(interface)[ni.AF_INET][0]['addr']
            return ip
            
        except KeyError:                    #We get a KeyError if the interface does not have the info
            continue                        #Try the next interface since this one has no IPv4
        
    return 0
    
stream_ip = getIp()

def cam_setup():
    #    Camera
    
    if not stream_ip: 
        print("Failed to get IP for camera stream")
        exit()
    
    



def camera():
    camera_input = 'http://' + stream_ip + ':8090/?action=stream'   # Address for stream

    size_w  = 240   # Resized image width. This is the image width in pixels.
    size_h = 160	# Resized image height. This is the image height in pixels.

    fov = 1         # Camera field of view in rad (estimate)

    #    Color Range, described in HSV
    v1_min = 5      # Minimum H value
    v2_min = 35     # Minimum S value
    v3_min = 80      # Minimum V value

    v1_max = 60     # Maximum H value
    v2_max = 185    # Maximum S value
    v3_max = 255    # Maximum V value

    target_width = 50      # Target pixel width of tracked object
    angle_margin = 0.2      # Radians object can be from image center to be considered "centered"
    width_margin = 10      # Minimum width error to drive forward/back
    try:
        camera = cv2.VideoCapture(0)    
    except: pass

    # Try opening camera stream if default method failed
    if not camera.isOpened():
        camera = cv2.VideoCapture(camera_input)    

    camera.set(3, size_w)                       # Set width of images that will be retrived from camera
    camera.set(4, size_h)                       # Set height of images that will be retrived from camera

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

            thresh = cv2.inRange(image, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))   # Find all pixels in color range

            kernel = np.ones((5,5),np.uint8)                            # Set kernel size
            mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)     # Open morph: removes noise w/ erode followed by dilate
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)      # Close morph: fills openings w/ dilate followed by erode
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)[-2]                        # Find closed shapes in image
            
            if len(cnts) and len(cnts) < 2:                             # If more than 0 and less than 3 closed shapes exist

                c = max(cnts, key=cv2.contourArea)                      # return the largest target area
                x,y,w,h = cv2.boundingRect(c)                           # Get bounding rectangle (x,y,w,h) of the largest contour
                center = (int(x+0.5*w), int(y+0.5*h))                   # defines center of rectangle around the largest target area
                angle = round(((center[0]/width)-0.5)*fov, 3)           # angle of vector towards target center from camera, where 0 deg is centered

                wheel_measured = L2_kinematics.getPdCurrent()                     # Wheel speed measurements

                # If robot is facing target
                if abs(angle) < angle_margin:                                 
                    e_width = target_width - w                          # Find error in target width and measured width

                    # If error width is within acceptable margin
                    if abs(e_width) < width_margin:
                        L2_speed_control.driveOpenLoop(np.array([0.,0.]))             # Stop when centered and aligned
                        print("Aligned! ",w)
                        exit()

                    fwd_effort = e_width/target_width                   
                    
                    wheel_speed = L2_inverse_kinematics.getPdTargets(np.array([0.8*fwd_effort, -0.5*angle]))   # Find wheel speeds for approach and heading correction
                    L2_speed_control.driveClosedLoop(wheel_speed, wheel_measured, 0)  # Drive closed loop
                    continue

                wheel_speed = L2_inverse_kinematics.getPdTargets(np.array([0, -1.1*angle]))    # Find wheel speeds for only turning

                L2_speed_control.driveClosedLoop(wheel_speed, wheel_measured, 0)          # Drive robot

            else:
                continue

                
    except KeyboardInterrupt: # condition added to catch a "Ctrl-C" event and exit cleanly
        pass

    


def spiral():
    outer_radius = 50  # Outer radius in meters
    theta_max = 1 * np.pi  # Maximum angle in radians (adjust for tighter or looser spiral)
    b = outer_radius / theta_max  # Calculate 'b' based on outer radius
    # Generate theta values from 0 to theta_max
    theta = np.linspace(0, theta_max, 100)
    # Calculate r for each theta (Archimedean spiral formula)
    r = b * theta

    # Convert polar to Cartesian coordinates (x, y) in the global frame
    x_global = r * np.cos(theta)
    y_global = r * np.sin(theta)
    vec = x_global, y_global, r, theta
    return vec


def global_to_local(vec):
    # Initialize lists for local coordinates and velocities
    x_local = []
    y_local = []
    linear_velocity = []
    angular_velocity = []
    x_global, y_global, r, theta = vec

    # Calculate the robot's local x and y positions and velocities at each point
    for i in range(1, len(x_global) - 1):
        # Current and previous points in the global frame
        x1, y1 = x_global[i], y_global[i]
        x2, y2 = x_global[i + 1], y_global[i + 1]

        # Calculate the robot's heading angle (theta) in the global frame
        heading_angle = np.arctan2( y2 - y1, x2 - x1 )

        # Transform (x2, y2) to the robot's local frame (origin at (x1, y1) with heading direction)
        dx = x2 - x1
        dy = y2 - y1
        x_local.append(dx * np.cos(heading_angle) - dy * np.sin(heading_angle))
        y_local.append(dx * np.sin(heading_angle) + dy * np.cos(heading_angle))

        # Calculate the forward (linear) velocity
        linear_velocity.append(np.hypot(dx, dy))

        # Calculate the angular velocity (change in heading)
        if i > 1:
            previous_heading = np.arctan2(y1 - y_global[i - 1], x1 - x_global[i - 1])
            angular_velocity.append((heading_angle - previous_heading) / (theta[i + 1] - theta[i]))
        else:
            angular_velocity.append(0)  # No angular velocity at the first point
    return linear_velocity, angular_velocity


def driver(xdot, thetadot):
    B = np.array([xdot, thetadot])
    B = L2_inverse_kinematics.getPdTargets(B)
    return B


def avoid():
    m, deg = L2_vector.getNearest()
    x, y = L2_vector.polar2cart(m, deg)
    return m, deg, x, y


def obstacle_avoid(near):
    offset = L2_inverse_kinematics.phi_influence(near)
    return offset


def closed_loop_drive():
    # INITIALIZE VARIABLES FOR CONTROL SYSTEM
    t0 = 0  # time sample
    t1 = 1  # time sample
    e00 = 0  # error sample
    e0 = 0  # error sample
    e1 = 0  # error sample
    dt = 0  # delta in time
    de_dt = np.zeros(2)  # initialize the de_dt
    import time
    while (1):
        count += 1  # count the number of times this loop has run
        # THIS CODE IS FOR OPEN AND CLOSED LOOP control
        pdTargets = np.array([9.7, 9.7])  # Input requested PhiDots (radians/s)
        # pdTargets = inv.getPdTargets() # populates target phi dots from GamePad
        L2_kinematics.getPdCurrent()  # capture latest phi dots & update global var
        pdCurrents = L2_kinematics.pdCurrents  # assign the global variable value to a local var

        # THIS BLOCK UPDATES VARIABLES FOR THE DERIVATIVE CONTROL
        t0 = t1  # assign t0
        t1 = time.time()  # generate current time
        dt = t1 - t0  # calculate dt
        e00 = e0  # assign previous previous error
        e0 = e1  # assign previous error
        e1 = pdCurrents - pdTargets  # calculate the latest error
        de_dt = (e1 - e0) / dt  # calculate derivative of error

        # CALLS THE CONTROL SYSTEM TO ACTION
        # sc.driveOpenLoop(pdTargets)  # call on open loop
        L2_speed_control.driveClosedLoop(pdTargets, pdCurrents, de_dt)  # call on closed loop
        time.sleep(0.05)  # this time controls the frequency of the controller



if __name__ == "__main__":
    while True:
        vec = spiral()
        # INITIALIZE VARIABLES FOR CONTROL SYSTEM
        t0 = 0  # time sample
        t1 = 1  # time sample
        e00 = 0  # error sample
        e0 = 0  # error sample
        e1 = 0  # error sample
        dt = 0  # delta in time
        de_dt = np.zeros(2)  # initialize the de_dt
        import time

        xdot, thetadot = global_to_local(vec)

        phil, phir = driver(xdot, thetadot)

        L2_speed_control.openLoop(0., 0.)
        cam_setup()

        for k in range(len(phil)):
            # duty = L2_speed_control.openLoop(phil[k], phir[k])
            # THIS CODE IS FOR OPEN AND CLOSED LOOP control

            pdTargets = np.array([phil[k], phir[k]])
            #pdTargets = np.array([-4.2, 4.2]) # Input requested PhiDots (radians/s)
            # pdTargets = inv.getPdTargets() # populates target phi dots from GamePad
            L2_kinematics.getPdCurrent()  # capture latest phi dots & update global var
            pdCurrents = L2_kinematics.pdCurrents  # assign the global variable value to a local var

            # THIS BLOCK UPDATES VARIABLES FOR THE DERIVATIVE CONTROL
            t0 = t1  # assign t0
            t1 = time.time()  # generate current time
            dt = t1 - t0  # calculate dt
            e00 = e0  # assign previous previous error
            e0 = e1  # assign previous error
            e1 = pdCurrents - pdTargets  # calculate the latest error
            de_dt = (e1 - e0) / dt  # calculate derivative of error

            # CALLS THE CONTROL SYSTEM TO ACTION
            # sc.driveOpenLoop(pdTargets)  # call on open loop
            L2_speed_control.driveClosedLoop(pdTargets, pdCurrents, de_dt)  # call on closed loop
            time.sleep(0.1)  # this time controls the frequency of the controller

            ## old
            desired = phil[k], phir[k]

            ### CHANGE LINE LATER

            '''
            L2_speed_control.driveOpenLoop(desired)
            '''

            m, deg, x, y = avoid()
            camera()
            
            print("m = ", m, " and deg =", deg)
            print("desired phil = ",pdTargets[0],"desired phir = ", pdTargets[1])
            print('current phil= ', pdCurrents[0] , 'current phir =  ', pdCurrents[1])
            
            # need fnct to relay if obstacle is in path, avoid
            # positive deg is left, neg deg is right
            
            # corner logic
            if ((0.05<m < 0.7) and ( -45<deg<45)):
                L2_speed_control.driveOpenLoop(np.array([-4.0,-4.0]))
                sleep(1)
                L2_speed_control.driveOpenLoop(np.array([-4.0,4.0]))
                sleep(2)
            ### left obstacle
            elif ((0.45<m < 0.8) and ( 0<deg<60)):
                L2_speed_control.driveOpenLoop(np.array([-4.0,-4.0]))
                sleep(1)
                L2_speed_control.driveOpenLoop(np.array([3.0,-3.0]))
                sleep(2)
            elif ((0.05<m < 0.45) and ( 45<deg<90)):
                L2_speed_control.driveOpenLoop(np.array([4.0,-3.0]))
                sleep(2)
            else:
                continue
            ### right obstacle
            if ((0.45<m < 0.8) and ( -60<deg<0)):
                L2_speed_control.driveOpenLoop(np.array([-4.0,-4.0]))
                sleep(1)
                L2_speed_control.driveOpenLoop(np.array([-3.0,3.0]))
                sleep(2)
            elif ((0.05<m < 0.45) and ( -90<deg<-45)):
                L2_speed_control.driveOpenLoop(np.array([-3.0,4.0]))
                sleep(2)
            else:
                continue

