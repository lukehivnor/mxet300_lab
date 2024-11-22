import threading
import cv2
import numpy as np
from time import sleep
from gpiozero import PWMOutputDevice as pwm
import netifaces as ni
import L2_speed_control
import L2_inverse_kinematics
import L2_kinematics
import L2_vector

# Motor configurations
FRQ = 150
motor_left = pwm(9, frequency=FRQ, initial_value=0)
motor_right = pwm(25, frequency=FRQ, initial_value=0)

# Camera parameters
CAMERA_WIDTH = 240
CAMERA_HEIGHT = 160
HSV_MIN = (5, 40, 90)
HSV_MAX = (210, 235, 255)
TARGET_WIDTH = 50

# Shared variables
global camera 
processed_image = None
cam_on = False
image_lock = threading.Lock()

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


def initialize_camera():
    """Initializes the camera and sets resolution."""
    global camera, cam_on
    cam_on = True
    
    camera_url = "http://10.250.23.243:8090/?action=stream"
    
    # Attempt to get the camera stream
    index = -1
    try:
        camera = cv2.VideoCapture(index, cv2.CAP_FFMPEG)
    except:
        camera = cv2.VideoCapture(camera_url)
    
     
    
    sleep(0.1)
    if not camera or not camera.isOpened():
        raise RuntimeError("Failed to initialize camera.")

    camera.set(3, CAMERA_WIDTH)
    camera.set(4, CAMERA_HEIGHT)
    return camera

def process_camera_feed():
    """Thread to process the camera feed and extract HSV mask."""
    global processed_image, cam_on
    global camera
    try:
        camera = initialize_camera()
    except RuntimeError as e:
        print(e)
        return

    while cam_on:
        ret, frame = camera.read()
        if ret:
            hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, HSV_MIN, HSV_MAX)

            # Morphological filtering to clean up the mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            with image_lock:
                processed_image = mask
        sleep(0.05)

    camera.release()
    return processed_image

def avoid_obstacles():
    """Manages obstacle avoidance using LIDAR data."""
    m, deg = L2_vector.getNearest()
    if 0.05 < m < 0.7 and -45 < deg < 45:
        # Front obstacle
        L2_speed_control.driveOpenLoop(np.array([-4.0, -4.0]))
        sleep(1)
        L2_speed_control.driveOpenLoop(np.array([-4.0, 4.0]))
        sleep(2)
    elif 0.45 < m < 0.8 and 0 < deg < 60:
        # Left obstacle
        L2_speed_control.driveOpenLoop(np.array([-4.0, -4.0]))
        sleep(1)
        L2_speed_control.driveOpenLoop(np.array([3.0, -3.0]))
        sleep(2)
    elif 0.05 < m < 0.45 and 45 < deg < 90:
        L2_speed_control.driveOpenLoop(np.array([4.0, -3.0]))
        sleep(2)
    elif 0.45 < m < 0.8 and -60 < deg < 0:
        # Right obstacle
        L2_speed_control.driveOpenLoop(np.array([-4.0, -4.0]))
        sleep(1)
        L2_speed_control.driveOpenLoop(np.array([-3.0, 3.0]))
        sleep(2)
    elif 0.05 < m < 0.45 and -90 < deg < -45:
        L2_speed_control.driveOpenLoop(np.array([-3.0, 4.0]))
        sleep(2)

def align_with_target():
    """Aligns the robot with the detected target using HSV filtering and motion control."""
    fov = 1         # Camera field of view in rad (estimate)

            #    Color Range, described in HSV
    v1_min = 5      # Minimum H value
    v2_min = 40     # Minimum S value
    v3_min = 90      # Minimum V value

    v1_max = 210     # Maximum H value
    v2_max = 235    # Maximum S value
    v3_max = 255    # Maximum V value

    target_width = 25      # Target pixel width of tracked object    
    camera = initialize_camera()
    try:
        while True:
            sleep(0.05)

            ret, image = camera.read()  # Capture frame from camera

            if not ret or image is None:
                print("Failed to retrieve image!")
                continue

            # Convert the image to HSV
            image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            height, width, _ = image.shape

            # Apply HSV filtering
            thresh = cv2.inRange(image, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

            if thresh is None:
                print("Thresholding failed. Skipping frame.")
                continue

            # Morphological transformations
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)  # Remove noise
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Fill gaps

            if mask is None or np.sum(mask) == 0:
                print("Mask is empty or invalid. Skipping frame.")
                continue

            # Find contours in the mask
            contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            if 0 < len(contours) < 3:  # If 1 or 2 contours are detected
                # Get the largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)
                center = (int(x + 0.5 * w), int(y + 0.5 * h))

                # Calculate the angle to the target
                angle = round(((center[0] / width) - 0.5) * fov, 3)
                wheel_measured = L2_kinematics.getPdCurrent()

                # Check if the robot is aligned with the target
                if abs(angle) < 0.05:  # Threshold for alignment angle
                    e_width = target_width - w

                    # Check if the target is at the desired distance
                    if abs(e_width) < 10:  # Threshold for width error
                        L2_speed_control.driveOpenLoop(np.array([0.0, 0.0]))  # Stop the robot
                        print("Aligned! Target width:", w)
                        continue

                    # Move forward or backward to adjust distance
                    forward_effort = e_width / target_width
                    wheel_speed = L2_inverse_kinematics.getPdTargets(
                        np.array([0.8 * forward_effort, -0.5 * angle])
                    )
                    L2_speed_control.driveClosedLoop(wheel_speed, wheel_measured, 0)
                    print(f"Adjusting distance. Angle: {angle}, Target L/R: {wheel_speed}, Measured L/R: {wheel_measured}")
                    continue

                # Adjust the angle to face the target
                wheel_speed = L2_inverse_kinematics.getPdTargets(np.array([0, -1.1 * angle]))
                L2_speed_control.driveClosedLoop(wheel_speed, wheel_measured, 0)
                print(f"Turning to align. Angle: {angle}, Target L/R: {wheel_speed}, Measured L/R: {wheel_measured}")

            else:
                print("No targets detected.")
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


    except KeyboardInterrupt:
        print("Alignment interrupted by user.")

    finally:
        print("Exiting align_with_target function.")
        L2_speed_control.driveOpenLoop(np.array([0.0, 0.0]))  # Ensure the robot stops


def main():
    """Main function to run the threads."""
    try:
        camera_thread = threading.Thread(target=process_camera_feed)
        control_thread = threading.Thread(target=align_with_target)

        camera_thread.start()
        control_thread.start()

        camera_thread.join()
        control_thread.join()
    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        global cam_on
        cam_on = False
        if camera:
            camera.release()

if __name__ == "__main__":
    main()
