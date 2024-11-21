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

np.set_printoptions(precision=3)


def rotor():
    pass


def camera():
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
            
            print("m = ", m, " and deg =", deg)
            print("desired phil = ",pdTargets[0],"desired phir = ", pdTargets[1])
            print('current phil= ', pdCurrents[0] , 'current phir =  ', pdCurrents[1])
            
            # need fnct to relay if obstacle is in path, avoid
            # positive deg is left, neg deg is right
            ### left obstacle
            if ((0.35<m < 0.8) and ( 0<deg<45)):
                L2_speed_control.driveOpenLoop(np.array([-4.0,-4.0]))
                sleep(2)
                L2_speed_control.driveOpenLoop(np.array([4.0,-2.0]))
                sleep(2)
            if ((0.05<m < 0.35) and ( 45<deg<90)):
                L2_speed_control.driveOpenLoop(np.array([5.0,-2.0]))
                sleep(2)
            ### right obstacle
            if ((0.35<m < 0.8) and ( -45<deg<0)):
                L2_speed_control.driveOpenLoop(np.array([-4.0,-4.0]))
                sleep(2)
                L2_speed_control.driveOpenLoop(np.array([-2.0,4.0]))
                sleep(2)
            if ((0.05<m < 0.35) and ( -90<deg<-45)):
                L2_speed_control.driveOpenLoop(np.array([-2.0,5.0]))
                sleep(2)
            else:
                continue

