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
    outer_radius = 10  # Outer radius in meters
    theta_max = 12 * np.pi  # Maximum angle in radians (adjust for tighter or looser spiral)
    b = outer_radius / theta_max  # Calculate 'b' based on outer radius
    # Generate theta values from 0 to theta_max
    theta = np.linspace(0, theta_max, 1000)
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
    for i in range(1, len(x_global)-1):
        # Current and previous points in the global frame
        x1, y1 = x_global[i], y_global[i]
        x2, y2 = x_global[i+1], y_global[i+1]

        # Calculate the robot's heading angle (theta) in the global frame
        heading_angle = np.arctan2(y2 - y1, x2 - x1)

        # Transform (x2, y2) to the robot's local frame (origin at (x1, y1) with heading direction)
        dx = x2 - x1
        dy = y2 - y1
        x_local.append(dx * np.cos(heading_angle) - dy * np.sin(heading_angle))
        y_local.append(dx * np.sin(heading_angle) + dy * np.cos(heading_angle))

        # Calculate the forward (linear) velocity
        linear_velocity.append(np.hypot(dx, dy))

        # Calculate the angular velocity (change in heading)
        if i > 1:
            previous_heading = np.arctan2(y1 - y_global[i-1], x1 - x_global[i-1])
            angular_velocity.append((heading_angle - previous_heading) / (theta[i+1] - theta[i]))
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



if __name__ == "__main__":
    while True:
        vec = spiral()


        xdot, thetadot = global_to_local(vec)

        phil, phir = driver(xdot, thetadot)
        
        for k in range(len(phil)):            
            # duty = L2_speed_control.openLoop(phil[k], phir[k])
            current = phil[k]/2, phir[k]/2

            ### CHANGE LINE LATER
            


            L2_speed_control.driveOpenLoop(current)

            
            m, deg, x, y = avoid()
            print("m = ", m, " and deg =", deg)
            #need fnct to relay if obstacle is in path, avoid
            if ((m<1) and ( -30< deg < 30)):
                L2_speed_control.driveOpenLoop([0,0])
                sleep(10)
            else:
                sleep(0.1)
                continue

