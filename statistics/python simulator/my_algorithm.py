'''
  ASSIGNMENT 1 - Luca Mosetti

  Write a Python script for achieving the following robot's behaviour:
  - the robot constantly drives around the circuit counter-clockwise;
  - the robot has to avoid the golden boxes;
  - when the robot detect a silver box it has to grab it and move it behind itself;

  Deadline: 12/11/21

'''

from __future__ import print_function
import time
import math
from sr.robot import *


DISTANCE_THRESHOLD= 0.9   # meters
FWD= "forward"
DX= "right"
SX= "left"


R= Robot()

##########################################
#         Functions for moving           #
##########################################
def drive(speed, seconds):
    """
    Function for setting a linear velocity for a given amount of time.

    Args: speed (int): the speed of the wheels;
	      seconds (int): the time interval;
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0

def turn(speed, seconds):
    """
    Function for setting an angular velocity for a given amount of time.

    Args: speed (int): the speed of the wheels;
	      seconds (int): the time interval;
    """
    R.motors[0].m0.power = speed
    R.motors[0].m1.power = -speed
    time.sleep(seconds)
    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0


##########################################
#    Functions for PID implementation    #
##########################################
def controller_PID(err, d_err, i_err):
    """
    Function for computing the output of the PID controller.

    Return: output (float): the controlling signal;
    """
    Kp= 1     # proportional action (the higher it is and the faster is the response)
    Kd= 50    # derivative action (to reach critical damping)
    Ki= 0

    output= Kp*err + Kd*d_err + Ki*i_err
    return output

def current_milli_time():
    """
    Function for returning the current system time converted in milliseconds.

    Return: the system time in milliseconds;
    """
    return time.time()*1000

def check_heading():
    """
    Function for returning the heading angle of the the mobile robot with
    respect to the inertial frame of reference.

    Return: heading angle expressed in degrees;
    """
    heading= 180/math.pi * R.heading
    return heading

def turn_ang(alfa):
    """
    Function to make the robot turn for a given angle with respect its current
    orientation.

    Args: ang (int): the turning angle, if positive the rotation occurs counter-
                     clockwise, otherwise it is clockwise;
    """
    angle= check_heading() + alfa
    # adjusting the angle
    if(angle > 180):
        angle= -(360 - angle)
    elif(angle < -180):
        angle= 360 + angle
    err= angle - check_heading()
    step= 0
    # entering the controlling loop
    while(abs(err)>= 0.01):
        t= current_milli_time()
        if(step == 0):
            d_err= 0
        else:
            err= angle - check_heading()
            d_err= (err - prevErr)/dt
        step+= 1
        speed= controller_PID(err, d_err, 0)
        R.motors[0].m0.power = speed
        R.motors[0].m1.power = -speed
        #print(err)
        prevErr= err
        dt= current_milli_time() - t   # compute the enlapsed time

    R.motors[0].m0.power = 0
    R.motors[0].m1.power = 0


##########################################
#        Functions for detection         #
##########################################
def search_obstacle(direction):
    """
    Function for detecting the closest obstacle (golden box) in a certain
    direction and return its distance and angular position with respect the
    robot.

    Args: driection (string): the direction where to search for an obstacle;
    """
    dist= 100

    if(direction == 'forward'):
        max_ang= 60
        min_ang= -60
    elif(direction == 'right'):
        max_ang= 135
        min_ang= 45
    elif(direction == 'left'):
        max_ang= -45
        min_ang= -135
    else:
        print("Error: direction not allowed.\n")
        return -1, -1

    for token in R.see():
        if(token.info.marker_type is MARKER_TOKEN_GOLD):
            if (token.dist < dist and min_ang < token.rot_y < max_ang):
                dist= token.dist
                rot_y= token.rot_y

    if (dist == 100):
        print("Error: no obstacle found in that direction.\n")
        return -1, -1
    else:
        return dist, rot_y

def search_target():
    """
    Function for finding the closest target (silver box) in front the Robot and
    return its distance and angular position.

    Returns:
	dist (float): distance of the closest token (-1 if no token is detected)
	rot_y (float): angle between the robot and the token (-1 if no token is detected)
    """
    dist= 1.5
    for token in R.see():
        if token.dist < dist and -45 < token.rot_y < 45 and token.info.marker_type is MARKER_TOKEN_SILVER:
            dist=token.dist
	    rot_y=token.rot_y
    if dist== 1.5:
	return -1, -1
    else:
   	return dist, rot_y


##########################################
#    Functions for target interaction    #
##########################################
def reach_target(target_coord):
    """
    Function for driving towards a target specified by the given coordinates.

    Args: target_coord (float): an array of two elements in which are saved the
          distance and the angular position of the target with respect the robot.
    """
    turn_ang(target_coord[1])
    while(target_coord[0] >= 0.4):
        drive(50, 0.1)
        target_coord= search_target()

def move_behind():
    """
    Function for grabbing a target and move it behind the robot.
    """
    R.grab()
    turn_ang(-180)
    R.release()
    turn_ang(180)



##########################################
#             Main function              #
##########################################
def main():
    print("\nGetting started, wait a second...")
    time.sleep(3)
    print("Ready to move.\n")

    while 1:
        obst_coord = search_obstacle(FWD)
        # if the forward obstacles are distant enough
        if(obst_coord[0] > DISTANCE_THRESHOLD):
            target_coord= search_target()
            # if a target has been detected, drive towards it
            if(target_coord[0] != -1):
                print("Target found, I'm going to reach it.\n")
                reach_target(target_coord)
                move_behind()
            # else, if no targets are in range, simply drive forward
            else:
                drive(50, 0.1)
        # else, if an obstacle in the front is too close
        else:
            dist_r, _ = search_obstacle(DX)
            dist_l, _ = search_obstacle(SX)
            print("Distance on the left: ", dist_l)
            print("Distance on the right: ", dist_r)
            # obstacle on the right
            if(dist_r <= dist_l or dist_l == -1):
                print("Obstacle closer on my right, I'm turning left.\n")
                turn(-30, 0.1)             # turn left
            # obstacle on the left
            elif(dist_l <= dist_r or dist_r == -1):
                print("Obstacle closer on my left, I'm turning right.\n")
                turn(30, 0.1)              # turn right



main()
