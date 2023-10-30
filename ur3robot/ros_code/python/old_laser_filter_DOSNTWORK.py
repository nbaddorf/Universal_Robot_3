#!/usr/bin/env python
import time
import math
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan

#WARNING---- this program dosnt work super well. It only works on square or rectangle robots. 


corner_angles = []
#footprint = [[-0.20955,-0.20955], [0.20955,-0.20955], [0.20955,0.20955], [-0.20955,0.20955]]
#footprint = [[-0.20955,0.20955], [-0.20955,-0.20955], [0.20955,-0.20955], [0.20955,0.20955]]
#this program is not designed verry well. I am not sure it is all labeled correctly, but it works for now

calibrate_mode = False 
laser_inflation = 0.05
laser_tf_difference = math.pi + math.pi/2

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def deg2rad(val):
    return val * math.pi/180

def set_vars():
    global footprint
    global corner_angles
                 #back right,             front right,          front left,         back left
    footprint = [[-0.24765,-0.20955], [0.24765,-0.20955], [0.24765,0.20955], [-0.24765,0.20955]]
    int_counter = 0
    #print(footprint)
    for counter in footprint:
        if counter[0] < 0:
            footprint[int_counter][0] = counter[0] - laser_inflation
        elif counter[0] > 0:
            footprint[int_counter][0] = counter[0] + laser_inflation
        if counter[1] < 0:
            footprint[int_counter][1] = counter[1] - laser_inflation
        elif counter[1] > 0:
            footprint[int_counter][1] = counter[1] + laser_inflation
        int_counter += 1
    #print(footprint)
    #corner_angles = []
    for i in footprint:
        dis, rot = cart2pol(i[0],i[1])
        if rot < 0:
            rot = rot + (math.pi * 2)
        corner_angles.append(rot)

def run_filter(msg):
    global corner_angles
    global footprint

    distances_array = []
    intensities_array = []

    for deg in range(360):
        rad_angle = deg2rad(deg) + laser_tf_difference
        if rad_angle > (2 * math.pi):
            rad_angle = rad_angle - (2 * math.pi)

        #set the new distance reading to 0, will get chainged if program works correctly
        new_dis = 0.0

        if (rad_angle < corner_angles[1]) and (rad_angle > corner_angles[0]):
            tmp_angle = rad_angle - corner_angles[0] - ((corner_angles[1] - corner_angles[0]) / 2)
            tmp_angle = abs(tmp_angle)
            new_dis = (abs(footprint[0][1])) / math.cos(tmp_angle)

        elif (rad_angle > corner_angles[1]) or (rad_angle < corner_angles[2]):
            middle = (2 * math.pi) - corner_angles[1] + corner_angles[2]
            tmp_angle = rad_angle - corner_angles[1] - (middle / 2)
            tmp_angle = abs(tmp_angle)
            new_dis = (abs(footprint[1][0])) / math.cos(tmp_angle)

        elif (rad_angle > corner_angles[2]) and (rad_angle < corner_angles[3]):
            tmp_angle = rad_angle - corner_angles[2] - ((corner_angles[3] - corner_angles[2]) / 2)
            tmp_angle = abs(tmp_angle)
            new_dis = (abs(footprint[2][1])) / math.cos(tmp_angle)

        elif (rad_angle > corner_angles[3]) and (rad_angle < corner_angles[0]):
            tmp_angle = rad_angle - corner_angles[3] - ((corner_angles[0] - corner_angles[3]) / 2)
            tmp_angle = abs(tmp_angle)
            new_dis = (abs(footprint[0][0])) / math.cos(tmp_angle)
        
        if msg.ranges[deg] <= new_dis:
            if calibrate_mode == False:
                new_dis = 0 
            new_intensities = 0
        else:
            if calibrate_mode == False:
                new_dis = msg.ranges[deg]
            new_intensities = msg.intensities[deg]
 
        distances_array.append(new_dis)
        intensities_array.append(new_intensities)
    return distances_array, intensities_array
       
def callback(msg):
    new_ranges, new_intensities = run_filter(msg)
    scan = LaserScan()
    scan.header = msg.header
    scan.angle_min = msg.angle_min
    scan.angle_max = msg.angle_max
    scan.angle_increment = msg.angle_increment
    scan.time_increment = msg.time_increment
    scan.scan_time = msg.scan_time
    scan.range_min = msg.range_min
    scan.range_max = msg.range_max
    scan.ranges = new_ranges
    scan.intensities = new_intensities
    pub.publish(scan)

if __name__ == "__main__":
    rospy.init_node('laser_filter')
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    pub = rospy.Publisher('scan/filtered', LaserScan, queue_size=5)
    #I have to run the set_vars so that the laser filter will work untill I get the param to load. after the peram loads, I run set_var again, to calculate the new filter inflation
    set_vars()
    laser_inflation = rospy.get_param('~laser_inflation', 0.0)
    #footprint = rospy.get_param('/move_base/local_costmap/footprint')
    
    set_vars()
    rospy.spin()

