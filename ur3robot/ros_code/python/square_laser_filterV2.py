#!/usr/bin/env python
#import time
#import math
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan

#******* This program is only for square robots. ********

robot_width = 0.4191 #my ur3 robot is 0.4191 meters width
inflation = 0.05

robot_width = robot_width + inflation

laser_lookup_table = [] #this will be where the initial startup math will calculate the minimum values for each position of laser values

def create_lookup_table():
    global robot_width
    global laser_lookup_table

    right_angle = 760 / 4
    deg_conversion = 360.0/760

    for deg in range(760): #
        #if ((deg <= (right_angle/2) or deg > ((right_angle/2)+ (right_angle*3)))) or (deg > ((right_angle/2) + right_angle) and deg <= ((right_angle/2) + (right_angle*2))):
        #    distance = (robot_width/2)/np.cos(np.radians(deg * deg_conversion))
        #    laser_lookup_table.append(abs(distance))
        if (((deg * deg_conversion) <= 45 or (deg * deg_conversion) > 315) or ((deg * deg_conversion) > 135 and (deg * deg_conversion) <= 225)):
            distance = (robot_width/2)/np.cos(np.radians((deg * deg_conversion)))
            laser_lookup_table.append(abs(distance))
        elif (((deg * deg_conversion) > 45 and (deg * deg_conversion) <= 135) or ((deg * deg_conversion) > 225 and (deg * deg_conversion) <= 315)):
            distance = (robot_width/2)/np.sin(np.radians(deg * deg_conversion))
            laser_lookup_table.append(abs(distance))
    print("DONE")

def run_filter(msg):
    global laser_lookup_table

    distances_array = []
    intensities_array = []

    deg_conversion = 360.0/760

    for deg in range(760):

        if msg.ranges[deg] <= laser_lookup_table[deg]:
            distances_array.append(0)
            intensities_array.append(0)
        else:
            distances_array.append(msg.ranges[deg])
            intensities_array.append(msg.intensities[deg])
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
    create_lookup_table()

    rospy.init_node('laser_filter')
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    pub = rospy.Publisher('scan/filtered', LaserScan, queue_size=5)

    rospy.spin()