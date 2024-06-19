#!/usr/bin/env python3

import rospy
from stdmsgs.msg import Float32, Float32MultiArray
from maincontrol.msg import Gnss
import math
from pyproj import Proj
import numpy as np


n1x, n1y, n2_x, n2_y = 0.0, 0.0, 0.0, 0.0
gps_x, gps_y = 0.0, 0.0
yaw = 0.0
count = 0


proj_wgs84 = Proj(proj='latlong', datum='WGS84')


proj_utm = Proj(proj='utm', zone=52, datum='WGS84')
def get_error(x1, y1, x2, y2, x0, y0):
    # 직선의 방정식 계수
    A = y2 - y1
    B = -(x2 - x1)
    C = x2 * y1 - y2 * x1


    numerator = abs(A * x0 + B * y0 + C)
    denominator = math.sqrt(A2 + B2)

    distance = numerator / denominator
    return distance

def point_callback(msg):
    global n1_x, n1_y, n2_x, n2_y

    lon = msg.data[0]
    lat = msg.data[1]
    n1_x, n1_y = proj_utm(lon, lat)
    lon = msg.data[2]
    lat = msg.data[3]
    n2_x, n2_y = proj_utm(lon, lat)

def gps_callback(msg):
    global gps_x, gps_y, n1_x, n1_y, n2_x, n2_y
    lon = msg.longitude
    lat = msg.latitude
    gps_x, gps_y = proj_utm(lon, lat)
    cross_error=get_error(n1_x,n1_y,n2_x,n2_y,gps_x,gps_y)
    rospy.loginfo(f"cross_error: {cross_error}")

    heading = msg.heading
    yaw_error = np.arctan2((n2_y - n1_y),(n2_x-n1_x)) - heading
    rospy.loginfo(f"yaw_error: {yaw_error}")


def main():
    rospy.init_node('contro_recode')
    rospy.Subscriber('/kalman_pose', Gnss, gps_callback)
    rospy.Subscriber('/current_node', Float32MultiArray, point_callback)
    rospy.spin()

if __name__ == '__main':
    try:
        main()
    except rospy.ROSInterruptException:
        pass