#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from pyquaternion import Quaternion
import yaml
import numpy as np
import message_filters
from sensor_msgs.msg import Image, CompressedImage, LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
# from ultralytics_ros.msg import YoloResult


class PROJECTNode:
    def __init__(self):

        self.bridge = CvBridge()

        # self.scan_topic = rospy.get_param("~scan_topic",default='/scan')
        # self.image_topic = rospy.get_param("~image_topic",default = "/usb_cam/image_raw")
        # self.calib_file = rospy.get_param("~calib_file",default = "/home/wonjae/설프/fusion_ws/src/camera_2d_lidar_calibration/data/calibration_result.txt")
        # self.config_file = rospy.get_param("~config_file",default="/home/wonjae/설프/fusion_ws/src/camera_2d_lidar_calibration/config/config.yaml")
        self.scan_topic = rospy.get_param("~scan_topic",default='/euclidean_clustering_pcl2')
        self.image_topic = rospy.get_param("~image_topic",default = "/usb_cam/image_rect_color")
        self.calib_file = rospy.get_param("~calib_file",default = "/home/baek/perception/sensor_fusion_ws/src/camera_2d_lidar_calibration/data/calibration_result.txt")
        self.config_file = rospy.get_param("~config_file",default="/home/baek/perception/sensor_fusion_ws/src/camera_2d_lidar_calibration/config/config.yaml")

        # self.scan_topic = rospy.get_param('~scan_topic')
        # self.image_topic = rospy.get_param('~image_topic')
        # self.calib_file = rospy.get_param('~calib_file')
        # self.config_file = rospy.get_param('~config_file')
        self.laser_point_radius = rospy.get_param("~laser_point_radius",default=3)
        self.time_diff = rospy.get_param("~time_diff",default=1)
        self.bridge = CvBridge()
        self.lp = lg.LaserProjection()

        with open(self.calib_file, 'r') as f:
            data = f.read().split()
            self.qx = float(data[0])
            self.qy = float(data[1])
            self.qz = float(data[2])
            self.qw = float(data[3])
            self.tx = float(data[4])
            self.ty = float(data[5])
            self.tz = float(data[6])
        self.q = Quaternion(self.qw,self.qx,self.qy,self.qz).transformation_matrix
        self.q[0,3] = self.tx
        self.q[1,3] = self.ty
        self.q[2,3] = self.tz
        print("Extrinsic parameter - camera to laser")
        print(self.q)
        self.tvec = self.q[:3,3]
        self.rot_mat = self.q[:3,:3]
        self.rvec, _ = cv2.Rodrigues(self.rot_mat)

        with open(self.config_file, 'r') as f:
            f.readline()
            config = yaml.full_load(f)
            # config = yaml.load(f)
            self.lens = config['lens']
            self.fx = float(config['fx'])
            self.fy = float(config['fy'])
            self.cx = float(config['cx'])
            self.cy = float(config['cy'])
            self.k1 = float(config['k1'])
            self.k2 = float(config['k2'])
            self.p1 = float(config['p1/k3'])
            self.p2 = float(config['p2/k4'])  

        self.K = np.matrix([[self.fx, 0.0, self.cx],
                    [0.0, self.fy, self.cy],
                    [0.0, 0.0, 1.0]])
        
        self.D = np.array([ -0.372105, 0.096005, 0.006422, -0.003821, 0.000000])
        # self.D = np.array([ 0.13526879, -0.43843309,  0.00320269, -0.00393454,  0.44240024])
        print("Camera parameters")
        print("Lens = %s" % self.lens)
        print("K =")
        print(self.K)
        print("D =")
        print(self.D)

        self.pub = rospy.Publisher("/reprojection", Image, queue_size=1)

        scan_sub = message_filters.Subscriber(self.scan_topic, PointCloud2, queue_size=1)
        # print(self.scan_topic)
        # print(scan_sub)
        # scan_sub = message_filters.Subscriber(self.scan_topic, LaserScan, queue_size=1)
        image_sub = message_filters.Subscriber(self.image_topic, Image, queue_size=1)
 
        ts = message_filters.ApproximateTimeSynchronizer([scan_sub, image_sub], 10, 9999999999)
        # print(ts)
        # print("ts.slop : " + str(ts.slop))
        ts.registerCallback(self.callback)
        # print("registerCallback")
        # ts.registerCallback(self.callback(scan_sub, image_sub))


    def get_z(self,T_cam_world, T_world_pc, K):
        R = T_cam_world[:3,:3]
        t = T_cam_world[:3,3]
        proj_mat = np.dot(K, np.hstack((R, t[:,np.newaxis])))
        xyz_hom = np.hstack((T_world_pc, np.ones((T_world_pc.shape[0], 1))))
        xy_hom = np.dot(proj_mat, xyz_hom.T).T
        z = xy_hom[:, -1]
        z = np.asarray(z).squeeze()
        return z

    def extract(self,point):
        # print(point[0], point[1], point[2])
        return [point[0], point[1], point[2]]
    
    # def project(self,scan):

    def callback(self,scan, image):
        # print("callback")
        # rospy.loginfo("image timestamp: %d ns" % image.header.stamp.to_nsec())
        # rospy.loginfo("scan timestamp: %d ns" % scan.header.stamp.to_nsec())
        # diff = abs(image.header.stamp.to_nsec() - scan.header.stamp.to_nsec())
        # rospy.loginfo("diff: %d ns" % diff)

        # print(f"lidar_len : {(scan.scan_time)} \n camera_len : {len(image.data)}")


        img = self.bridge.imgmsg_to_cv2(image,"bgr8")
        # img = self.bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        # img = self.bridge.imgmsg_to_cv2(image)

        # cloud = self.lp.projectLaser(scan)
        # cloud = scan
    
        # points = pc2.read_points(cloud)
        points = pc2.read_points(scan)


        objPoints = np.array([self.extract(point) for point in points])
        
        
        Z = self.get_z(self.q, objPoints, self.K)
    
        objPoints = objPoints[Z > 0]
        if self.lens == 'pinhole':
            img_points, _ = cv2.projectPoints(objPoints, self.rvec, self.tvec, self.K, self.D)
        # elif self.lens == 'fisheye':
        #     objPoints = np.reshape(objPoints, (1,objPoints.shape[0],objPoints.shape[1]))
        #     img_points, _ = cv2.fisheye.projectPoints(objPoints, self.rvec, self.tvec, self.K, self.D)
        img_points = np.squeeze(img_points)

        for i in range(len(img_points)):
            # print('x:', img_points[i][0], ', y: ', img_points[i][1])
            
            if abs(int(round(img_points[i][0]))) > 1000 or abs(int(round(img_points[i][1]))) >1000 :
                continue
            try:   
                # print('x:', int(round(img_points[i][0])), ', y: ', int(round(img_points[i][1])))
                img = cv2.circle(img, (int(round(img_points[i][0])),int(round(img_points[i][1]))), 
                                 self.laser_point_radius, (0,255,0), 1)
            except OverflowError:
                continue

        self.pub.publish(self.bridge.cv2_to_imgmsg(img,encoding='bgr8'))
        # self.pub.publish(self.bridge.cv2_to_imgmsg(img))

if __name__ == "__main__":

    rospy.init_node("PROJECTION_node")
    node = PROJECTNode()
    rospy.spin()

