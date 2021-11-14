import os
import time

import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from turbojpeg import TJPF_RGBA, TurboJPEG

from pycam_lib.log import PyCamLog
# from pycam_lib.driver import DriverException

TOPIC_CAM_INFO = 'camera_info'
TOPIC_IMG_RAW = 'image_raw'
TOPIC_IMG_COMPRESSED = 'compressed'

class Publisher:
    def __init__(self, topic_prefix='/pycam', queue_size=1, calib_file=''):
        '''
        Publishes camera frames and data as ROS topics

        Parameters:
            topic_prefix: prefix to be used in front of camera topics
            queue_size: size of the queue for publishers
            calib_file: file containing camera calibration data
        '''

        topic_path = topic_prefix if topic_prefix[0] == '/' else f'/{topic_prefix}'

        info_topic = os.path.join(topic_path, TOPIC_CAM_INFO)
        img_raw_topic = os.path.join(topic_path, TOPIC_IMG_RAW)
        img_compressed_topic = os.path.join(img_raw_topic, TOPIC_IMG_COMPRESSED)

        self.info_pub = rospy.Publisher(info_topic, CameraInfo, queue_size=queue_size)
        self.img_pub = rospy.Publisher(img_raw_topic, Image, queue_size=queue_size)
        self.compressed_pub = rospy.Publisher(img_compressed_topic, CompressedImage, queue_size=queue_size)

        self.camera_info = self.__get_camera_info(calib_file)

        self.bridge = CvBridge()

        self.jpeg = TurboJPEG()

    def __get_camera_info(self, calib_file):
        '''
        Reads the camera calibration file and outputs a CameraInfo object
        If calib_file is an empty string, an empty CameraInfo object is returned

        Parameters:
            calib_file: yaml file with camera calibrations

        Return:
            returns a CameraInfo object, containing the camera calibration data
        '''
        
        PyCamLog.debug("Camera info extraction")
            
        cam_info = CameraInfo()

        if calib_file != '':

            calib_file = os.path.abspath(os.path.expanduser(calib_file))

            # if not os.path.isfile(calib_file):
            #     raise DriverException("Calibration file does not exist!")

            # if not os.access(calib_file, os.R_OK):
            #     raise DriverException("Cannot read calibration file!")

            with open(calib_file, 'r') as f:
                params = yaml.load(f)

                cam_info.height = params['image_height']
                cam_info.width = params['image_width']
                cam_info.distortion_model = params['distortion_model']
                cam_info.K = params['camera_matrix']['data']
                cam_info.D = params['distortion_coefficients']['data']
                cam_info.R = params['rectification_matrix']['data']
                cam_info.P = params['projection_matrix']['data']

        return cam_info

    def __publish_cam_info(self):
        '''
        Updates and publishes camera info as a ROS topic

        Return:
            Timestamp for frame synchronization
        '''
        
        # Update timestamp
        stamp = rospy.Time.from_sec(time.time())
        self.camera_info.header.stamp = stamp

        # Publish
        self.info_pub.publish(self.camera_info)

        return stamp

    def __publish_raw(self, image, stamp):
        '''
        Publishes raw image in ROS

        Parameters:
            image: OpenCV image to publish in ROS
            stamp: timestamp to write on the image
        '''

        img_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        img_msg.header.frame_id = 'py_cam'
        img_msg.header.stamp = stamp

        self.img_pub.publish(img_msg)

    def __publish_compressed(self, image, stamp): # TODO
        '''
        Publishes raw image in ROS

        Parameters:
            image: OpenCV image to publish in ROS
            stamp: timestamp to write on the image
        '''
        compressed_image = np.array(self.jpeg.encode(image)).tostring()

        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.format = "jpeg"
        msg.data = compressed_image

        self.compressed_pub.publish(msg)

    def publish(self, image):
        '''
        Publishes an image data to ROS topic
        
        Parameters:
            image: OpenCV image to publish in ROS
        '''

        stamp = self.__publish_cam_info()

        self.__publish_raw(image, stamp)
        self.__publish_compressed(image, stamp)
