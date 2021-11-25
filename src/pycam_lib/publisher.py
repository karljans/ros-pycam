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

TOPIC_CAM_INFO = 'camera_info'
TOPIC_IMG_RAW = 'image_raw'
TOPIC_IMG_COMPRESSED = 'compressed'

class PublisherException(Exception):
    '''
    Exception raised in case of errors in the publisher code

    Attributes:
        message: explanation for the error
    '''

    def __init__(self, message):
        self.message = message
        super().___init__(self.message)

    def __str__(self):
        return "Publisher Exception: " + str(self.message)

class Publisher:
    def __init__(self, topic_prefix='/pycam/camera', queue_size=1, calib_file=''):
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

        self.camera_info = self._get_camera_info(calib_file)

        self.bridge = CvBridge()

        self.jpeg = TurboJPEG()

    def _get_camera_info(self, calib_file):
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

            try:
                with open(calib_file, 'r') as f:
                    params = yaml.load(f, Loader=yaml.FullLoader)

                    cam_info.height = params['image_height']
                    cam_info.width = params['image_width']
                    cam_info.distortion_model = params['distortion_model']
                    cam_info.K = params['camera_matrix']['data']
                    cam_info.D = params['distortion_coefficients']['data']
                    cam_info.R = params['rectification_matrix']['data']
                    cam_info.P = params['projection_matrix']['data']

            except (yaml.YAMLError, OSError) as err:
                raise PublisherException(f"Cannot read camera calibration file: {err}")

        return cam_info

    def _publish_cam_info(self, dimensions):
        '''
        Updates and publishes camera info as a ROS topic

        Parameters:
            dimensions: a tuple containing  image dimensions. Format is (width, height)

        Return:
            Timestamp for frame synchronization
            None in case of errors
        '''
        
        # Update timestamp
        stamp = rospy.Time.from_sec(time.time())
        self.camera_info.header.frame_id = 'pycam'
        self.camera_info.header.stamp = stamp

        # Publish
        if (self.camera_info.width, self.camera_info.height) == (0, 0):
            self.camera_info.width, self.camera_info.height = dimensions

        elif (self.camera_info.width, self.camera_info.height) != dimensions:
            # Will cause an exception
            return None

        self.info_pub.publish(self.camera_info)

        return stamp

    def _publish_raw(self, image, stamp):
        '''
        Publishes raw image in ROS

        Parameters:
            image: OpenCV image to publish in ROS
            stamp: timestamp to write on the image
        '''

        img_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        img_msg.header.frame_id = 'pycam'
        img_msg.header.stamp = stamp

        self.img_pub.publish(img_msg)

    def _publish_compressed(self, image, stamp):
        '''
        Publishes raw image in ROS

        Parameters:
            image: OpenCV image to publish in ROS
            stamp: timestamp to write on the image
        '''
        compressed_image = np.array(self.jpeg.encode(image)).tostring()

        msg = CompressedImage()
        msg.header.frame_id = 'pycam'
        msg.header.stamp = stamp
        msg.format = "jpeg"
        msg.data = compressed_image

        self.compressed_pub.publish(msg)

    def publish(self, image, dimensions):
        '''
        Publishes an image data to ROS topic
        
        Parameters:
            image: OpenCV image to publish in ROS
            dimensions: a tuple containing  image dimensions. Format is (width, height)
        '''
        stamp = self._publish_cam_info(dimensions)
        if stamp is None:
            return None

        self._publish_raw(image, stamp)
        self._publish_compressed(image, stamp)

        return 0
