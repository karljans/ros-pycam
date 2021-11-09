import rospy
import time
from sensor_msgs.msg import CameraInfo, Image
from pycam_lib.log import PyCamLog
import os

TOPIC_CAM_INFO = 'camera_info'
TOPIC_IMG_RAW = 'image_raw'
TOPIC_IMG_COMPRESSED = 'image_compressed'

class Publisher:
    def __init__(self, topic_prefix='/pycam', queue_size=10, calib_file=''):
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
        img_compressed_topic = os.path.join(topic_path, TOPIC_IMG_COMPRESSED)

        # TODO Compressed video
        self.info_pub = rospy.Publisher(info_topic, CameraInfo, queue_size=queue_size)
        self.img_pub = rospy.Publisher(img_raw_topic, Image, queue_size=queue_size)

        self.camera_info = self.__get_camera_info(calib_file)

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

        img_msg = Image()
        img_msg.height = image.shape[0]
        img_msg.width = image.shape[1]
        img_msg.step = image.strides[0]
        img_msg.encoding = 'bgr8'
        img_msg.header.frame_id = 'py_cam'
        img_msg.header.stamp = stamp
        img_msg.data = image.flatten().tolist()

        self.img_pub.publish(img_msg)

    def __publish_compressed(self, image, stamp): # TODO
        '''
        Publishes raw image in ROS

        Parameters:
            image: OpenCV image to publish in ROS
            stamp: timestamp to write on the image
        '''

        pass

    def publish(self, image):
        '''
        Publishes an image data to ROS topic
        
        Parameters:
            image: OpenCV image to publish in ROS
        '''

        stamp = self.__publish_cam_info()

        self.__publish_raw(image, stamp)
        self.__publish_compressed(image, stamp)
