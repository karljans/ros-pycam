#!/usr/bin/python3
from inspect import getmembers, isfunction
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
import rospy
import cv2
import sys
import time

from py_camera.test import it_works

LOG_LEVEL = rospy.DEBUG

INFO_TOPIC = '/pycam/camera_info'
IMAGE_TOPIC = '/pycam/image_raw'

class PyCam:
    def __init__(self):
        self.cap = None
        self.frame = None
        self.src_is_gst = False
        self.src_is_file = False

        self.bridge = CvBridge()
        self.info_pub = rospy.Publisher(INFO_TOPIC, CameraInfo, queue_size=10)
        self.img_pub = rospy.Publisher(IMAGE_TOPIC, Image, queue_size=10)

    def setup(self):
        rospy.logdebug_once("PYCAM: Driver Startup")

        video_src = ''

        # Read inputs
        if rospy.has_param('/pycam_node/gst_pipeline'):
            video_src = rospy.get_param('/pycam_node/gst_pipeline')
            rospy.logdebug_once("PYCAM: Gst pipeline")
            self.src_is_gst = True

        elif rospy.has_param('/pycam_node/video_src'):
            video_src = rospy.get_param('/pycam_node/video_src')

            if not '/dev/video' in video_src:
                self.src_is_file = True
                rospy.logdebug_once("PYCAM: file")
            else:
                rospy.logdebug_once("PYCAM: input")

        else:
            video_src = '/dev/video0'
            rospy.logdebug_once("PYCAM: Default input")

        # Input is Pipeline
        if self.src_is_gst:
            self.cap = cv2.VideoCapture(video_src, cv2.CAP_GSTREAMER)
            rospy.loginfo_once(f"PYCAM: Opening pipeline: {video_src}")
            if not self.cap.isOpened():
                rospy.logfatal_once("PYCAM: Cannot open video source!")
                return 1

        # Input is not pipeline
        else:
            self.cap = cv2.VideoCapture(video_src)
            if not self.cap.isOpened():
                rospy.logfatal_once("PYCAM: Cannot open video source!")
                return 1

            # Input is file
            if self.src_is_file:
                rospy.loginfo_once(f"PYCAM: Opening file: {video_src}")

            # Input is camera
            else:
                rospy.loginfo_once(f"PYCAM: Opening camera: {video_src}")

                if rospy.has_param('/pycam_node/width'):
                    width = rospy.get_param('/pycam_node/width')
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)

                if rospy.has_param('/pycam_node/height'):
                    height = rospy.get_param('/pycam_node/height')
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

                if rospy.has_param('/pycam_node/fps'):
                    fps = rospy.get_param('/pycam_node/fps')
                    cap.set(cv2.CAP_PROP_FPS, fps)

        return 0

    def capture(self):
        ret = False

        if self.cap.isOpened():
            ret, self.frame = self.cap.read()

        return ret

    def get_rate(self):
        rate = -1

        if self.src_is_gst and not rospy.has_param('/pycam_node/fps'):
            rospy.logfatal_once("PYCAM: FPS parameter needs to be set when using GStreamer pipeline!")
            return 1

        if rospy.has_param('/pycam_node/fps'):
            rate = rospy.get_param('/pycam_node/fps')
        else:
            rate = self.cap.get(cv2.CAP_PROP_FPS)

        return 1/rate

    def publish(self):

        if self.frame is not None:
            cam_info = CameraInfo()

            stamp = rospy.Time.from_sec(time.time())
            cam_info.header.stamp = stamp

            self.info_pub.publish(cam_info)

            img_msg = Image()
            img_msg.height = self.frame.shape[0]
            img_msg.width = self.frame.shape[1]
            img_msg.step = self.frame.strides[0]
            img_msg.encoding = 'bgr8'
            img_msg.header.frame_id = 'py_cam'
            img_msg.header.stamp = stamp
            img_msg.data = self.frame.flatten().tolist()

            self.img_pub.publish(img_msg)

def node_main():

    rospy.init_node('pycam', log_level=LOG_LEVEL)

    camera = PyCam()

    setup_failed = camera.setup()
    if setup_failed:
        return 1

    rate = camera.get_rate()
    if rate == -1:
        return 1

    rospy.loginfo_once("PYCAM: Started")

    while not rospy.is_shutdown():
        if camera.capture():
            camera.publish()

        rospy.sleep(rate)

    return 0

if __name__ == '__main__':
    it_works()
    sys.exit(node_main())
