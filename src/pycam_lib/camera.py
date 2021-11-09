import rospy
import cv2
import time

from pycam_lib.log import PyCamLog
from pycam_lib.publisher import Publisher


class CameraException(Exception):
    '''
    Exception raised in case of errors in the camera code

    Attributes:
        message: explanation for the error
    '''

    def __init__(self, message):
        self.message = message
        super().__init__(self.message)

    def __str__(self):
        return "Camera Exception: " + self.message


class Camera:
    '''
    Class, which implements actual OpenCV and ROS communication

    Parameters:
        publisher: Instance of pycam_lib.publisher.Publisher class
        video_src: String containing either the path to the video device/file or GStreamer pipeline
        src_is_gst: If True, it marks that video_src should be treated as GStreamer pipeline
        src_is_file: If True, it marks that source should be treated as a video file
        width: Int specifying the width of the video. It is only applied to cameras.
                If width == 0, then camera's default value is used.
        height: Int specifying the height of the video. It is only applied to cameras.
                If height == 0, then camera's default value is used.
        fps: Int specifying the FPS (frames per second) value for the video. It is only applied to cameras.
                If fps == 0, then camera's default value is used.
        
    Raises:
        CameraException: In case of errors in the camera class
    '''
    
    def __init__(self, publisher=None, video_src=None, src_is_gst=None, src_is_file=None, 
                 width=0, height=0, fps=0):
        self.video_src = video_src
        self.src_is_gst = src_is_gst
        self.src_is_file = src_is_file

        self.width = width
        self.height = height
        self.fps = fps

        self.cap = None
        self.frame = None

        self.publisher = publisher

        self.enable_counter = True
        self.prev_frame_time = time.time()
        
        self.__setup()

    def __setup(self):

        '''
        Sets up and opens video input and reads back video parameters
        '''
        PyCamLog.info("Begin camera setup")

        # Input is Pipeline
        if self.src_is_gst:

            # Open GStreamer pipeline
            PyCamLog.info(f"Opening pipeline: {self.video_src}")
            self.cap = cv2.VideoCapture(self.video_src, cv2.CAP_GSTREAMER)

            if not self.cap.isOpened():
                raise CameraException("Cannot open video source!")

        # Input is not pipeline
        else:
            PyCamLog.info(f"Opening video source: {self.video_src}")
            self.cap = cv2.VideoCapture(self.video_src)

            if not self.cap.isOpened():
                raise CameraException("Cannot open video source!")

            # Input is file
            if not self.src_is_file:

                if self.width > 0:
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)

                if self.height > 0:
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

                if self.fps > 0:
                    self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # Read one frame to verify that everyhing is working and to get actual video parameters
        ret, frame = self.cap.read()
        if not ret:
            raise CameraException('Cannot read from opened input!')

        # Read size
        self.height, self.width, _ = frame.shape

        # If we are reading file, scroll back to first frame
        if self.src_is_file:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 1)

        # Read the actual FPS value of the video (won't work)
        if not self.src_is_gst:
            self.fps = self.cap.get(cv2.CAP_PROP_FPS)

        PyCamLog.debug("Camera setup complete")
        PyCamLog.debug(f"Video Parameters are the following: {self.width}x{self.height}@{self.fps}")

    def __counter_start(self):
        self.prev_frame_time = time.time()

    def __counter_stop(self):
        curr_counter_time = time.time()
        fps = str(int(1 / (curr_counter_time - self.prev_frame_time)))
        return fps

    def get_fps(self):
        '''
        Returns FPS value for the opened video
        '''

        return self.fps

    def read(self, fps):
        '''
        Reads a frame from camera

        return:
            True if reading was successful, False otherwise
        '''
        ret = False
        
        if self.cap.isOpened():
            ret, self.frame = self.cap.read()

        font = cv2.FONT_HERSHEY_SIMPLEX
        self.frame = cv2.putText(self.frame, fps, (7, 70), font, 3, (100, 255, 0), 3, cv2.LINE_AA)
        return ret

    def publish(self):
        if self.publisher is None:
            raise CameraException("Publisher is None!")

        self.publisher.publish(self.frame)