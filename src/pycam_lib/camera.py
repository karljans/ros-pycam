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
    
    def __init__(self, publisher=None, pipeline=None):
        self.__pipeline = pipeline

        self.__cap = None
        self.__frame = None

        self.__publisher = publisher

        self.__enable_counter = True
        self.__prev_frame_time = time.time()
        
        # Set up camera
        self.__setup()

    def __setup(self):

        '''
        Sets up and opens video input and reads back video parameters
        '''
        PyCamLog.info("Begin camera setup")

        # Open GStreamer pipeline
        PyCamLog.info(f"Opening pipeline: {self.__pipeline}")
        self.__cap = cv2.VideoCapture(self.__pipeline, cv2.CAP_GSTREAMER)

        if not self.__cap.isOpened():
            raise CameraException("Cannot open video source!")

        # Read one frame to verify that everyhing is working and to get actual video parameters
        ret, frame = self.__cap.read()
        if not ret:
            raise CameraException('Cannot read from opened input!')

        PyCamLog.debug("Camera setup complete")

    def __counter_start(self):
        self.__prev_frame_time = time.time()

    def __counter_stop(self):
        curr_counter_time = time.time()
        fps = str(int(1 / (curr_counter_time - self.__prev_frame_time)))
        return fps

    def get_fps(self):
        '''
        Returns FPS value for the opened video
        '''

        return self.fps

    def read(self, fps=None):
        '''
        Reads a frame from camera

        return:
            True if reading was successful, False otherwise
        '''
        ret = False
        
        if self.__cap.isOpened():
            ret, self.__frame = self.__cap.read()

        # If an fps value was provided, put it on the video
        if fps:
            font = cv2.FONT_HERSHEY_SIMPLEX
            text = f"AVG FPS: {fps}"
            self.__frame = cv2.putText(self.__frame, text, (7, 70), font, 2, 
                                      (100, 255, 0), 3, cv2.LINE_AA)
        return ret

    def publish(self):
        if self.__publisher is None:
            raise CameraException("Publisher is None!")

        self.__publisher.publish(self.__frame)