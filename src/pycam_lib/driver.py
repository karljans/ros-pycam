import time

import rospy
import signal

from pycam_lib.camera import Camera, CameraException
from pycam_lib.log import PyCamLog
from pycam_lib.publisher import Publisher

class DriverException(Exception):
    '''
    Exception raised in case of errors in the driver code

    Attributes:
        message: explanation for the error
    '''

    def __init__(self, message):
        self.message = message
        super().__init__(self.message)

    def __str__(self):
        return "Driver Exception: " + str(self.message)


class Driver:
    '''
    Class, which implements the top level camera driver code
    '''

    def __init__(self):
        self.__camera = None

        signal.signal(signal.SIGINT, self.__sigint_handler)
        signal.signal(signal.SIGTERM, self.__sigterm_handler)

        # Setup the driver
        self.__setup()

    def __setup(self):
        '''
        Reads in the parameters and configures the driver for exectution

        Raises:
            DriverException: If errors are encountered
        '''

        PyCamLog.debug("Begin driver setup")

        pipeline = None
        
        # Read GSt pipeline
        if rospy.has_param('/pycam_node/gst_pipeline'):
            pipeline = str(rospy.get_param('/pycam_node/gst_pipeline'))

        else:
            raise DriverException("Cannot find GStreamer pipeline in launch configuration!")

        # FPS measurement
        measure_fps = False
        print_fps_values = False
        display_fps = False

        if rospy.has_param('/pycam_node/measure_fps'):
            measure_fps = bool(rospy.get_param('/pycam_node/measure_fps'))

        if rospy.has_param('/pycam_node/print_fps_values'):
            print_fps_values = bool(rospy.get_param('/pycam_node/print_fps_values'))

        if rospy.has_param('/pycam_node/display_fps'):
            display_fps = bool(rospy.get_param('/pycam_node/display_fps'))
            measure_fps = bool(rospy.get_param('/pycam_node/measure_fps'))

        # Read publisher conf
        calib_file = ''
        topic_prefix = '/pycam/image'
        queue_size = 10
        
        if rospy.has_param('/pycam_node/calib_file'):
            calib_file = str(rospy.get_param('/pycam_node/calib_file'))

        if rospy.has_param('/pycam_node/topic_prefix'):
            topic_prefix = str(rospy.get_param('/pycam_node/topic_prefix'))

        if rospy.has_param('/pycam_node/queue_size'):
            queue_size = str(rospy.get_param('/pycam_node/queue_size'))

        # Setup the publisher
        publisher = Publisher(topic_prefix, queue_size, calib_file)

        # Setup the camera
        self.__camera = Camera(publisher, pipeline, measure_fps, print_fps_values, display_fps)

        PyCamLog.debug("Finished driver setup")

    def __sigint_handler(self, sig, frame):
        '''
        Stops the GStreamer's main loop when a SIGINT is received
        '''
        PyCamLog.debug("Received SIGINT, stopping")
        self.__camera.stop()

    def __sigterm_handler(self, sig, frame):
        '''
        Stops the GStreamer's main loop when a SIGTERM is received
        '''
        PyCamLog.debug("Received SIGTERM, stopping")
        self.__camera.stop()

    def run(self):
        '''
        Starts the camera
        '''
        self.__camera.start()


