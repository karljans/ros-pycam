import os
import signal
import time

import rospy

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
        self._camera = None

        signal.signal(signal.SIGINT, self._sigint_handler)
        signal.signal(signal.SIGTERM, self._sigterm_handler)

        # Setup the driver
        self._setup()

    def _setup(self):
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

        # Resize
        resize_width = None
        resize_height = None
        should_resize = False

        if rospy.has_param('/pycam_node/resize_width'):
            resize_width = int(rospy.get_param('/pycam_node/resize_width'))

        if rospy.has_param('/pycam_node/resize_height'):
            resize_height = int(rospy.get_param('/pycam_node/resize_height'))

        resize_dim = tuple([resize_width, resize_height])

        # Read publisher conf
        calib_file = ''
        topic_prefix = '/pycam'
        queue_size = 10
        compress = False
        frame_id = 'pycam'
        
        if rospy.has_param('/pycam_node/calib_file'):
            calib_file = str(rospy.get_param('/pycam_node/calib_file'))

        if rospy.has_param('/pycam_node/topic_prefix'):
            topic_prefix = str(rospy.get_param('/pycam_node/topic_prefix'))

        if rospy.has_param('/pycam_node/queue_size'):
            queue_size = str(rospy.get_param('/pycam_node/queue_size'))

        if rospy.has_param('/pycam_node/publish_compressed'):
            compress = bool(rospy.get_param('/pycam_node/publish_compressed'))

        if rospy.has_param('/pycam_node/frame_id'):
            frame_id = str(rospy.get_param('/pycam_node/frame_id'))

        # Setup the publisher
        publisher = Publisher(os.path.join(topic_prefix, 'camera'), 
                              queue_size, calib_file, compress, frame_id)

        # Setup the camera
        self._camera = Camera(publisher, pipeline, measure_fps, print_fps_values, display_fps, resize_dim)

        PyCamLog.debug("Finished driver setup")

    def _sigint_handler(self, sig, frame):
        '''
        Stops the GStreamer's main loop when a SIGINT is received
        '''
        PyCamLog.debug("Received SIGINT, stopping")
        self._camera.stop()

    def _sigterm_handler(self, sig, frame):
        '''
        Stops the GStreamer's main loop when a SIGTERM is received
        '''
        PyCamLog.debug("Received SIGTERM, stopping")
        self._camera.stop()

    def run(self):
        '''
        Starts the camera
        '''
        self._camera.run()



