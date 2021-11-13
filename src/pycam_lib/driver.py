import time

import rospy

from pycam_lib.camera import Camera, CameraException
from pycam_lib.log import PyCamLog
from pycam_lib.publisher import Publisher

# How often to print average FPS
FPS_AVG_TIME = 10

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
        self.__rate = None

        # For FPS calculation
        self.__measure_fps = False
        self.__display_fps = False
        self.__print_fps_values = False 

        self.__prev_frame_time = time.time()
        self.__fps_measure_time_start = time.time()

        self.__fps_values = []
        self.__last_avg_fps = None

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
            pipeline = rospy.get_param('/pycam_node/gst_pipeline')

        else:
            raise DriverException("Cannot find GStreamer pipeline in launch configuration!")

        # Read ROS rate
        if rospy.has_param('/pycam_node/fps'):
            fps = float(rospy.get_param('/pycam_node/fps'))
            self.__rate = 1/rospy.get_param('/pycam_node/fps')
            PyCamLog.info(f"FPS from parameters: {fps}. "
                          f"This results in ROS rate of {round(self.__rate, 2)}.")

        if rospy.has_param('/pycam_node/rate'):
            if self.__rate is not None:
                PyCamLog.warn("Both 'rate' and 'fps' variables specified. "
                              f"FPS parameter will be ignored!")

            self.__rate = float(rospy.get_param('/pycam_node/rate'))
            PyCamLog.info(f"ROS rate from parameters: {self.__rate}. "
                          f"This results in FPS value of {round(1/self.__rate)}.")

            
        if self.__rate is None:
            raise DriverException("Neither rate nor FPS value specified in configuration! "
                                  "Cannot continue. Sorry.")

        # FPS measurement
        if rospy.has_param('/pycam_node/measure_fps'):
            self.__measure_fps = bool(rospy.get_param('/pycam_node/measure_fps'))

        if rospy.has_param('/pycam_node/print_fps_values'):
            self.__print_fps_values = bool(rospy.get_param('/pycam_node/print_fps_values'))

        if rospy.has_param('/pycam_node/display_fps'):
            self.__display_fps = bool(rospy.get_param('/pycam_node/display_fps'))
            self.__measure_fps = bool(rospy.get_param('/pycam_node/measure_fps'))

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
        self.__camera = Camera(publisher, pipeline)

        PyCamLog.debug("Finished driver setup")

    def __calculate_fps(self):
        '''
        Calculates average FPS and prints it on the screen
        '''

        curr_time = time.time()

        fps = round(1 / (curr_time - self.__prev_frame_time))
        self.__prev_frame_time = curr_time

        self.__fps_values.append(fps)

        if curr_time - self.__fps_measure_time_start >= FPS_AVG_TIME:
            avg_fps = round(sum(self.__fps_values) / len(self.__fps_values))

            if self.__display_fps:
                self.__last_avg_fps = avg_fps

            if self.__print_fps_values:
                print(self.__fps_values)

            print('AVG FPS:', avg_fps)

            self.__fps_values = []
            self.__fps_measure_time_start = curr_time

    def get_rate(self):
        '''
        Gets the ROS sleeping rate
        '''

        return self.__rate

    def advance(self):
        '''
        Reads a frame from camera and publishes it to ROS
        '''
        
        if self.__measure_fps:
            self.__calculate_fps()

        if self.__camera.read(self.__last_avg_fps):
            self.__camera.publish()

