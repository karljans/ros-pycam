import rospy
from pycam_lib.camera import Camera, CameraException
from pycam_lib.log import PyCamLog
from pycam_lib.publisher import Publisher
import time


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
        self.camera = None
        self.rate = 0

        self.prev_frame_time = time.time()

        self.__setup()

    def __setup(self):
        '''
        Reads in the parameters and configures the driver for exectution

        Raises:
            DriverException: If errors are encountered
        '''

        PyCamLog.debug("Begin driver setup")

        video_src = ''
        src_is_gst = False
        src_is_file = False
        
        width = 0
        height = 0
        fps = 0

        # Read input parameters
        if rospy.has_param('/pycam_node/gst_pipeline'):
            video_src = rospy.get_param('/pycam_node/gst_pipeline')
            src_is_gst = True
            PyCamLog.info("Source is GStreamer pipeline")

        elif rospy.has_param('/pycam_node/video_src'):
            video_src = rospy.get_param('/pycam_node/video_src')

            if not '/dev/video' in video_src:
                src_is_file = True
                PyCamLog.info("Source is file")

            else:
                PyCamLog.info("Source is input device")

        else:
            video_src = '/dev/video0'
            PyCamLog.info("Source is default input device")

        # Read other configurations
        if rospy.has_param('/pycam_node/width'):
            width = rospy.get_param('/pycam_node/width')

        if rospy.has_param('/pycam_node/height'):
            height = rospy.get_param('/pycam_node/height')

        if rospy.has_param('/pycam_node/fps'):
            fps = rospy.get_param('/pycam_node/fps')

        # GStreamer input needs framerate to be specified in the configuration
        if src_is_gst and fps == 0:
            raise DriverException("GStreamer input needs valid framerate to be specified in the configuration!")

        # Read publisher conf
        calib_file = ''
        topic_prefix = '/pycam/image'
        queue_size = 10
        
        if rospy.has_param('/pycam_node/calib_file'):
            calib_file = rospy.get_param('/pycam_node/calib_file')

        if rospy.has_param('/pycam_node/topic_prefix'):
            topic_prefix = rospy.get_param('/pycam_node/topic_prefix')

        if rospy.has_param('/pycam_node/queue_size'):
            queue_size = rospy.get_param('/pycam_node/queue_size')

        # Setup the publisher
        publisher = Publisher(topic_prefix, queue_size, calib_file)

        # Setup the camera
        self.camera = Camera(publisher, video_src, src_is_gst, src_is_file, width, height, fps)

        # Ros sleeping rate
        actual_fps = self.camera.get_fps()
        self.rate = 1.0 / actual_fps

        PyCamLog.debug("Finish driver setup")

    def get_rate(self):
        '''
        Gets the ROS sleeping rate
        '''

        return self.rate

    def advance(self):
        '''
        Reads a frame from camera and publishes it to ROS
        '''
        curr_counter_time = time.time()
        fps = str(int(1 / (curr_counter_time - self.prev_frame_time)))
        self.prev_frame_time = curr_counter_time

        time_start = time.time()
        if self.camera.read(fps):
            self.camera.publish()
        print(int(1/(time.time()-time_start)))
