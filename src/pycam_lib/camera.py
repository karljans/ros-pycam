import time

import cv2
import gi
import rospy

import numpy as np

from pycam_lib.log import PyCamLog
from pycam_lib.publisher import Publisher

gi.require_version('Gst', '1.0')
gi.require_version('GstBase', '1.0')
gi.require_version('GstVideo', '1.0')

from gi.repository import GLib, GObject, Gst, GstBase, GstVideo

APPSINK_STR = 'appsink name=appsink sync=false'

# How often to print average FPS
FPS_AVG_TIME = 10

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
    
    def __init__(self, publisher=None, pipeline_str=None, 
                 measure_fps=False, print_fps_values=False, display_fps=False):
        
        self.__pipeline_str = pipeline_str
        self.__publisher = publisher

        # For FPS calculation
        self.__measure_fps = measure_fps
        self.__display_fps = display_fps
        self.__print_fps_values = print_fps_values 

        self.__prev_frame_time = 0
        self.__fps_measure_time_start = 0

        self.__fps_values = []
        self.__last_avg_fps = None

        # Initialize GStreamer
        Gst.init(None)
        __gst_mainloop = None
        __pipeline = None

        # Set up camera
        self.__setup()

    def __setup(self):

        '''
        Sets up and opens video input and reads back video parameters
        '''
        PyCamLog.info("Begin camera setup")

        # Open GStreamer pipeline
        if not '!' in self.__pipeline_str:
            raise CameraException("Pipeline with only a single component!?!?")

        if self.__pipeline_str.strip()[-1] == '!':
            self.__pipeline_str = self.__pipeline_str.strip()[0:-1].strip()

        pipeline_last_element_name = self.__pipeline_str.strip().split('!')[-1].strip().split(' ')[0]
        
        # Reject the sink in pipeline and substitute it with our own
        if 'sink' in pipeline_last_element_name:
            self.__pipeline_str = self.__pipeline_str[0:self.__pipeline_str.rfind('!')].strip()
            PyCamLog.warn(f"Removed '{pipeline_last_element_name}' from the pipeline "
                           "as a properly configured appsink will be automatically added")

        self.__pipeline_str += f' ! {APPSINK_STR}'

        PyCamLog.info(f"Opening pipeline: {self.__pipeline_str}")
        self.__pipeline = Gst.parse_launch(self.__pipeline_str)

        if self.__pipeline is None:
            raise CameraException("GStreamer pipeline parsing failed! "
                                  "Please verify that the pipeline string is correct.")

        # Connect the appsink to our program
        appsink = self.__pipeline.get_by_name('appsink')
        appsink.set_property('emit-signals', True)
        appsink.connect('new-sample', self.__new_sample_callback)

        # Create GStreamer's main loop
        self.__gst_mainloop = GObject.MainLoop.new(None, False)

        PyCamLog.debug("Camera setup complete")

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

    def __new_sample_callback(self, sink):
        '''
        Processes the sample received from GStreamer and publishes it.
        This function is called by GStreamer when a new sample is available

        Parameters:
            sink: GStreamer sink instance

        Return:
            Gst.FlowReturn.OK
        '''

        if self.__measure_fps:
            self.__calculate_fps()

        # Get a frame from appsink
        sample = sink.emit('pull-sample')
        buf = sample.get_buffer()
        result, mapinfo = buf.map(Gst.MapFlags.READ)
        
        if result:
            # Get sample's parameters
            caps_format = sample.get_caps().get_structure(0)

            frmt_str = caps_format.get_value('format') 

            w, h = caps_format.get_value('width'), caps_format.get_value('height')

            # print(frmt_str)
            # Assemble an image from the buffer
            image = np.frombuffer(mapinfo.data, np.uint8)
            image = np.reshape(image, [h, w,  -1])

            # The native format is BGR.
            # However, we support converting some formats in SW.
            if frmt_str != 'BGR':
                if frmt_str == 'RGB':
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                elif frmt_str == 'RGBA':
                    image = image[:,:,0:3]
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                elif frmt_str == 'BGRA':
                    image = image[:,:,0:3]

                elif frmt_str == 'YUY2':
                    image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2)

                else:
                    raise CameraException("Unknown format!")
                    self.stop()
                    return Gst.FlowReturn.ERROR

            if self.__display_fps:
                if self.__last_avg_fps:
                    text = f"AVG FPS: {self.__last_avg_fps}"
                else:
                    text = "Calculating FPS"

                font = cv2.FONT_HERSHEY_SIMPLEX
                image = cv2.putText(image, text, (7, 70), font, 2, 
                                    (100, 255, 0), 3, cv2.LINE_AA)

            # Publish the image
            if self.__publisher is None:
                
                # Since we are in the callback function, 
                # ROS printing does not work.
                print("PYCAM: ERROR: Publisher is None!")
                raise CameraException("Publisher is None!")
                return Gst.FlowReturn.ERROR

            self.__publisher.publish(image)

        return Gst.FlowReturn.OK

    def start(self):
        '''
        Starts GStreamer's main loop
        '''
        
        curr_time = time.time()
        self.__prev_frame_time = curr_time
        self.__fps_measure_time_start = curr_time

        PyCamLog.debug("Starting GStreamer main loop")
        self.__pipeline.set_state(Gst.State.PLAYING)
        self.__gst_mainloop.run()

        PyCamLog.debug("GStreamer main loop has stopped")

    def stop(self):
        '''
        Stops GStreamer's main loop and performs cleanup
        '''

        PyCamLog.debug("Stopping GStreamer main loop")

        self.__gst_mainloop.quit()
        self.__pipeline.set_state(Gst.State.NULL)
