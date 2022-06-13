import time

import cv2
import gi
import numpy as np
import rospy

from pycam_lib.log import PyCamLog
from pycam_lib.publisher import Publisher

gi.require_version('Gst', '1.0')
gi.require_version('GstBase', '1.0')
gi.require_version('GstVideo', '1.0')

from gi.repository import GLib, GObject, Gst, GstApp, GstBase, GstVideo

APPSINK_STR = 'appsink name=appsink'

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
                 measure_fps=False, print_fps_values=False, display_fps=False, resize_dim = (None, None)):
        
        self._publisher = publisher
        self._shutdown = False
        self._callback_error = None

        self._resize_dim = resize_dim

        # For FPS calculation
        self._measure_fps = measure_fps
        self._display_fps = display_fps
        self._print_fps_values = print_fps_values 

        self._prev_frame_time = 0
        self._fps_measure_time_start = 0

        self._fps_values = []
        self._last_avg_fps = None

        # Initialize GStreamer
        Gst.init(None)

        # Set up camera
        self._setup(pipeline_str)

    def _setup(self, pipeline_str):

        '''
        Sets up and opens video input and reads back video parameters
        '''
        PyCamLog.info("Begin camera setup")

        # Open GStreamer pipeline
        if not '!' in pipeline_str:
            raise CameraException("Pipeline with only a single component!?!?")

        if pipeline_str.strip()[-1] == '!':
            pipeline_str = pipeline_str.strip()[0:-1].strip()

        pipeline_last_element = pipeline_str.strip().split('!')[-1]
        pipeline_last_element_name = pipeline_last_element.strip().split(' ')[0]
        
        # If the user already put an appsink in the pipeline, then use his/her appsink
        if 'appsink' in pipeline_last_element_name:
            if 'name=' in pipeline_last_element:
                raise CameraException("Please remove 'name=' parameter from appsink in the pipeline")
            else:
                pipeline_str += ' name=appsink'

        # If there is some other sink, then reject the sink in pipeline and substitute it with our own
        elif 'sink' in pipeline_last_element_name:
            pipeline_str = pipeline_str[0:pipeline_str.rfind('!')].strip()
            PyCamLog.warn(f"Removed '{pipeline_last_element_name}' from the pipeline "
                           "An appsink was automatically added")

        # If no appsink was found, add it
        if not 'appsink' in pipeline_last_element_name:
            pipeline_str += f' ! {APPSINK_STR}'

        PyCamLog.info(f"Opening pipeline: {pipeline_str}")
        self._pipeline = Gst.parse_launch(pipeline_str)

        if self._pipeline is None:
            raise CameraException("GStreamer pipeline parsing failed! "
                                  "Please verify that the pipeline string is correct.")

        # Connect the appsink to our program
        appsink = self._pipeline.get_by_name('appsink')

        appsink.set_property('emit-signals', True)
        appsink.connect('new-sample', self._new_sample_callback)

        PyCamLog.debug("Camera setup complete")

    def _calculate_fps(self):
        '''
        Calculates average FPS and prints it on the screen
        '''

        curr_time = time.time()

        fps = round(1 / (curr_time - self._prev_frame_time))
        self._prev_frame_time = curr_time

        self._fps_values.append(fps)

        if curr_time - self._fps_measure_time_start >= FPS_AVG_TIME:
            avg_fps = round(sum(self._fps_values) / len(self._fps_values))

            if self._display_fps:
                self._last_avg_fps = avg_fps

            if self._print_fps_values:
                print(self._fps_values)

            print('AVG FPS:', avg_fps)

            self._fps_values = []
            self._fps_measure_time_start = curr_time

    def _new_sample_callback(self, sink):
        '''
        Processes the sample received from GStreamer and publishes it.
        This function is called by GStreamer when a new sample is available

        Parameters:
            sink: GStreamer sink instance

        Return:
            Gst.FlowReturn.OK
        '''

        if self._measure_fps:
            self._calculate_fps()

        # Get a frame from appsink
        sample = sink.emit('pull-sample')
        buf = sample.get_buffer()
        result, mapinfo = buf.map(Gst.MapFlags.READ)
        
        if result:
            # Get sample's parameters
            caps = sample.get_caps()

            frmt_str = caps.get_structure(0).get_value('format')
            h = caps.get_structure(0).get_value('height')
            w = caps.get_structure(0).get_value('width')

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
                    self._callback_error = f"Unknown image format: {frmt_str}"
                    return Gst.FlowReturn.ERROR

            if None not in self._resize_dim:
                image = cv2.resize(image, self._resize_dim)

            # FPS
            if self._display_fps:
                if self._last_avg_fps:
                    text = f"AVG FPS: {self._last_avg_fps}"
                else:
                    text = "Calculating FPS"

                font = cv2.FONT_HERSHEY_SIMPLEX
                image = cv2.putText(image, text, (7, 70), font, 2, 
                                    (100, 255, 0), 3, cv2.LINE_AA)

            # Publish the image
            if self._publisher is None:
                self._callback_error = f"Unknown image format: {frmt_str}"
                return Gst.FlowReturn.ERROR

            return_value = self._publisher.publish(image, (w, h))
            if return_value is None:
                self._callback_error = "Image dimensions in the calibration file are" \
                                        " different than the dimensions of the image read from GStreamer"
                return Gst.FlowReturn.ERROR
            
            # Clear the buffer
            buf.unmap(mapinfo)

        return Gst.FlowReturn.OK

    def run(self):
        '''
        Starts runs the camera loop
        '''

        PyCamLog.debug("Starting GStreamer main loop")
        
        # For FPS calculation
        curr_time = time.time()
        self._prev_frame_time = curr_time
        self._fps_measure_time_start = curr_time

        # Start the pipeline
        self._pipeline.set_state(Gst.State.PLAYING)
        bus = self._pipeline.get_bus()

        # Monitor the communication bus for messages from GST
        while not self._shutdown:
            message = bus.timed_pop_filtered(10000, Gst.MessageType.ANY)

            if message:
                if message.type == Gst.MessageType.ERROR:
                    err, debug = message.parse_error()
                    PyCamLog.error(f"GST ERROR from element {message.src.get_name()}: {err}")
                    PyCamLog.debug(f"GST ERROR debug info: {debug}")
                    break

                elif message.type == Gst.MessageType.EOS:
                    PyCamLog.info(f"End of stream")
                    self._shutdown = True
                    break

                elif message.type == Gst.MessageType.STATE_CHANGED:
                    if isinstance(message.src, Gst.Pipeline):
                        old_state, new_state, pending_state = message.parse_state_changed()
                        PyCamLog.debug(f"Pipeline state changed from {old_state.value_nick} to {new_state.value_nick}.")

                elif message.type == Gst.MessageType.QOS:
                    qos_msg = message.parse_qos_stats()
                    PyCamLog.debug(f"GST QoS warning (dropped frame?)")

                elif message.type == Gst.MessageType.WARNING:
                    warn, debug = message.parse_warning()
                    PyCamLog.warn(f"GST WARN from element {message.src.get_name()}: {warn}")
                    PyCamLog.debug(f"GST WARN debug info: {debug}")

                elif message.type == Gst.MessageType.INFO:
                    info = message.parse_info()
                    PyCamLog.info(f"GST INFO Message: {info}")

                else:
                    PyCamLog.debug(f"Unprocessed GST message received. MSG Type: {message.type}")

            if self._callback_error:
                self._pipeline.set_state(Gst.State.NULL)
                raise CameraException(self._callback_error)

        # We are exiting. Stop the pipeline
        self._pipeline.set_state(Gst.State.NULL)
        PyCamLog.debug("Camera main loop has stopped")

    def stop(self):
        '''
        Stops the main loop and performs cleanup
        '''

        PyCamLog.debug("Stopping camera main loop")

        self._shutdown = True
        