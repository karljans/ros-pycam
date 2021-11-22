# ROS-PyCam
GStreamer-based camera driver for ROS, implemented in Python.

Tested on ROS Noetic, Ubuntu 20.04

**This project is still work-in-progress**

## Dependencies

**It might not be a complete list, if you find that anything is missing, please open an issue**

The following packages are needed to run the driver:

* GStreamer, with Python support
* Rospy
* Opencv
* Numpy
* libturbojpeg (and PyTurboJPEG for python support)

```
sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-tools python3-gst-1.0 python3-rospy libturbojpeg
```

```
pip3 install numpy python-opencv PyTurboJPEG
```

## Published topics
The driver will publish three topics:

|              Topic                | Description                             |
|-----------------------------------|-----------------------------------------|
| /pycam/image/camera_info          | Camera information and calibration data |
| /pycam/image/image_raw            | Uncompressed image, using BGR colors    |
| /pycam/image/image_raw/compressed | JPEG compressed image                   |

The `/pycam` prefix of the topics can be changed using the `topic_prefix` parameter

## Accepted Color Formats
The native color format of the driver is BGR

However, in addition, the following output color formats are also accepted and automatically converted to BGR using OpenCV's `cvtColor` function:

* RGB
* RGBA
* BGRA
* YUY2

*Normally, the color conversion to BGR is automatically handled by the `videoconvert` element of the pipeline. However, on some embedded platforms, which do not possess hardware accelerated `videoconvert`, running `videoconvert` on CPU can be extremely slow. However, replacing `videoconvert` in the pipeline with `identity` and then using OpenCV's `cvtColor` function for color conversion from YUY2 to BGR can increase the framerate considerably.*

## Launch Parameters

### GStreamer Pipeline

```
gst_pipeline - specifies GStreamer pipeline to process

```

Adding `appsink` in the pipeline is optional. If `appsink` is not specified in the pipeline the driver will add it automatically. However, if an appsink is specified, all its parameters (e.g. `sync=false`, `drop=true`) will work. An exception is the `name` parameter for the `appsink`, which is not supported and will generate an error.

*Currently, any GStreamer sink, which is not `appsink` (e.g. `kmssink`), will be replaced with `appsink` automatically. However, this "feature" will be removed soon, so it advised to either add `appsink` to your pipeline or not specify any sink at all (in which case, the appsink with default parameters will be automatically added).*

### Camera Calibration File

```
calib_file - Path to camera calibration file
```

At this point, the driver can only read camera calibration data YAML format. INI files are not supported.
If this parameter is not specified, camera info and images will still be published, but most fields will be empty.

### Topic Prefix

```
topic_prefix - Allows the user to specify alternative prefix for published topics. (default: /pycam)
```

### Debugging

**Loglevel**

```
loglevel - (int) Specify loglevel (default: 1)
```

Specifies the loglevel Available loglevels are:

| Loglevel | Description |
|----------|-------------|
| 1        |  debug      |
| 2        |  info       |
| 3        |  warning    |
| 4        |  error      |
| 5        |  fatal      |

Currently defaults to `1 - debug`, however, the default might be changed soon to `warning`.

**Publisher Queue Size Specification**

```
queue_size - Size of the publisher queue (defaults to 1)
```

The `queue_size` parameter might be removed soon.

**FPS Measurement**

```
measure_fps      - Takes FPS measurements and prints the average FPS values on screen every 10 seconds

print_fps_values - Also prints a list of FPS values for each individual frame every 10 seconds. 
                   Allows to check the stability of the FPS

display_fps      - Adds the average FPS display to the images
```
