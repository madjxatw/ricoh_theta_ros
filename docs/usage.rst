Usage
=====

Only having completed all installations mentioned in the :doc:`prereq` and
optionally in the :doc:`hwaccel` can you proceed with ricoh_theta_ros.

Build ricoh_theta_ros:

.. code-block:: sh

   # Replace </path/to/your/ros/workspace> with the exact path of the ROS
   # workspace on your system.
   cd </path/to/your/ros/workspace>
   git -C src clone --recursive https://github.com/madjxatw/ricoh_theta_ros.git
   # Build and install all deps

   # build ricoh_theta_ros
   catkin_make

Run:

.. code-block:: sh

   # Replace </path/to/your/ros/workspace> with the exact path of the ROS
   # workspace on your system.
   # Assuming a roscore process is running.
   source </path/to/your/ros/workspace>/devel/setup.bash
   rosrun ricoh_theta_ros start.sh

Startup Script
--------------

The :file:`start.sh` file is a startup script that:

- checks whether the ``v4l2loopback`` kernel module has been loaded to make sure
  that a virtual v4l2 looopback device is available.
- wakes the camera up and set it to live streaming mode.
- runs ``gst_loopback`` in background.
- runs ``roslaunch`` to load the specified launch file with appropriate
  parameters.

You can take it as an example, and develop your own launch files or startup
scripts if the default is not satisfying.

Manual Procedure
----------------

Instead of using the launch script, you can start all involved stuff manually:

- If ``v4l2loopback`` module hasn't been loaded, run:

  .. code-block:: sh

      # change video_nr accordingly
      sudo modprobe v4l2loopback video_nr=2

- Open a new terminal and run ``gst_loopback --format 2K`` (change ``2K``
  (1920x960) to ``4K`` (3840x1920) if you want 4K resolution)
- Open a new terminal and run:

   .. code-block:: sh

      source </path/to/your/ros/workspace>/devel/setup.bash
      # device_id here must be as same as supplied to v4l2loopback module
      roslaunch </path/to/your/ros/workspace>/src/scripts/start.sh device_id:=2

.. _opencv-video-capture-support:

OpenCV Video Capture Support
----------------------------

Running the cv_camera node on Ubuntu 18.04 with OpenCV 3.2.0 installed, you will
encounter an error stating:

   VIDEOIO ERROR: V4L2: Pixel format of incoming image is unsupported by OpenCV

This is because the default pixel format of the decoded video output by the
GStreamer pipeline is ``I420`` (i.e. ``YUV420``) which is not supported by
OpenCV Video I/O module before 3.4.3 version. All pixel formats supported by
OpenCV 3.2.0 can be found in `modules/videoio/cap_v4l.cpp
<https://github.com/opencv/opencv/blob/3.2.0/modules/videoio/src/cap_v4l.cpp>`_,
the involved code snippet is shown as below:

.. code-block:: cpp

   static int autosetup_capture_mode_v4l2(CvCaptureCAM_V4L* capture) {
       //in case palette is already set and works, no need to setup.
       if(capture->palette != 0 and try_palette_v4l2(capture)){
           return 0;
       }
       __u32 try_order[] = {
               V4L2_PIX_FMT_BGR24,
               V4L2_PIX_FMT_YVU420,
               V4L2_PIX_FMT_YUV411P,
   #ifdef HAVE_JPEG
               V4L2_PIX_FMT_MJPEG,
               V4L2_PIX_FMT_JPEG,
   #endif
               V4L2_PIX_FMT_YUYV,
               V4L2_PIX_FMT_UYVY,
               V4L2_PIX_FMT_SN9C10X,
               V4L2_PIX_FMT_SBGGR8,
               V4L2_PIX_FMT_SGBRG8,
               V4L2_PIX_FMT_RGB24,
               V4L2_PIX_FMT_Y16
       };

The same function from OpenCV `3.4.3
<https://github.com/opencv/opencv/blob/3.4.3/modules/videoio/src/cap_v4l.cpp>`_

.. code-block:: c++
   :emphasize-lines: 10

   static int autosetup_capture_mode_v4l2(CvCaptureCAM_V4L* capture) {
       //in case palette is already set and works, no need to setup.
       if(capture->palette != 0 and try_palette_v4l2(capture)){
           return 0;
       }
       __u32 try_order[] = {
               V4L2_PIX_FMT_BGR24,
               V4L2_PIX_FMT_RGB24,
               V4L2_PIX_FMT_YVU420,
               V4L2_PIX_FMT_YUV420,
               V4L2_PIX_FMT_YUV411P,
               V4L2_PIX_FMT_YUYV,
               V4L2_PIX_FMT_UYVY,
               V4L2_PIX_FMT_SBGGR8,
               V4L2_PIX_FMT_SGBRG8,
               V4L2_PIX_FMT_SN9C10X,
   #ifdef HAVE_JPEG
               V4L2_PIX_FMT_MJPEG,
               V4L2_PIX_FMT_JPEG,
   #endif
               V4L2_PIX_FMT_Y16
       };

The pull request for adding YUV420 to OpenCV 3.4.3 can be found at
https://github.com/opencv/opencv/pull/12134

The simplest workaround for this issue is to tweak the GStreamer pipeline to
have it use ``YV12`` (i.e. ``YVU420``) as pixel format for the decoded video,
and this can be done by modifying :file:`gst/gst_viewer.c` in
libuvc-theta-sample:

.. code-block:: diff

   diff --git a/gst/gst_viewer.c b/gst/gst_viewer.c
   index 92d4fb9..659197f 100644
   --- a/gst/gst_viewer.c
   +++ b/gst/gst_viewer.c
   @@ -186,7 +186,7 @@ main(int argc, char **argv)

           if (strcmp(cmd_name, "gst_loopback") == 0)
                   pipe_proc = "decodebin ! autovideoconvert ! "
   -                       "video/x-raw,format=I420 ! identity drop-allocation=true !"
   +                       "video/x-raw,format=YV12 ! identity drop-allocation=true !"
                           "v4l2sink device=/dev/video2 qos=false sync=false";
           else
                   pipe_proc = " decodebin ! autovideosink sync=false";

.. seealso::

   :ref:`Install libuvc-theta-sample <install-libuvc-theta-sample>`
