Prerequisites
=============

First of all the RICOH THETA V/Z1 camera must be driven successfully at the
operating system level. Live streaming on Ubuntu requires three components: a
device driver, a streaming sample application, and a virtual (dummy) V4L2
loopback device, which can be obtained respectively from the following three
projects:

- libuvc-theta_
- libuvc-theta-sample_ (original, but recommend using a :ref:`forked version
  <libuvc-theta-sample-fork>`)
- v4l2loopback_


These dependencies are shipped as submodules of the ricoh_theta_ros package.
You don't have to download these code repositories manually, instead run the
following commands to pull all submodules along with the main repository:

.. code-block:: sh
   :name: git-clone-ricoh-theta-ros

   git clone --recursive https://github.com/madjxatw/ricoh_theta_ros

All submodules are placed in the :file:`deps` subdirectory.

libuvc-theta
------------

UVC
   USB Video Class is a USB device class that describes devices capable of
   streaming videos like webcams, digital camcoders, transcoders, analog video
   converters and still-image cameras. UVC version 1.5 is its latest
   specification.

libuvc
   libuvc is a cross-platform library for USB video devices, built atop libusb.
   It enables fine-grained control over USB video devices exporting the standard
   USB Video Class (UVC) interface, enabling developers to write drivers for
   previously unsupported devices, or just access UVC devices in a generic
   fashion.

libuvc-theta
   Detection of UVC 1.5 devices was introduced in Linux kernel version 4.5, but
   support in the driver for UVC 1.5 specific features or specific UVC 1.5
   devices was not added. libuvc-theta is a RICOH patched UVC library that
   added UVC 1.5 support, which is necessary to get RICOH THETA V/Z1 cameras
   to work on Linux.

libuvc can be built and installed by following the steps below:

.. code-block:: sh

   # install build dependencies
   sudo apt-get install build-essential cmake libjpeg-dev
   # clone libuvc-theta repo
   git clone https://github.com/ricohapi/libuvc-theta.git
   # navigation into the local repo directory
   cd libuvc-theta
   # check out the theta_uvc branch
   git checkout theta_uvc
   # build and install
   mkdir build && cd build
   cmake ..
   make
   sudo make install

.. _install-libuvc-theta-sample:

libuvc-theta-sample
-------------------

The libuvc-theta-sample project provides an application based on GStreamer that
samples and decodes video streaming from the camera. The application writes the
decoded video data to a virtual v4l2 loopback device (created by v4l2loopback
kernel module) so that any application built with v4l2 features can use the
video data by directly reading the v4l2 loopback device.

To build libuvc-theta-sample, the development packages for GStreamer 1.0 need to
be installed. On Ubuntu 18.04 and 20.04, they can be installed as follows:

.. code-block:: sh

   sudo apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base \
   gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
   gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x \
   gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 \
   gstreamer1.0-pulseaudio libgstreamer-plugins-base1.0-dev

The source code sets the default device path for the virtual v4l2 loopback
device to :file:`/dev/video2`, assuming there is a non-THETA camera (laptop or
desktop cam) already installed on your system taking :file:`/dev/video0`
and :file:`/dev/video1`. You need to change the device path for the v4l2
loopback device in the source code to the exact path created by the
``v4l2loopback`` kernel module on your machine.

Suppose the ``v4l2loopback`` module created a v4l2 loopback device at
:file:`/dev/video3`, then you need to make the following modifications to
:file:`gst/gst_viewer.c`:

.. code-block:: diff

   diff --git a/gst/gst_viewer.c b/gst/gst_viewer.c
   index 92d4fb9..6f721ee 100644
   --- a/gst/gst_viewer.c
   +++ b/gst/gst_viewer.c
   @@ -187,7 +187,7 @@ main(int argc, char **argv)
           if (strcmp(cmd_name, "gst_loopback") == 0)
                   pipe_proc = "decodebin ! autovideoconvert ! "
                           "video/x-raw,format=I420 ! identity drop-allocation=true !"
   -                       "v4l2sink device=/dev/video2 qos=false sync=false";
   +                       "v4l2sink device=/dev/video3 qos=false sync=false";
           else
                   pipe_proc = " decodebin ! autovideosink sync=false";


.. tip::

   In fact, we can specify the device path when loading the ``v4l2loopback``
   module. See :ref:`next section <v4l2loopback-video-nr>` for howto.

.. attention::

   The default pixel format set for the decoded video is hardcoded to ``I420``
   (``YUV420``). If your OS is Ubuntu 18.04 with OpenCV 3.2.0 installed, you
   will have to change ``I420`` to some other format that is supported by the
   Video I/O module of OpenCV 3.2.0, e.g. ``YV12`` (``YVU420``). See
   :ref:`opencv-video-capture-support` for details.

For system using NVIDIA GPUs, hardware acceleration could be achieved by using
NVIDIA decoder as a plug-in of the GStreamer pipeline. See :doc:`hwaccel` for
the howto.

.. _libuvc-theta-sample-fork:

A `forked libuvc-theta-sample
<https://github.com/madjxatw/libuvc-theta-sample>`_ repo has several branches
pre-created for different configurations, e.g. the ``nvdec`` branch uses nvdec
as GStreamer plugin for :doc:`hardware acceleration <hwaccel>`; the ``yv12``
branch uses YVU420 instead of the original YUV420 as pixel format; the
``nvdec-yv12`` combines the previous two. Check out the branch that fits your
case, or create a new branch with your particular needs.

With all necessary dependencies having been installed, libuvc-theta-sample can
be built by following the steps below:

.. code-block:: sh

   git clone https://github.com/madjxatw/libuvc-theta-sample.git
   cd libuvc-theta-sample/gst
   # On Ubuntu 18.04 (OpenCV v3.2) with nvdec installed
   checkout nvdec-yv12
   make

A successful build produces two new files:

- :file:`gst/gst_view` which is used for testing purpose
- :file:`gst/gst_loopback` which actually is a symlink to :file:`gst/gst_view`
  for normal use

Copy both of them to a system binary path that is listed in :envvar:`PATH`
environment variable so that you can later run them anywhere.
:file:`/usr/local/bin/` is recommended if you have root privilege, otherwise
:file:`~/.local/bin/`.

v4l2loopback
------------

v4l2loopback is a kernel module that allows you to create v4l2 "virtual video
devices". Normal v4l2-based applications will read these devices as if they were
ordinary video devices, but the video are not read directly from a physical
device, e.g. a capture card, instead it is generated by another application. In
our case, the decoded video is generated by the ``gst_loopback`` program that
was previously built from libuvc-theta-sample, and is written to the v4l2
virtual loopback device created by the ``v4l2loopback`` kernel module.

The installation of the ``v4l2loopback`` module can be done as follows:

.. code-block:: sh

   # install dependencies
   sudo apt-get install linux-header-`uname -r`
   # clone the repo
   git clone https://github.com/umlaeute/v4l2loopback
   # build and install
   cd v4l2loopback
   make && sudo make install
   # rebuild module map files
   sudo depmod -a

To load the ``v4l2loopback``, run:

.. code-block:: sh

   sudo modprobe v4l2loopback

.. _v4l2loopback-video-nr:

Then a new device will be created at :file:`/dev/video{N}`, where ``N`` differs
depending on whether there is other cameras already installed to your system.
However, the value of ``N`` can be manually specified as long as the value
that you take hasn't been taken by other devices. For example:

.. code-block:: sh

   # specify /dev/video2
   sudo modprobe v4l2loopback video_nr=2

It is better to have the ``v4l2loopback`` module automatically loaded upon
system boot. This can done as follows:

.. code-block:: sh

   echo 'v4l2loopback' | sudo tee -a /etc/modules-load.d/modules.conf
   # change 2 accordingly to match the exact number on your system
   echo 'options v4l2loopback video_nr=2' | sudo /etc/modprobe.d/v4l2loopback.conf


cv_camera
---------

Once the system-level driver installation completes, a ROS-level driver is also
required. The cv_camera_ ROS package has been verified to work with RICOH THETA
cameras without extra tweaking.

Install cv_camera:

.. code-block:: sh

   sudo apt update
   sudo apt install ros-${ROS_DISTRO}-cv-camera
