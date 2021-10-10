Hardware Acceleration
=====================

As suggested by RICOH THETA development docs, hardware acceleration could be
achieved by using the two plugins:

- nvdec hardware decoding plugin for NVIDIA GPUs
- glimagesink OpenGL plugin

Building NVDEC and NVENC as GStreamer plugins
---------------------------------------------

Check whether nvdec and nvenc available as GStreamer plugins:

.. code-block:: sh

   gst-inspect-1.0 | grep nvdec
   gst-inspect-1.0 | grep nvenc

No output on the screen indicates the unavailability.

Downloading gst-plugins-bad
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Download ``gst-plugins-bad`` repository and check out the same version as the
GStreamer installed on your system:

.. code-block:: sh

   git clone git://anongit.freedesktop.org/gstreamer/gst-plugins-bad
   cd gst-plugins-bad/
   git checkout "$(gst-inspect-1.0 --version | head -n1 | cut -d' ' -f3)"


Downloading Video Codec SDK
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Access `Video Encode and Decode GPU Support Matrix`__ to make sure that your
NVIDIA GPU is supported by nvdec.

__ https://developer.nvidia.com/video-encode-decode-gpu-support-matrix

Video Codec SDK requires that NVIDIA proprietary driver and CUDA have been
installed, and the SDK version picked must support your CUDA version and NVIDA
driver version. If your system specification does not meet the requirement of
the latest SDK, you will have to download a `legacy version
<https://developer.nvidia.com/video-codec-sdk-archive>`_. For example, you need
to download Video Codec SDK 10.0 for CUDA 10.0.

Run ``nvcc --version`` to check the version of CUDA installed on your system.

Run ``nvidia-smi`` to check the information of NVIDIA display driver installed
on your system

.. note::

   Downloading Video Codec SDK requires a membership of the NVIDIA Developer
   Program, hence you have to join this program first.

Preparing Header Files
~~~~~~~~~~~~~~~~~~~~~~

Assuming CUDA in installed under :file:`/usr/local/`, do:

.. code-block:: sh

   cd <path/to/video/codec/sdk>
   cp /usr/local/cuda/include/cuda.h </path/to/gst-plugins-bad>/sys/nvenc
   cp Interface/nvEncodeAPI.h </path/to/gst-plugins-bad>/sys/nvenc
   cp Interface/cuviddec.h </path/to/gst-plugins-bad>/sys/nvdec
   cp Interface/nvcuvid.h </path/to/gst-plugins-bad>/sys/nvdec

replace ``<path/to/video/codec/sdk>`` and ``</path/to/gst-plugins-bad>`` with
the exact directory paths in your case.

Building plugins
~~~~~~~~~~~~~~~~

Assuming CUDA in installed under :file:`/usr/local/`, do:

.. code-block:: sh

   cd <path/to/gst-plugins-bad>
   NVENCODE_CFLAGS="-I</path/to/gst-plugins-bad>/sys/nvenc" ./autogen.sh \
      --with-cuda-prefix="/usr/local/cuda" --disable-gtk-doc
   cd sys/nvenc
   make
   sudo cp .libs/libgstnvenc.so /usr/lib/x86_64-linux-gnu/gstreamer-1.0/
   cd ../nvdec
   make
   sudo cp .libs/libgstnvdec.so /usr/lib/x86_64-linux-gnu/gstreamer-1.0/

Replace ``</path/to/gst-plugins-bad>`` with the exact directory paths in your
case.

Verification
~~~~~~~~~~~~

Run the following commands again:

.. code-block:: sh

   gst-inspect-1.0 | grep nvdec
   gst-inspect-1.0 | grep nvenc

This time you should get some output if the plugins have installed successfully.

Reconstructing GStreamer Pipleline
----------------------------------

The :file:`gst/gst_viewer.c` file from libuvc-theta-sample needs to be modified
and recompiled to enable nvdec in the GStreamer pipleline:

.. code-block:: diff

   diff --git a/gst/gst_viewer.c b/gst/gst_viewer.c
   index 92d4fb9..9b0ea0b 100644
   --- a/gst/gst_viewer.c
   +++ b/gst/gst_viewer.c
   @@ -185,11 +185,11 @@ main(int argc, char **argv)
                   cmd_name++;

           if (strcmp(cmd_name, "gst_loopback") == 0)
   -               pipe_proc = "decodebin ! autovideoconvert ! "
   +               pipe_proc = "nvdec ! gldownload ! videoconvert n-thread=0 ! "
                           "video/x-raw,format=I420 ! identity drop-allocation=true !"
                           "v4l2sink device=/dev/video2 qos=false sync=false";
           else
   -               pipe_proc = " decodebin ! autovideosink sync=false";
   +               pipe_proc = " nvdec ! glimagesink qos=false sync=false";

           if (!gst_src_init(&argc, &argv, pipe_proc))
                   return -1;

For ``gst_loopback``, the ``gldownload`` component is used to transfer the frame
from the buffers in GPU memory to system memory. Although the transfer increases
latency,
it appears to be faster than streaming without hardware decoding.

For ``gst_view``, the ``glimagesink`` component is used to directly render the
OpenGL texture on the monitor without having to transfer the frame to system
memory.

.. seealso::

   :ref:`Install libuvc-theta-sample <install-libuvc-theta-sample>`
