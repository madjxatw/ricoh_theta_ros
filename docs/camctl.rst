Camera Control
==============

Although RICOH has no official camera contorl apps for Linux, it is still
possible to control the camera through RICOH THETA USB API. A common utility
that we can use on Linux is the command line program from libptp2 called
:program:`ptpcam`.

Installing libptp
-----------------

`libptp`_ is a library used to communicate with PTP devices like still imaging
cameras or MP3 players. A command line tool :program:`ptpcam` bundled with
libptp can be used to download files or tweak camera properties.

libptp builds upon an old version (0.1) of libusb, which can usually be
installed from the libusb-dev package on Ubuntu. If libusb-dev is not available
on your Ubuntu distribution, instead install the libusb-compat-0.1 package along
with the libusb-1.0-*X*-dev package, where *x* depends on your distribution.

The  2-1.2 version of libptp  has a trivial `bug
<https://codetricity.github.io/theta-linux/usb_api/#test-ptpcam>`_ that can be
fixed by modifying the :file:`src/ptp.h` file:

.. code-block:: diff

   @@ -74,7 +74,7 @@ struct _PTPUSBBulkContainer {
    };
    typedef struct _PTPUSBBulkContainer PTPUSBBulkContainer;

   -#define PTP_USB_INT_PACKET_LEN 8
   +#define PTP_USB_INT_PACKET_LEN 28

    /* PTP USB Asynchronous Event Interrupt Data Format */
    struct _PTPUSBEventContainer {

The build of 2-1.2.0 version of libptp can be done as follows:

.. code-block:: sh

   wget https://jaist.dl.sourceforge.net/project/libptp/libptp2/libptp2-1.2.0/libptp2-1.2.0.tar.gz
   tar zxf libptp2-1.2.0.tar.gz
   cd liptp2-1.2.0
   # perform the bug fix described above
   ./configure
   make && sudo make install
   sudo ldconfig -v

USB API
-------

Read `RICOH THETA USB API Docs`__ if you are interested in details.

__ https://api.ricoh/docs/theta-usb-api/

The ricoh_theta_ros package provides a :file:`ricoh_theta_ros/utils/ricoh`
script that wraps the various execution of ``ptpcam`` into a set of more
intuitive commands. Copy this script file to a system binary directory, e.g.
:file:`/usr/local/bin/` or :file:`~/.local/bin/`, so that you can run ``ricoh``
anywhere.

Run ``ricoh --help`` or simply just enter ``ricoh`` without any argument to view
its usage. Here are some common use cases:

.. code-block:: sh

   # check camera status
   ricoh status
   # put the camera to sleep
   ricoh sleep 1 # or ricoh sleep
   # awaken the camera
   ricoh sleep 0 # or ricoh wake
   # set the camera to live streaming mode
   ricoh mode live
   # set the camera to still image mode
   ricoh mode image
   # disable auto sleep
   ricoh ausleep 0
   # disable auto poweroff
   ricoh auoff 0
   # power off the camera
   ricoh poweroff
