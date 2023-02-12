Equirectangular-to-Perspective Image Conversion
===============================================

RICOH THETA V and Z1 cameras both output the stitched equirectangular panorama
image in live streaming mode, and it is currently not possible to disable
the internal stitcher when streaming via USB.

Equirec2Perspec_ is a library that converts a section of an equirectangular
panorama image into normal view. ricoh_theta_ricoh has integrated
Equirec2Perspec as a ROS catkin package named **quirec2perspec** (all
lowercase), hence you don't have to install it beforehand.

Using Integrated equirec2perspec Pacakge
----------------------------------------

To use equirec2perspec package in your own ROS package, do as follows:

- Add ``equirec2perspec`` as ``<builde_depend>`` and ``<exec_depend>`` in the
  :file:`package.xml` of your package.
- Add ``equirec2perspec`` to ``find_package(catkin)`` in your
  :file:`CMakeLists.txt` file:

  .. code-block:: cmake

     find_package(CATKIN REQUIRED COMPONENTS equirec2perspec)

then you can reference its header files and libraries with
``CATKIN_INCLUDE_DIRS`` and ``CATKIN_LIBRARIES`` respectively.

Include the header file as below:

.. code-block:: c++

   #include "equirec2perspec/equirec2perspec.h"

Insatlling Equirec2Perspec (optionally)
---------------------------------------

Equirec2Perspec is actually an independent project that is not tied to
ricoh_theta_ros. To use the Equirec2Perspec library in multiple ROS workspaces,
it would be better to install it to the system so that you can link against it
in any other project.

The system-wide installation can be done as follows:

.. code-block:: sh

   git clone https://github.com/madjxatw/Equirec2Perspec
   cd Equirec2Perspec
   mkdir build && cd build
   cmake ..
   make
   sudo make install

The execution of ``make install`` installs the following files:

- :file:`/usr/local/include/equirec2perspec/equirec2perspec.h`
- :file:`/usr/local/lib/libequi2pers.so`
- :file:`/usr/local/lib/cmake/equirec2perspec/Equirec2PerspecConfig.cmake`
- :file:`/usr/local/lib/cmake/equirec2perspec/Equirec2PerspecConfigVersion.cmake`
- :file:`/usr/local/lib/cmake/equirec2perspec/Equirec2PerspecConfigTargets.cmake`

Once installed, it could be imported in CMake by doing:

.. code-block:: cmake

   find_package(Equirec2Perspec NO_MODULE)
   target_link_libraries(${YOUR_TARGET_NAME} Equirec2Perspec)

.. note::

   The original CMake pacakge is named ``Equirec2Perspec`` whereas the
   integrated package is named ``equirec2persepc``.


Use Cases
---------

A brief use case:

.. code-block:: c++

   // Inclucde header files
   #include <equirec2perspec/equirec2perspec.h>

   // Output image resolution
   const int height = 720;
   const int width = 1080;

   // z-axis angle (0: forward, 180: backward)
   float theta = 180.0f;

   // y-axis angle (>0: upper, <0: lower)
   float phi = 0.0f;

   // FOV of the output perspective image
   float fov = 120.0f;

   // instantiate a parser
   Equirec2Perspec equi_parser;
   // input_image and output_image are both of cv::Mat type
   equi_parser.setParams(input_image, fov, height, width);
   equi_parser.convert(input_image, output_image, theta, phi);
   // setParams and convert can be combined into one step
   equi_parser.convert(input_image, output_image, fov, theta, phi, height, width);
