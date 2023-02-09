# ricoh_theta_ros

The ROS package for RICOH THETA V and Z1 cameras.

## Prerequisites

Having RICOH THETA V/Z1 cameras work on Linux requires a patched libuvc 1.5
driver, a video sampling application, and a dummy v4l2 loopback device, each of
which can be obtained from the following projects respectively.

- [libuvc-theta](https://github.com/ricohapi/libuvc-theta)
- [libuvc-theta-sample](https://github.com/madjxatw/libuvc-theta-sample.git)
- [v4l2loopback](https://github.com/umlaeute/v4l2loopback)

The libuvc-theta-sample repo given above is a fork of the original containing
several pre-created branches specific to different configurations. For example,
the `nvdec` branch uses NVIDIA decoder instead of the open source one; the
`yv12` branch sets the pixel format for the decoded video to YVU420; the
`nvdec-yv12` branch combines the previous two. Check out the branch that
fits your case, or create a new branch with your own modification.

ricoh_theta_ros adds all mentioned 3rd-party dependencies as Git submodules
under the `deps` subdirectory, hence you don't have to download them
manually.

See [RICOH Linux development docs](https://codetricity.github.io/theta-linux/)
if you are interested in more details.

## Installation

Make sure the following ROS packages have been installed:

- `camera_info_manager`
- `catkin`
- `cv_bridge`
- `image_transport`
- `nodelet`
- `roscpp`
- `roslint`
- `rostest`
- `sensor_msgs`
- `cv_camera`

Navigate into your ROS workspace directory and run:

```sh
git -C src clone --recursive https://github.com/madjxatw/ricoh_theta_ros.git
```

Install any dependencies in the `deps` directory if it is not yet installed.
See the [documentation](#documentation) for how to build, install and configure
them.

Run

```sh
catkin_make
```

to build the workspace.

## Usage

Once the build is done successfully, source the workspace setup script depending
on your shell type, e.g. `</path/to/your/ros/workspace>/devel/setup.bash` for
bash.

Run the startup script:

```sh
rosrun ricoh_theta_ros start.sh
```

The `start.sh` script performs:

- Starting all the stuff required to capture the live streaming data from the
  camera.
- Running a launch file that starts `cv_camera` node and remaps the `image_raw`,
  `camera_info` and `set_camera_info` topics from `cv_camera` namespace to
  `360cam` namespace.
- Setting up resolution. RICOTH THETA V and Z1 support live streaming in either 4K
  (3840x1920) or 2K (1920x960) resolution, `start.sh` sets resolution to 2K to
  reduce latency.

Write your own launch files or startup scripts if the default is not satisfying.

## Camera control

RICOH has no official camera control application for Linux. Instead of using the
physical buttons, we can use a command line tool bundled with
[libptp](http://libptp.sourceforge.net/) called `ptpcam` together with [RICOH
THETA USB API](https://api.ricoh/docs/theta-usb-api/) to control the cameras on
Linux.

ricoh_theta_ros ships the `ricoh_theta_ros/utils/ricoh` script that facilitates
the camera control by wrapping the execution of `ptpcam` into a set of more
intuitive commands. Copy the `ricoh` script file to a system binary path (e.g.
`~/.local/bin/` or `/usr/local/bin`) and make sure its executable permission bit
is set so that you can run it anywhere.

See the [documentation](#documentation) for how to install libptp.

## Equirectangular-to-Perspective image conversion

RICOH THETA V1 and Z1 stream stitched 360-degree panorama images in live
USB streaming mode, and there is so far no way turning off the internal
stitcher. We use the
[Equirec2Perspec](https://github.com/madjxatw/Equirec2Perspec) library to do
equirectangular-to-perspective conversion.

ricoh_theta_ros has integrated Equirec2Perspec as a ROS catkin package named
**equirec2perspec** (all lowercase), hence you don't have to install it
manually.

To use the equirec2perspec package:

- add `equirec2perspec` as `<build_depend>` and `<exec_depend>` in the
  `package.xml` of your own package.
- Append `equirec2perspec` to `find_package(catkin)` in your `CMakeLists.txt`
  file:

  ```cmake
  find_package(CATKIN REQUIRED COMPONENTS equirec2perspec)
  ```

then you can reference its header files and libraries using
`CATKIN_INCLUDE_DIRS` and `CATKIN_LIBRARIES` respectively.

Include the header file as below:

```c++
#include "equirec2perspec/equirec2perspec.h"
```

See the [documentation](#documentation) for more about it.

## Documentation

The documentation was written in RestructuredText, you need to install Sphinx
and the required theme to build it.

```sh
pip install python3-sphinx sphinx-rtd-theme
cd docs
make html
```

To view docs, open `docs/_build/html/index.html` in your web browser.
