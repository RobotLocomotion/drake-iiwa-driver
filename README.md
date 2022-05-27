# drake-iiwa-driver

This repository contains the application code used to communicate with
the KUKA iiwa are from Drake.  Communicating with the arm requires two
pieces, a Java application to run on the KUKA Sunrise cabinet, and an
local C++ application communicating with the cabinet using the FRI
protocol.

The KUKA control cabinet has two network interfaces which are
configured with static IP addresses.  Both will need to be connected
to communicate with the arm.

 * X66: 172.31.1.147/16 -- This interface is used by Sunrise Workbench to load new software onto the controller.
 * KONI: 192.170.10.2/24 -- This is the interface which FRI clients communicate over.  That's not in the reserved IP space, so it could potentially cause a conflict if you happen to want to contact a host in that subnet.

Selecting the command mode to use for the arm (position vs. torque) is
done by starting a different Java application on the arm
(DrakeFRIPositionDriver vs. DrakeFRITorqueDriver).  The C++ driver
will detect which mode the FRI connection is in and send commands
appropriately.

## Sunrise Workbench

Provisioning the IIWA arm must be done from Sunrise Workbench. Please ensure
that you have Sunrise Workbench 1.14 installed, and that it is compatible with
your control cabinet (see below). If you need instructions for older
versions, please see [`1faf413`](https://github.com/RobotLocomotion/drake-iiwa-driver/tree/1faf413).

When installing, the default checkboxes for Connectivity Software should cover
the use cases needed here.

The computer running Sunrise Workbench must be configured with an
address which can communicate with 172.31.1.147/16, and which is
connected to the X66 port.

**Note**: Try to briefly familiarize yourself with first-party documentation
from KUKA. For relevant software, the documents should be named something
along the lines of "KUKA Sunrise.OS / KUKA Sunrise.Workbench",
"KUKA Sunrise.FRI".

### Backing up Existing Applications / Checking Compatibility

Before creating a new project, it is recommended to back up existing projects
from the controller. This is useful for backing up *and* ensuring that your
Sunrise Workbench is compatible with the Sunrise OS installed on the control
cabinet.

* (Optional) Switch to a new workspace: File -> Switch Workspace.
* File -> New -> Sunrise project
  * Select "Load project from controller"
  * Instructions from here should be straightforward.

### Creating the Java Application

TODO(sam.creasey) Can I just zip up a project/workspace?

  * File -> New -> SunriseProject
    * IP address of the controller can be left at 172.31.1.147
    * Create new project
    * Project Name: DrakeFRIDriver
    * Topology template: LBR iiwa 14 R820 (or the appropriate model)
    * Media Flange: Medien-Flansch Touch pneumatisch (or the appropriate model)
    * Uncheck "Create Sunrise application (starts another wizard)"
    * Source folder: whatever
    * Package: drake_fri
    * Name: DrakeFRIDriver

  * In "Package Explorer", select StationSetup.cat
    * Software
      * Leave anything that is already checked as-is
      * Check "Fast Robot Interface Extension"
    * Save (Ctrl-S)

  * In "Package Explorer", select SafetyConfiguration.sconf
    * Customer PSM
      * Uncheck row 1 "External EMERGENCY STOP"
      * Uncheck row 2 "Operator Protection",
      * Uncheck row 3 "Safety Stop"

  * Copy the Java source code for DrakeFRIPositionDriver and DrakeFRITorqueDriver.
    * In "Package Explorer", expand tree for your new project, navigate to `src`, right-click, and select New -> Package. Name this new package `drake_fri`.
    * Use code from `kuka-driver/sunrise_1.14`.
    * Copy DrakeFRIPositionDriver.java and DrakeFRITorqueDriver to DrakeFRIDriver/src/drake_fri. It may be easiest to drag and drop from a Windows Explorer window into the "Package Explorer" pane.  You can remove any exising DrakeFRIDriver files.

  * In "Package Explorer", select StationSetup.cat
    * Installation
      * Push "Install" (this will not actually install the application, but it will wipe the existing configuration of the KUKA cabinet and replace it with yours).  It will take a few minutes, and eventually will reboot the controller.

  * Press the "sync" button.  It's on the toolbar at the top, 5th from the right.  It looks a bit like a square with a couple of arrows over it (though it doesn't look much like this).  This will compile the Java source code and install the resulting applications.
    * Execute

### Enabling the Safety Configuration on the Robot

When you upload a new safety configuration to the robot it needs to be enabled. To do this go to `Safety-->Activation` on the pendant. Then click `Activation`. Use the default user, the password is `argus`. This should enable the new configuration.

## C++ driver

Once Sunrise Workbench is provisioned, you'll need to configure the
system which will communicate directly with the KONI interface.  This
system must be configured for the IP address 192.170.10.200 (netmask
/24, or 255.255.255.0) (this can be changed in the Java applications).
KUKA recommends directly attaching the computer to the KONI port
instead of using a switch.  Some network interfaces (particularly some
Intel models) have issues when cabled directly to the KONI port (problems
include link flapping up/down repeatedly).

On the SmartPad, turn the key switch, and choose the "AUT" mode, then
turn the key switch back. Choose either "DrakeFRITorqueDriver" or
"DrakeFRIPositionDriver" from "Application". Press the green "Play"
button on the left sidebar of the SmartPad.

### Compiling the driver

Next, build the driver program to communicate with the iiwa arm using
FRI, and with the controlling application using LCM.  Compiling this
project will output a single program in the build directory called
"kuka_driver".  Running it with no arguments will connect to the IIWA
at it's default address and port (192.170.10.2, port 30200), negotiate
LCM into the command state, and report the IIWA status via LCM.

An application wishing to control the arm should listen to LCM for
status updates and command the joints appropriately in response.

The C++ driver can be build with either CMake or bazel.  There are two
external dependencies which need to be provided, FRI and drake.

This repository is configured with a private git submodule for the
KUKA FRI source code. To pull it:
```
git submodule init
git submodule update
```

If you do not have access to that repository, you will need to install
your own version of the FRI source (which can be found in your Sunrise
project in the directory FastRobotInterface_Client_Source).

```
cd kuka-fri
unzip /path/to/your/copy/of/FRI-Client-SDK_Cpp.zip
patch -p1 < ../fri_udp_connection_file_descriptor.diff
```

The patch above applies correctly to the FRI 1.7, 1.11, and 1.14 source.
Other versions have not been tested.

#### Building with bazel

When building with bazel, it will automatically fetch a copy of drake
to build against.  You should be able to run `bazel build //...` to
produce a working driver.

This is the easiest method for developers who are already working with
the drake source, since a working bazel build enviornment for drake
should be sufficient to build this package.

#### Building with CMake

When building with CMake, you'll need a binary copy of drake.  The
latest release can be found at
https://github.com/RobotLocomotion/drake/releases.  Uncompress the
binary tarfile, and add
`-DCMAKE_PREFIX_PATH=<path-to-drake>/drake/lib/cmake` to your cmake
command line.  For example:

```
mkdir build
cd build
cmake .. -DCMAKE_PREFIX_PATH=/tmp/drake/lib/cmake
make
```
