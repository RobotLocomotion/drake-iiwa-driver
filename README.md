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

Provisioning the IIWA arm must be done from Sunrise Workbench.

The computer running Sunrise Workbench must be configured with an
address which can communicate with 172.31.1.147/16, and which is
connected to the X66 port.

### Installing Sunrise Connectivity Software

**Note**: This step may no longer be necessary with newer version of Sunrise
which install options such as FRI during the main installation
process.  For older versions, see below.

In order to use FRI to drive the arms additional software must be
installed. In the DVD's that were shipped to you their should be one
called "Media". It should contain files with names like
`0000288631_00_KUKA Sunrise.FRI 1.13_1.13.0_B6_C358496.zip`. These are
Java packages. In order to use them when developing the Sunrise
Workbench Application they must be installed. In kuka speak these are
called "software options". In the SunriseWorkbench documentation
(1.11) Section 10.5.1 details how to install software options. The
basic steps are

  * In dropdown menu **Help->Install new software**
  * In the rop right of the **Work with** box click **Add**
  * Click **Archive** and navigate to where the aforementioned zip file is.
  * Hit **Ok** and then select that software option when it appears with checkbox.
  * Keep hitting **next** until it is installed.
  * Make sure to restart SunriseWorkbench when prompted.

### Creating the Java Application

TODO(sam.creasey) Can I just zip up a project/workspace?

  * File -> New -> SunriseProject
    * IP address of the controller can be left at `172.31.1.147`
    * Create new project
    * Project Name: `DrakeFRIDriver`
    * Topology template: `LBR iiwa 14 R820` (or the appropriate model)
    * Media Flange: `Medien-Flansch Touch pneumatisch` (or the appropriate model)
    * Create Application
    * Source folder: whatever
    * Package: `drake_fri`
    * Name: `DrakeFRIDriver`

  * In "Package Explorer", select `StationSetup.cat`
    * Software (leave anything checked which already is, I think)
      * Fast Robot Interface
    * Save (Ctrl-S)

  * In "Package Explorer", select `SafetyConfiguration.sconf`
    * Customer PSM
      * Uncheck row 1 "External EMERGENCY STOP"
      * Uncheck row 2 "Operator Protection",
      * Uncheck row 3 "Safety Stop"

  * Copy the Java source code for `DrakeFRIPositionDriver` and
  `DrakeFRITorqueDriver`.
    * KUKA changed the spelling of the Java FRI interface as of Sunrise OS version 1.11.  The appropriate sources files can be found in either `kuka-driver/sunrise_1.7` or `kuka-driver/sunrise_1.11`  (NOTE: The `sunrise_1.7` version is no longer actively tested, as I don't have any cabinets still running the older version of the software).
    * Copy `DrakeFRIPositionDriver.java` and `DrakeFRITorqueDriver` to `DrakeFRIDriver/src/drake_fri` (make sure Sunrise sees the update, you may need to import the files into the project).  You can remove any exising `DrakeFRIDriver`.

  * In "Package Explorer", select `StationSetup.cat`
    * Installation
        * Push "Install" (this will not actually install the application, but it will wipe the existing configuration of the KUKA cabinet and replace it with yours).  It will take a few minutes, and eventually will reboot the controller.

  * Press the "sync" button.  It's on the toolbar at the top, 5th from the right.  It looks a bit like a square with a couple of arrows over it (though it doesn't look much like this).  This will install the application.
    * Execute

### Enabling the Safety Configuration on the Robot

When you upload a new safety configuration to the robot it needs to be enabled. To do this, start at the main page on the pendant and go to `Safety-->Activation`. Then click `Activation`. Use the default user, the password is `argus`. This should enable the new configuration.

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
