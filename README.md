# drake-kuka-driver

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

## Sunrise Workbench

Provisioning the IIWA arm must be done from Sunrise Workbench.

The computer running Sunrise Workbench must be configured with an
address which can communicate with 172.13.1.47/16, and which is
connected to the X66 port.

TODO(sam.creasey) Can I just zip up a project/workspace?

 * File -> New -> SunriseProject
  * IP address of the controller can be left at 172.31.1.147
  * Create new project
  * Project Name: DrakeFRIDriver
  * Topology template: LBR iiwa 14 R820
  * Media Flange: Medien-Flansch Touch pneumatisch
  * Create Application
  * Source folder: whatever
  * Package: drake_fri
  * Name: DrakeFRIDriver

 * In "Package Explorer", select StationSetup.cat
  * Software (leave anything checked which already is, I think)
   * Direct Servo Motion Extension (might not be needed?)
   * Fast Robot Interface
   * Smart Servo Motion Extension
  * Save (Ctrl-S)

 * In "Package Explorer", select SafetyConfiguration.sconf
  * Customer PSM
   * Uncheck row 1 "External EMERGENCY STOP"
   * Uncheck row 2 "Operator Protection",
   * Uncheck row 3 "Safety Stop"

 * Copy in kuka-driver/DrakeFRIPositionDriver.java and kuka-driver/DrakeFRITorqueDriver (make sure Sunrise sees the update, you may need to import the files into the project) in DrakeFRIDriver/src/drake_fri/  You can remove any exising DrakeFRIDriver.

 * In "Package Explorer", select StationSetup.cat
  * Installation
    * Push "Install" (this will not actually install the application, but it will wipe the existing configuration of the KUKA cabinet and replace it with yours).  It will take a few minutes, and eventually will reboot the controller.

 * Press the "sync" button.  It's on the toolbar at the top, 5th from the right.  It looks a bit like a square with a couple of arrows over it (though it doesn't look much like this).  This will install the application.
  * Execute

## C++ driver

TODO(sam.creasey) Update the Drake external to point to master once
the needed changes are integrated.

The computer running Sunrise Workbench must be configured with an
address which can communicate with 192.170.10.2/24, and which is
connected to the KONI port.  KUKA recommends directly attaching the
computer to the KONI port instead of using a switch.

Once Sunrise Workbench is provisioned, you'll need to build the local
interface which communicates with the iiwa arm using FRI, and with the
controlling application using LCM.  Compiling this project will output
a single program in the build directory called "kuka_driver".  Running
it with no arguments will connect to the IIWA at it's default address
and port (192.170.10.2, port 30200), negotiate LCM into the command
state, and report the IIWA status via LCM.  If no LCM control messages
are received, the arm will be in a "limp" state where it can be moved
externally subject to the configured impedence force (hardcoded in the
Java application).  Once it receives a command via LCM, that position
will be commanded until the next LCM position is received.

An application wishing to control the arm should listen to LCM for
status updates and command the joints appropriately in response.
