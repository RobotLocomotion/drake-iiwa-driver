# drake-kuka-driver

This repository contains the application code used to communicate with
the KUKA iiwa are from Drake.  Communicating with the arm requires two
pieces, a Java application to run on the KUKA Sunrise cabinet, and an
local C++ application communicating with the cabinet using the FRI
protocol.

## Sunrise Workbench

Provisioning the IIWA arm must be done from Sunrise Workbench.

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

 * Copy in kuka_driver/DrakeFRIDriver.java over the old one (make sure Sunrise sees the update) in DrakeFRIDriver/src/drake_fri/

 * In "Package Explorer", select StationSetup.cat
  * Installation
    * Push "Install" (this will not actually install the application, but it will wipe the existing configuration of the KUKA cabinet and replace it with yours).  It will take a few minutes, and eventually will reboot the controller.

 * Press the "sync" button.  It's on the toolbar at the top, 5th from the right.  It looks a bit like a square with a couple of arrows over it (though it doesn't look much like this).  This will install the application.
  * Execute

## C++ driver

TODO(sam.creasey) Update the Drake external to point to master once
the needed changes are integrated.

Once Sunrise Workbench is provisioned, you'll need to build the local
interface which communicated with the iiwa arm using FRI, and with the
controlling application using LCM.  Compiling the project in this
directory will output a single program called "kuka_driver".  Running
it with no arguments will connect to the IIWA at it's default address
and port (192.170.10.2, port 30200), negotiate LCM into the command
state, and report the IIWA status via LCM.  If no LCM control messages
are received, the arm will be in a "limp" state where it can be moved
externally subject to the configured impedence force (hardcoded in the
Java application).  Once it receives a command via LCM, that position
will be commanded until the next LCM position is received.

An application wishing to control the arm should listen to LCM for
status updates and command the joints appropriately in response.
