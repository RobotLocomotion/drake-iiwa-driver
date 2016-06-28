# drake-kuka-driver

This repository contains the application code used to communicate with
the KUKA iiwa are from Drake.  Communicating with the arm requires two
pieces, a Java application to run on the KUKA Sunrise cabinet, and an
local C++ application communicating with the cabinet using the FRI
protocol.

TODO(sam.creasey) Add a section explaining the FRI C++ portions, the externals, and building the resulting driver


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

 * Copy in DrakeFRIDriver.java over the old one (make sure Sunrise sees the update) in DrakeFRIDriver/src/drake_fri/

 * In "Package Explorer", select StationSetup.cat
  * Installation
    * Push "Install" (this will not actually install the application, but it will wipe the existing configuration of the KUKA cabinet and replace it with yours).  It will take a few minutes, and eventually will reboot the controller.

 * Press the "sync" button.  It's on the toolbar at the top, 5th from the right.  It looks a bit like a square with a couple of arrows over it (though it doesn't look much like this).  This will install the application.
  * Execute