/*****************************************************************/
/*    NAME: Tyler Errico                                         */
/*    ORGN: Robotics Research Center, USMA, West Point, NY       */
/*    FILE: M1_8_Info.cpp                                        */
/*    DATE: 03 AUGUST 2022                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include <cstdlib>
#include <iostream>
#include "M1_8_Info.h"
#include "ColorParse.h"
#include "ReleaseInfo.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  The iM1_8 application serves as an intermediary between       ");
  blk("  a MOOS community (backseat) and a vehicle (frontseat).        ");
  blk("                                                                ");
  blk("  iM1_8 performs two essential tasks:                           ");
  blk("                                                                ");
  blk("    1) CONNECT: [FRONTSEAT] <-(IP:TCP)-> [BACKSEAT]             ");
  blk("                                                                ");
  blk("    2) TRANSLATE: [NMEA MESSAGES] <-(iM1_8)-> [MOOS MESSAGES]   ");
  blk("                                                                ");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: iM1_8 file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch iM1_8 with the given process name                  ");
  blk("      rather than iM1_8.                                        ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of iM1_8.                     ");
  blk("                                                                ");
  blk("Note: If argv[2] does not otherwise match a known option,       ");
  blk("      then it will be interpreted as a run alias. This is       ");
  blk("      to support pAntler launching conventions.                 ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showExampleConfigAndExit

void showExampleConfigAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("iM1_8 Example MOOS Configuration                                ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = iM1_8                                           ");
  blk("{                                                               ");
  blk("  AppTick   = 10                                                ");
  blk("  CommsTick = 10                                                ");
  blk("                                                                ");
  blk("  ip_address     = 192.168.1.11 // frontseat IP address         ");
  blk("  port_number    = 8001        // port number at IP address    ");
  blk("  gps_prefix     = NAV_         // prepended to MOOS variables  ");
  blk("  nav_prefix     = NAV                                          ");
  blk("  compass_prefix = COMPASS                                      ");
  blk("                                                                ");
  blk("  max_rudder     = 50.0         // Max Rudder Angle  [+/- deg]  ");
  blk("  max_thrust     = 100.0        // Max Thrust        [+/- %]    ");
  blk("                                                                ");
  blk("  drive_mode     = normal       // [normal], aggro, or rotate   ");
  blk("  comms_type     = client       // [server], client             ");
  blk("  stale_thresh   = 15           // [1.5] seconds                ");
  blk("                                                                ");
  blk("  rev_factor     = 0.3          // factor for max rev thrust    ");
  blk("                                // allowed in reverse when in   ");
  blk("                                // aggro mode.  Example:        ");
  blk("                            // max_rev_thrust= -[0.3]*max_thrust");
  blk("                                                                ");
  blk("  ivp_allstop    = false        // Default is true              ");
  blk("                                                                ");
  blk("  ignore_msg = $GPGLL, $GPZDA                                   ");
  blk("  ignore_msg = $GPGSV, $GPVTG                                   ");
  blk("  ignore_checksum_errors = true // [false], true                ");
  blk("                                                                ");
  blk("  max_appcast_events = 8                                        ");
  blk("  max_appcast_run_warnings = 10                                 ");
  blk("                                                                ");
  blk("  // heading_source options are gps, imu, or auto where auto    ");
  blk("  // uses imu when available and not timed out as defined in    ");
  blk("  // nav_thresh param.                                          ");
  blk("  heading_source = auto                                         ");
  blk("                                                                ");
  blk("  // threshold in seconds, default is 1.5                       ");
  blk("  stale_nvg_msg_thresh = 2                                      ");
  blk("                                                                ");
  blk(" //Legacy iM1_8 controller                                      ");
  blk(" legacy = false //default is true                               ");
  blk(" min_speed = 1.5 //mps                                          ");
  blk(" max_speed = 3.5 //mps                                          ");
  blk(" min_speed_angle = 75 //degrees between current heading and     ");
  blk("                      //desired heading for min speed           ");
  blk(" max_speed_angle = 15 //degrees between current heading and     ");
  blk("                      //desired heading for max speed           ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}


//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blu("=============================================================== ");
  blu("iM1_8 INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  DESIRED_THRUST        (double) Desired thrust            [%]  ");
  blk("  DESIRED_RUDDER        (double) Desired rudder angle      [deg]");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  [gps_prefix]_X        (double)  X-position on local grid [m]  ");
  blk("  [gps_prefix]_Y        (double)  Y-position on local grid [m]  ");
  blk("  [gps_prefix]_LAT      (double)  Latitude                 [deg]");
  blk("  [gps_prefix]_LON      (double)  Longitude                [deg]");
  blk("  [gps_prefix]_SPEED    (double)  Speed                    [m/s]");
  blk("  [gps_prefix]_HEADING  (double)  Heading from true North  [deg]");
  blk("                                                                ");
  blk("  IMU_ROLL              (double)  Roll                     [deg]");
  blk("  IMU_PITCH             (double)  Pitch                    [deg]");
  blk("  IMU_YAW               (double)  Yaw (Heading)            [deg]");
  blk("                                                                ");
  blk("  M300_BATT_VOLTAGE     (double)  Vehicle battery voltage  [V]  ");
  blk("  M300_RAW_NMEA         (string)  Raw NMEA sentences       []   ");
  blk("  M300_THRUST_L         (double)  Motor thrust (left)      [?]  ");
  blk("  M300_THRUST_R         (double)  Motor thrust (right)     [?]  ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  showReleaseInfo("iM1_8", "gpl");
  exit(0);
}


