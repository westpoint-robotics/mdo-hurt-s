/*****************************************************************/
/*    NAME: Tyler Errico                                         */
/*    ORGN: Robotics Research Center, USMA, West Point, NY       */
/*    FILE: M1_8.cpp                                             */
/*    DATE: 03 AUGUST 2022                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#include "MBUtils.h"
#include "LatLonFormatUtils.h"
#include "M1_8.h"
#include "SockNinja.h"

using namespace std;

//---------------------------------------------------------
// Constructor()

M1_8::M1_8()
{
  // Configuration variables  (overwritten by .moos params)
  m_max_rudder   = 30.0;        // default MAX_RUDDER (+/-)
  m_max_thrust   = 100.0;       // default MAX_THRUST (+/-)
  m_drive_mode   = "normal";    // default DRIVE_MODE ("normal"|"aggro"|"rotate")

  m_ivp_allstop      = true;
  m_moos_manual_override = true;

  // Stale Message Detection
  m_stale_check_enabled = false;
  m_stale_mode          = false;
  m_stale_threshold     = 1.5;
  m_count_stale         = 0;
  m_tstamp_des_rudder   = 0;
  m_tstamp_des_thrust   = 0;

  m_num_satellites      = 0;
  m_batt_voltage        = 0;
  m_bad_nmea_semantic   = 0;

  m_nav_x   = -1;
  m_nav_y   = -1;
  m_nav_hdg = -1;
  m_nav_spd = -1;

  m_heading_source        = "auto";
  m_stale_nvg_msg_thresh  = 1.5;
  m_last_nvg_msg_time     = 0;

  m_nav_prefix      = "NAV";
  m_gps_prefix      = "GPS";
  m_compass_prefix  = "COMPASS";
  m_gps_blocked     = false;
}

//---------------------------------------------------------
// Destructor()

M1_8::~M1_8()
{
  m_ninja.closeSockFDs();
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool M1_8::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  //------------------------------------------------------
  // HANDLE PARAMETERS IN .MOOS FILE ---------------------
  //------------------------------------------------------
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for (p = sParams.begin(); p != sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if((param == "port") && isNumber(value)) {
      int port = atoi(value.c_str());
      handled = m_ninja.setPortNumber(port);
    }
    else if(param == "ip_addr")
      handled = m_ninja.setIPAddr(value);
    else if(param == "ivp_allstop")
      handled = setBooleanOnString(m_ivp_allstop, value);
    else if(param == "comms_type")
      handled = m_ninja.setCommsType(value);
    else if(param == "stale_thresh")
      handled = setPosDoubleOnString(m_stale_threshold, value);
    else if(param == "max_rudder")
      handled = m_thrust.setMaxRudder(value);
    else if(param == "max_thrust")
      handled = m_thrust.setMaxThrust(value);
    else if(param == "drive_mode"){
      handled = m_thrust.setDriveMode(value);
      m_drive_mode = value;
    }
    else if(param == "ignore_msg") 
      handled = handleConfigIgnoreMsg(value);
    else if(param == "heading_source"){
      if (value == "gps") {
	m_heading_source = "gps";
	handled = true;
      } else if (value == "imu") {
	m_heading_source = "imu";
	handled = true;
      } else if (value == "auto") {
	m_heading_source = "auto";
	handled = true;
      }
    }
    else if(param == "stale_nvg_msg_thresh")
      handled = setPosDoubleOnString(m_stale_nvg_msg_thresh, value);
    else if(param == "ignore_checksum_errors") {
      bool bool_val;
      bool ok1 = setBooleanOnString(bool_val, value);
      bool ok2 = m_ninja.setIgnoreCheckSum(bool_val);
      handled = ok1 && ok2;
    }
    else if(param == "nav_prefix"){ 
      if(!strContainsWhite(value)){m_nav_prefix=value; handled=true;}
    }
    else if(param == "gps_prefix"){ 
      if(!strContainsWhite(value)){m_gps_prefix=value; handled=true;}
    }
    else if(param == "compass_prefix"){ 
      if(!strContainsWhite(value)){m_compass_prefix=value; handled=true;}
    }
    
    if(!handled){
      reportUnhandledConfigWarning(orig);
      list<string> warnings = m_thrust.getWarnings();
      while (!warnings.empty()){
        reportConfigWarning(warnings.front());
        warnings.pop_front();
      }
    }
  }
  
  // Init Geodesy 
  GeodesySetup();
  
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool M1_8::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void M1_8::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("IVPHELM_ALLSTOP", 0);
//  Register("DESIRED_THRUST",  0);
//  Register("DESIRED_RUDDER",  0);
  Register("DESIRED_HEADING", 0);
  Register("DESRIRED_SPEED", 0);
  Register("BLOCK_GPS",  0);
  Register("ROTATE_IN_PLACE", 0);
  Register("ROTATE_HDG_TARGET", 0);
  Register("ROTATE_TO_POINT", 0);
  Register("MOOS_MANUAL_OVERRIDE", 0);
  
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool M1_8::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);
  
  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    double mtime  = msg.GetTime();
    string key    = msg.GetKey();
    double dval   = msg.GetDouble();
    string sval   = msg.GetString();
    
#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity(); 
    string msrc  = msg.GetSource();  
#endif

   
    if(key == "IVPHELM_ALLSTOP")
      m_ivp_allstop = (toupper(sval) != "CLEAR");
    else if(key == "DESIRED_RUDDER") {
      m_tstamp_des_rudder = mtime;
      m_thrust.setRudder(dval);
    }
    else if(key == "DESIRED_THRUST") {
      m_tstamp_des_thrust = mtime;
      m_thrust.setThrust(dval);
      Notify("M3_DEBUG", m_thrust.getThrust());
    }
    else if(key == "DESIRED_HEADING") {

	m_heading = dval;
    }
    else if(key == "DESIRED_SPEED") {
    
	m_speed = dval;
    }
    else if(key == "BLOCK_GPS") 
      setBooleanOnString(m_gps_blocked, sval);
    else if(key == "ROTATE_IN_PLACE") {      
      bool bval, ok1;
      ok1 = setBooleanOnString(bval, sval);
      m_rot_ctrl.setRotateInPlace(bval);
      
      if (ok1 && bval ) {
	// Record the time and location
	m_rot_ctrl.setCmdTimeStamp(mtime);
	m_rot_ctrl.setStartRotX(m_nav_x);
	m_rot_ctrl.setStartRotY(m_nav_y);
      }
    }
    else if(key == "ROTATE_HDG_TARGET") {
      m_rot_ctrl.setHeadingTarget(dval);
    }
    else if(key == "ROTATE_TO_POINT") {
      // save the current location for calculation
      m_rot_ctrl.setStartRotX(m_nav_x);
      m_rot_ctrl.setStartRotY(m_nav_y);
      
      bool ok2 = m_rot_ctrl.handlePoint(sval);
      if (ok2)
	Notify("ROTATE_HDG_TARGET", m_rot_ctrl.getHeadingTarget() );  // for debugging. 
      
    }
    else if(key == "MOOS_MANUAL_OVERRIDE") {
      setBooleanOnString(m_moos_manual_override, sval);

    }
    
    else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
      reportRunWarning("Unhandled Mail: " + key);


  }
  return(true);
}


//---------------------------------------------------------
// Procedure: Iterate()

bool M1_8::Iterate()
{
  AppCastingMOOSApp::Iterate();
  
  // Part 1: Check for allstop or staleness
  checkForStalenessOrAllStop();
    
  // Part 2: Connect if needed, and write/read from socket
  if(m_ninja.getState() != "connected")
    m_ninja.setupConnection();

  if(m_ninja.getState() == "connected") {
    sendMessagesToSocket();
    readMessagesFromSocket();
  }

  // Part 3: Get Appcast events from ninja and report them
  reportWarningsEvents();
  AppCastingMOOSApp::PostReport();
  return(true);
}


//---------------------------------------------------------
// Procedure: GeodesySetup()
//   Purpose: Initialize geodesy object with lat/lon origin.
//            Used for LatLon2LocalUTM conversion.

bool M1_8::GeodesySetup()
{
  double LatOrigin = 0.0;
  double LonOrigin = 0.0;

  // Get Latitude Origin from .MOOS Mission File
  bool latOK = m_MissionReader.GetValue("LatOrigin", LatOrigin);
  if(!latOK) {
    reportConfigWarning("Latitude origin missing in MOOS file.");
    return(false);
  }

  // Get Longitude Origin from .MOOS Mission File
  bool lonOK = m_MissionReader.GetValue("LongOrigin", LonOrigin);
  if(!lonOK){
    reportConfigWarning("Longitude origin missing in MOOS file.");
    return(false);
  }

  // Initialise CMOOSGeodesy object
  bool geoOK = m_geodesy.Initialise(LatOrigin, LonOrigin);
  if(!geoOK) {
    reportConfigWarning("CMOOSGeodesy::Initialise() failed. Invalid origin.");
    return(false);
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: sendMessagesToSocket()

void M1_8::sendMessagesToSocket()
{

  double thrustL, thrustR;

  // Mode 1:  Rotation

  // Check if still ok to rotate in place
  bool ok_to_rotate = m_rot_ctrl.checkClearToRotate( m_nav_x, m_nav_y, m_curr_time);

  if (ok_to_rotate and !m_moos_manual_override) {

    // overwrite incoming thrust and rudder commands
    double thrust, rudder;
    m_rot_ctrl.calControl(m_nav_hdg, m_nav_x, m_nav_y, thrust, rudder);

    // Check if finished
    bool rot_finished = m_rot_ctrl.checkRotateFinished(m_nav_hdg);
    // If finished, and the command to rotate is still true,
    // then send the ROTATE_FINISHED end flag.
    if ( rot_finished and m_rot_ctrl.getRotateInPlace() )
      Notify("ROTATE_FINISHED", "true");
    
    m_thrust.setRudder(rudder * m_max_rudder);
    m_thrust.setThrust(thrust * m_max_thrust);

    // use rotate mode for thrusters. 
    m_thrust.setDriveMode("rotate");
    
  } else {

    // Mode 2: Carry on as normal
    m_thrust.setDriveMode(m_drive_mode);

  }

  // Calculate the right and left thruster values
  // with whatever DriveMode is selected.
  m_thrust.calcDiffThrust();
  // Update differential thrust values
  thrustL = m_thrust.getThrustLeft();
  thrustR = m_thrust.getThrustRight();
  
  // Send the primary PSEAC front seat command
  string msg = "PSEAC,";
  if (m_stale_mode = true) { 
    // msg += "L,"; 
  }
  else if (m_drive_mode == "rotate") { 
    // msg += "C,"; 
  } 
  else if (m_drive_mode == "normal" || m_drive_mode == "aggro") { 
    //msg += "T,";
  }
  //msg +="T,,20,-10,THR_ON";
  // msg += doubleToStringX(thrustL,1) + ",";
  //msg += doubleToStringX(thrustR,1) + ",";
  // Spead/Heading Controller on SeaRobotics Surveyor M1.8
  msg += "G,";
  msg += doubleToStringX(m_heading) + ",";
  msg += doubleToStringX(m_speed) +",,,";
//  double str_des_thrL = (m_thrust.getThrustLeft());
//  double str_des_thrR = (m_thrust.getThrustRight());
  
  //msg += doubleToStringX((str_des_thrL - str_des_thrR),1) + ",";
  //msg = "echo $" + msg + "*" + checksumHexStr(msg);
  msg = "$"+msg+"*"+checksumHexStr(msg) + "\r\n";
  
  m_ninja.sendSockMessage(msg);
  
  // Publish command to MOOSDB for logging/debugging
  //Notify("PSEAC_THRUST_L", thrustL);
  //Notify("PSEAC_THRUST_R", thrustR);
}

//---------------------------------------------------------
// Procedure: readMessagesFromSocket()
//      Note: Messages returned from the SockNinja have been
//            confirmed to be valid NMEA format and checksum

void M1_8::readMessagesFromSocket()
{
  list<string> incoming_msgs = m_ninja.getSockMessages();
  list<string>::iterator p;
  for(p=incoming_msgs.begin(); p!=incoming_msgs.end(); p++) {
    string msg = *p;
    msg = biteString(msg, '\r'); // Remove CRLF
    Notify("IM300_RAW_NMEA", msg);

    bool handled = false;
    if(m_ignore_msgs.count(msg.substr (0,6)) >= 1) 
      handled = true;
    else if(strBegins(msg, "$GPRMC"))
      handled = handleMsgGPRMC(msg);
    else if(strBegins(msg, "$GNRMC"))
      handled = handleMsgGNRMC(msg); // Added on 06-01-2022 by Supun for 2022 Herons
    else if(strBegins(msg, "$GPGGA"))
      handled = handleMsgGPGGA(msg);
    else if(strBegins(msg, "$GNGGA"))
      handled = handleMsgGNGGA(msg); // Added on 06-01-2022 by Supun for 2022 Herons
    else if(strBegins(msg, "$PSEAA")){
      bool cond1 = m_heading_source == "imu";
      bool cond2 = m_heading_source == "auto";
      if (cond1 or cond2)
        handled = handleMsgPSEAA_heading(msg);
      else   // ignore it
	handled = true;
    }
    else if(strBegins(msg, "$PSEAB"))
      handled = handleMsgPSEAB(msg);
    else
      reportBadMessage(msg, "Unknown NMEA Key");
            
    if(!handled)
      m_bad_nmea_semantic++;
  }
}

//---------------------------------------------------------
// Procedure: handleConfigIgnoreMsg()
//  Examples: ignore_msg = $GPGLL
//            ignore_msg = $GPGLL, GPGSV, $GPVTG

bool M1_8::handleConfigIgnoreMsg(string str)
{
  bool all_ok = true;
  
  vector<string> msgs = parseString(str, ',');
  for(unsigned int i=0; i<msgs.size(); i++) {
    string msg = stripBlankEnds(msgs[i]);
    // Check if proper NMEA Header
    if((msg.length() == 6) && (msg.at(0) = '$'))
      m_ignore_msgs.insert(msg);
    else
      all_ok = false;
  }

  return(all_ok);
}

//---------------------------------------------------------
// Procedure: checkForStalenessOrAllStop()
//   Purpose: If DESIRED_RUDDER or _THRUST commands are stale,
//            set local desired_rudder/thrust to zero.
//            If an all-stop has been posted, also set the
//            local desired_rudder/thrust vals to zero.

void M1_8::checkForStalenessOrAllStop()
{
  if(m_ivp_allstop) {
    m_thrust.setRudder(0);
    m_thrust.setThrust(0);
    return;
  }

  // If not checking staleness, ensure stale mode false, return.
  if(!m_stale_check_enabled) {
    m_stale_mode = false;
    return;
  }

  double lag_rudder = m_curr_time - m_tstamp_des_rudder;
  double lag_thrust = m_curr_time - m_tstamp_des_thrust;

  bool stale_rudder = (lag_rudder > m_stale_threshold);
  bool stale_thrust = (lag_thrust > m_stale_threshold);

  if(stale_rudder)
    m_count_stale++;
  if(stale_thrust)
    m_count_stale++;

  bool stale_mode = false;
  if(stale_rudder || stale_thrust) {
    m_thrust.setRudder(0);
    m_thrust.setThrust(0);
    stale_mode = true;
  }

  // Check new stale_mode represents a change from previous
  if(stale_mode && !m_stale_mode) 
    reportRunWarning("Stale Command Detected: Stopping Vehicle");
  if(!stale_mode && m_stale_mode) 
    retractRunWarning("Stale Command Detected: Stopping Vehicle");

  m_stale_mode = stale_mode;
}


//---------------------------------------------------------
// Procedure: handleMsgGPRMC()
//      Note: Proper NMEA format and checksum prior confirmed  
//   Example:
//   $GPRMC,150942.619,A,0000.00,N,00000.00,W,1.1663,0,291263,0,E*41

//  0   $GPRMC
//  1 [Timestamp]    UTC of position fix
//  2 [Data status]  A-ok, V-invalid
//  3 [Lat_NMEA]     Calculated latitude, in NMEA format
//  4 [LatNS_NMEA]   Hemisphere (N or S) of latitude
//  5 [Lon_NMEA]     Calculated longitude, in NMEA format
//  6 [LonEW_NMEA]   Hemisphere (E or W) of longitude
//  7 [Speed]        Speed over ground in Knots
//  8 [Course]       True Course, Track made good in degrees
//  9 [DepthTop]     Date of Fix
// 10 [Mag Var]      Magnetic variation degrees
// 11 [Mag Var E/W]  Easterly subtracts from true course

bool M1_8::handleMsgGPRMC(string msg)
{
  if(!strBegins(msg, "$GPRMC,"))
    return(false);

  // Remove the checksum info from end
  rbiteString(msg, '*');

  vector<string> flds = parseString(msg, ',');
  if(flds.size() != 13) {
    if(!m_ninja.getIgnoreCheckSum())
      return(reportBadMessage(msg, "Wrong field count"));
  } 
  
  if((flds[4] != "N") && (flds[4] != "S"))
    return(reportBadMessage(msg, "Bad N/S Hemisphere"));
  if((flds[6] != "W") && (flds[4] != "E"))
    return(reportBadMessage(msg, "Bad E/W Hemisphere"));
  
  string str_lat = flds[3];
  string str_lon = flds[5];
  string str_kts = flds[7];
  string str_hdg = flds[8];
  if(!isNumber(str_lat))
    return(reportBadMessage(msg, "Bad Lat"));
  if(!isNumber(str_lon))
    return(reportBadMessage(msg, "Bad Lon"));
  if(!isNumber(str_kts))
    return(reportBadMessage(msg, "Bad Kts"));
  if(!isNumber(str_hdg))
    return(reportBadMessage(msg, "Bad Hdg"));
  
  double dbl_lat = latDDMMtoDD(str_lat);
  double dbl_lon = lonDDDMMtoDDD(str_lon);
  if(flds[4] == "S")
    dbl_lat = -dbl_lat;
  if(flds[6] == "W")
    dbl_lon = -dbl_lon;
  Notify(m_nav_prefix+"_LAT", dbl_lat, "GPRMC");
  Notify(m_nav_prefix+"_LON", dbl_lon, "GPRMC");
  Notify(m_nav_prefix+"_LONG", dbl_lon, "GPRMC");
  if (!m_gps_blocked){
    Notify(m_gps_prefix+"_LAT", dbl_lat, "GPRMC");
    Notify(m_gps_prefix+"_LON", dbl_lon, "GPRMC");
    Notify(m_gps_prefix+"_LONG", dbl_lon, "GPRMC");
  }

  double x, y;
  bool ok = m_geodesy.LatLong2LocalGrid(dbl_lat, dbl_lon, y, x);
  if(ok) {
    m_nav_x = x;
    m_nav_y = y;
    Notify(m_nav_prefix+"_X", x, "GPRMC");
    Notify(m_nav_prefix+"_Y", y, "GPRMC");
    if (!m_gps_blocked){
      Notify(m_gps_prefix+"_X", x, "GPRMC");
      Notify(m_gps_prefix+"_Y", y, "GPRMC");
    }
  }
  
  double dbl_kts = atof(str_kts.c_str());
  double dbl_mps = dbl_kts * 0.514444;
  dbl_mps = snapToStep(dbl_mps, 0.05);
  m_nav_spd = dbl_mps;
  Notify(m_nav_prefix+"_SPEED", dbl_mps, "GPRMC");

  double dbl_hdg = atof(str_hdg.c_str());
  bool stale_imu = ( (MOOSTime() - m_last_nvg_msg_time) > m_stale_nvg_msg_thresh );
  if ( (m_heading_source == "gps") or ( (m_heading_source == "auto") and stale_imu ) ){
    m_nav_hdg = dbl_hdg;
    Notify(m_nav_prefix+"_HEADING", dbl_hdg, "GPRMC");
    Notify(m_compass_prefix+"_HEADING", dbl_hdg, "GPRMC");
  }
  else{
    Notify("GPS_HEADING", dbl_hdg, "GPRMC");
  }
  return(true);
}


//---------------------------------------------------------
// Procedure: handleMsgGNRMC()
bool M1_8::handleMsgGNRMC(string msg)
{
  if(!strBegins(msg, "$GNRMC,"))
    return(false);

  // Remove the checksum info from end
  rbiteString(msg, '*');

  vector<string> flds = parseString(msg, ',');
  if(flds.size() != 13){
    if (!m_ninja.getIgnoreCheckSum() )
      return(reportBadMessage(msg, "Wrong field count"));
  } 
  
  if((flds[4] != "N") && (flds[4] != "S"))
    return(reportBadMessage(msg, "Bad N/S Hemisphere"));
  if((flds[6] != "W") && (flds[4] != "E"))
    return(reportBadMessage(msg, "Bad E/W Hemisphere"));
  
  string str_lat = flds[3];
  string str_lon = flds[5];
  string str_kts = flds[7];
  string str_hdg = flds[8];
  if(!isNumber(str_lat))
    return(reportBadMessage(msg, "Bad Lat"));
  if(!isNumber(str_lon))
    return(reportBadMessage(msg, "Bad Lon"));
  if(!isNumber(str_kts))
    return(reportBadMessage(msg, "Bad Kts"));
  
  double dbl_lat = latDDMMtoDD(str_lat);
  double dbl_lon = lonDDDMMtoDDD(str_lon);
  if(flds[4] == "S")
    dbl_lat = -dbl_lat;
  if(flds[6] == "W")
    dbl_lon = -dbl_lon;
  Notify(m_nav_prefix+"_LAT", dbl_lat, "GNRMC");
  Notify(m_nav_prefix+"_LON", dbl_lon, "GNRMC");
  Notify(m_nav_prefix+"_LONG", dbl_lon, "GNRMC");
  if (!m_gps_blocked){
    Notify(m_gps_prefix+"_LAT", dbl_lat, "GNRMC");
    Notify(m_gps_prefix+"_LON", dbl_lon, "GNRMC");
    Notify(m_gps_prefix+"_LONG", dbl_lon, "GNRMC");
  }

  double x, y;
  bool ok = m_geodesy.LatLong2LocalGrid(dbl_lat, dbl_lon, y, x);
  if(ok) {
    m_nav_x = x;
    m_nav_y = y;
    Notify(m_nav_prefix+"_X", x, "GNRMC");
    Notify(m_nav_prefix+"_Y", y, "GNRMC");
    if (!m_gps_blocked){
      Notify(m_gps_prefix+"_X", x, "GNRMC");
      Notify(m_gps_prefix+"_Y", y, "GNRMC");
    }
  }
  
  double dbl_kts = atof(str_kts.c_str());
  double dbl_mps = dbl_kts * 0.514444;
  dbl_mps = snapToStep(dbl_mps, 0.05);
  m_nav_spd = dbl_mps;
  Notify(m_nav_prefix+"_SPEED", dbl_mps, "GNRMC");

  if(!isNumber(str_hdg))
    return(reportBadMessage(msg, "Bad Hdg"));

  double dbl_hdg = atof(str_hdg.c_str());
  bool stale_imu = ( (MOOSTime() - m_last_nvg_msg_time) > m_stale_nvg_msg_thresh );
  if ( (m_heading_source == "gps") or ( (m_heading_source == "auto") and stale_imu ) ){
    m_nav_hdg = dbl_hdg;
    Notify(m_nav_prefix+"_HEADING", dbl_hdg, "GNRMC");
    Notify(m_compass_prefix+"_HEADING", dbl_hdg, "GNRMC");
  }
  else{
    Notify("GPS_HEADING", dbl_hdg, "GNRMC");
  }
  return(true);
}


//---------------------------------------------------------
// Procedure: handleMsgGPGGA()
//      Note: Proper NMEA format and checksum prior confirmed  
//      Note: Only grabbing the number of satellites from this msg
//   Example:
//   $GPGGA,150502.00,4221.46039,N,07105.28402,W,2,11,0.98,5.5,M,-33.2,M,,0000*62
//                                                 ^^
//  0   $GPGGA
//  1 [Timestamp]    UTC of position fix
//  2 [Lat_NMEA]     Calculated latitude, in NMEA format
//  3 [LatNS_NMEA]   Hemisphere (N or S) of latitude
//  4 [Lon_NMEA]     Calculated longitude, in NMEA format
//  5 [LonEW_NMEA]   Hemisphere (E or W) of longitude
//  6 [GPS Qual]     (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
//  7 [Num Sats]     Num satellites in use, not those in view
//  8 [Horz Dilu]    Horizontal dilution of position
//  9 [Ant Alt]      Antenna altitude above/below mean sea level (geoid)
// 10 [Ant Units]    Meters  (Antenna height unit)
// 11 [Geo Sep]      Geoidal separation (Diff. between WGS-84 earth
//                   ellipsoid and mean sea level.
// 12 [GS Units]     Meters  (Units of geoidal separation)
// 13 [Age]          in secs since last update from diff. ref station
// 14 [Diff ID]     Diff. reference station ID#

bool M1_8::handleMsgGPGGA(string msg)
{
  if(!strBegins(msg, "$GPGGA,"))
    return(false);

  // Remove the checksum info from end
  rbiteString(msg, '*');

  vector<string> flds = parseString(msg, ',');
  if(flds.size() != 14) {
    if(!m_ninja.getIgnoreCheckSum())
      return(reportBadMessage(msg, "Wrong field count"));
  }
  
  string str_sats = flds[7];
  if(!isNumber(str_sats))
    return(reportBadMessage(msg, "Bad Sats"));
  
  int int_sats = atoi(str_sats.c_str());
  Notify("GPS_SATS", int_sats, "GPGGA");

  if(int_sats < 0)
    int_sats = 0;
  m_num_satellites = (unsigned int)(int_sats);

 
  if((flds[3] != "N") && (flds[3] != "S"))
    return(reportBadMessage(msg, "Bad N/S Hemisphere"));
  if((flds[5] != "W") && (flds[5] != "E")) 
    return(reportBadMessage(msg, "Bad E/W Hemisphere"));
 
  string str_lat = flds[2];
  string str_lon = flds[4];
 
  if(!isNumber(str_lat))  
    return(reportBadMessage(msg, "Bad Lat"));
  if(!isNumber(str_lon)) 
    return(reportBadMessage(msg, "Bad Lon"));

  
  double dbl_lat = latDDMMtoDD(str_lat);
  double dbl_lon = lonDDDMMtoDDD(str_lon);
  if(flds[3] == "S")
    dbl_lat = -dbl_lat;
  if(flds[5] == "W")
    dbl_lon = -dbl_lon;
  Notify(m_nav_prefix+"_LAT", dbl_lat, "GPGGA");
  Notify(m_nav_prefix+"_LON", dbl_lon, "GPGGA");
  Notify(m_nav_prefix+"_LONG", dbl_lon, "GPGGA");

  double x, y;
  bool ok = m_geodesy.LatLong2LocalGrid(dbl_lat, dbl_lon, y, x);
  if(ok) {
    m_nav_x = x;
    m_nav_y = y;
    Notify(m_nav_prefix+"_X", x, "GPRMC");
    Notify(m_nav_prefix+"_Y", y, "GPRMC");
  }

  return(true);
}

//---------------------------------------------------------
// Procedure: handleMsgGNGGA()
bool M1_8::handleMsgGNGGA(string msg)
{
  if(!strBegins(msg, "$GNGGA,"))
    return(false);

  // Remove the checksum info from end
  rbiteString(msg, '*');

  vector<string> flds = parseString(msg, ',');
  if(flds.size() != 14){
    if (!m_ninja.getIgnoreCheckSum() )
      return(reportBadMessage(msg, "Wrong field count"));
  }
  
  string str_sats = flds[7];
  if(!isNumber(str_sats))
    return(reportBadMessage(msg, "Bad Sats"));
  
  int int_sats = atoi(str_sats.c_str());
  Notify("GPS_SATS", int_sats, "GNGGA");

  if(int_sats < 0)
    int_sats = 0;
  m_num_satellites = (unsigned int)(int_sats);
  
  return(true);
}


//---------------------------------------------------------
//Procedure: handleMsgPSEAA()
//      Note: Proper NMEA format and checksum prior confirmed  
//   Example:
//   $PSEAA,-4.1,-1.4,301.5, ,32.6,-0.07,0.02,-1.00,0.10*5F
//        0,   1,   2,    3,4,   5,    6,   7,    8,   9*HH 
//
//  0	PSEAA
//  1	[Pitch]	Pitch, Degrees
//  2	[Roll]		Roll, Degrees
//  3	[Heading]	Heading, Degrees Magnetic
//  4	[Heave]	Heave
//  5	[Temperature]	Temperature in Elecctronics Box, Degrees Celsius
//  6	[Accel_X]	Aceeleration x, Forward, G
//  7	[Accel_Y]	Aceeleration y, Starboard, G
//  8	[Accel_Z]	Aceeleration z, Down, G 
//  9 	[Yaw_R]	Yaw Rate, Degrees per Second
// 
/*
// Procedure: handleMsgCPNVG()
//      Note: Proper NMEA format and checksum prior confirmed  
//   Example:
//   $CPNVG,160743.715,0000.00,N,00000.00,W,1,,,0,,,160743.715*64
//      0      1         2     3   4      5 6   9      12
//
//  0   CPNVG
//  1 [Timestamp]    Timestamp of the sentence
//  2 [Lat_NMEA]     Calculated latitude, in NMEA format
//  3 [LatNS_NMEA]   Hemisphere (N or S) of latitude
//  4 [Lon_NMEA]     Calculated longitude, in NMEA format
//  5 [LonEW_NMEA]   Hemisphere (E or W) of longitude
//  6 [PosQual]      Quality of position est (no GPS=0, otherwise=1)
//  7 [AltBottom]    Alt in meters from bottom, blank for USVs
//  8 [DepthTop]     Dep in meters from top, blank for USVs
//  9 [Heading]      Dir of travel in degs clockwise from true north
// 10 [Roll]         Degrees of roll
// 11 [Pitch]        Degrees of pitch
// 12 [NavTimestamp] Timestamp for time this pose/position*/

bool M1_8::handleMsgPSEAA(string msg)
{
  if(!strBegins(msg, "$PSEAA,"))
    return(false);
  
  // Remove the checksum info from end
  rbiteString(msg, '*');
  vector<string> flds = parseString(msg, ',');
  if(flds.size() != 10) {
    if(!m_ninja.getIgnoreCheckSum())
      return(reportBadMessage(msg, "Wrong field count"));
  }

  //KEEP THIS
  string str_hdg = flds[3];

  //KEEP THIS
  if(!isNumber(str_hdg)) 
    return(reportBadMessage(msg, "Bad Hdg"));
  //KEEP THIS
  double dbl_hdg = atof(str_hdg.c_str());
  m_nav_hdg = dbl_hdg;
  Notify(m_nav_prefix+"_HEADING", dbl_hdg, "PSEAA");
  Notify(m_compass_prefix+"_HEADING", dbl_hdg, "PSEAA");
  return(true);
}

//---------------------------------------------------------
// Procedure: handleMsgPSEAA_heading()
//      Note: Proper NMEA format and checksum prior confirmed  
// This function will only publish NAV_HEADING from the 
// $PSEAA message

bool M1_8::handleMsgPSEAA_heading(string msg)
{
  if(!strBegins(msg, "$PSEAA,"))
    return(false);
  
  // Remove the checksum info from end
  rbiteString(msg, '*');
  vector<string> flds = parseString(msg, ',');
  if(flds.size() != 10) {
    if(!m_ninja.getIgnoreCheckSum()) {
      string warning = "Wrong field count:" + uintToString(flds.size());      
      return(reportBadMessage(msg, warning));
    }
  }
  
  string str_hdg = flds[3];
  if(!isNumber(str_hdg)) 
    return(reportBadMessage(msg, "Bad Hdg"));  

  double dbl_hdg = atof(str_hdg.c_str());
  m_nav_hdg = dbl_hdg;
  Notify(m_nav_prefix+"_HEADING", dbl_hdg, "PSEAA");
  Notify(m_compass_prefix+"_HEADING", dbl_hdg, "PSEAA");
  m_last_nvg_msg_time = MOOSTime();
  return(true);
}


//---------------------------------------------------------
// Procedure: handleMsgPSEAB()
//      Note: Proper NMEA format and checksum prior confirmed  
//   Example: $PSEAB,24.8,15433,1.3,22.3,42308, , ,24.8, ,1.3,0.0,0.0,  ,  ,26*71
//                 0,   1,    2,  3,   4,    5,6,7,   8,9, 10, 11, 12,13,14,15*HH
//
//  0	PSEAB
//  1	[Voltage]		Voltage
//  2	[Charge_Remaining]	Charge Remaining or Fuel Remaining
//  3	[Current]		Current
//  4	[TemperatureC]		Temperature in Celsius
//  5	[Charge_Used]		Charge Used
//  6	[UTC_Time_CR]		UTC Time of Charge Reset
//  7	[UTC_Date_CR]		UTC Date of Charge Reset, ddmmyy
//  8	[P_Volt]		Port Voltage
//  9	[S_Volt]		Starboard Voltage
//  10	[Sys_Curr]		System Current
//  11	[P_Thr_Bat_Curr]	Port Thruster or Battery Current
//  12	[S_Thr_Bat_Curr]	Port Thruster or Battery Current
//  13	[B_Pump_1_Curr]	Bilge Pump 1 Current
//  14	[B_Pump_2_Curr]	Bilge Pump 2 Current
//  15	[USV_Hour_Meter]	USV Hour Meter
//  16	[P_Fuel_Remaining]	Port Fuel Remaining
//  17	[S_Fuel_Remaining]	Starboard Fuel Remaining
//
//	IGNORE THIS
//  0   CPRBS
//  1 [Timestamp]     Timestamp of the sentence.
//  2 [ID_Battery]    Unique ID of battery being reported on.
//  3 [V_Batt_Stack]  Voltage of the battery bank.
//  4 [V_Batt_Min]    Lowest voltage read from cells in bank.
//  5 [V_Batt_Max]    Highest voltage read from cells in bank. 
//  6 [TemperatureC]  Temperature in Celsius.

bool M1_8::handleMsgPSEAB(string msg)
{
  /*if(!strBegins(msg, "$CPRBS,"))
    return(false);
  
  // Remove the checksum info from end
  rbiteString(msg, '*');
  vector<string> flds = parseString(msg, ',');
  if(flds.size() != 7) {
    if(!m_ninja.getIgnoreCheckSum()) {
      string warning = "Wrong field count:" + uintToString(flds.size());
      return(reportBadMessage(msg, warning));
    }
  }
  
  string str_voltage = flds[3];
  if(!isNumber(str_voltage))
    return(reportBadMessage(msg, "Bad Voltage"));
  
  double dbl_voltage = atof(str_voltage.c_str());
  m_batt_voltage = dbl_voltage;
  Notify("M300_BATT_VOLTAGE", dbl_voltage, "CPRBS");
  return(true);*/
  if(!strBegins(msg, "$PSEAB,"))
    return(false);
  
  // Remove the checksum info from end
  rbiteString(msg, '*');
  vector<string> flds = parseString(msg, ',');
  if(flds.size() != 16) {
    if(!m_ninja.getIgnoreCheckSum()) {
      string warning = "Wrong field count:" + uintToString(flds.size());
      return(reportBadMessage(msg, warning));
    }
  }
  
  string str_voltage = flds[1];
  if(!isNumber(str_voltage))
    return(reportBadMessage(msg, "Bad Voltage"));
  
  double dbl_voltage = atof(str_voltage.c_str());
  m_batt_voltage = dbl_voltage;
  Notify("M300_BATT_VOLTAGE", dbl_voltage, "PSEAB");
  return(true);
}


//---------------------------------------------------------
// Procedure: reportBadMessage()
  
bool M1_8::reportBadMessage(string msg, string reason)
{
  reportRunWarning("Bad NMEA Msg: " + reason + ": " + msg);
  Notify("IM300_BAD_NMEA", reason + ": " + msg);
  return(false);
}

//---------------------------------------------------------
// Procedure: reportWarningsEvents()
//      Note: Get the AppCast-consistent events, warnings and
//            retractions from the sock ninja for posting

void M1_8::reportWarningsEvents()
{
  // Part 1: Handle Event Messages()
  list<string> events = m_ninja.getEvents();
  list<string>::iterator p;  
  for(p=events.begin(); p!=events.end(); p++) {
    string event_str = *p;
    reportEvent(event_str);
  }

  // Part 2: Handle Warning Messages()
  list<string> warnings = m_ninja.getWarnings();
  list<string> thrust_warnings = m_thrust.getWarnings();
  warnings.splice(warnings.end(), thrust_warnings);
  for(p=warnings.begin(); p!=warnings.end(); p++) {
    string warning_str = *p;
    reportRunWarning(warning_str);
  }

  // Part 3: Handle Retraction Messages()
  list<string> retractions = m_ninja.getRetractions();
  for(p=retractions.begin(); p!=retractions.end(); p++) {
    string retraction_str = *p;
    retractRunWarning(retraction_str);
  }
}
  
//------------------------------------------------------------
// Procedure: buildReport()
//
// -------------------------------------------
// Config:   max_r/t: 30/100      stale_check:  false
//           dr_mode: normal      stale_thresh: 15
// -------------------------------------------
// Drive     des_rud: -30         des_thrust_L: 0
// State:    des_thr: 40          des_thrust_R: 0
// -------------------------------------------
// Nav:      nav_x: 5968          nav_hdg: 0
//           nav_y: -6616.3       nav_spd: 0.5
// -------------------------------------------
// System:   voltage: 15.2        satellites: 0
// -------------------------------------------
// Comms:    Type: client         IPv4: 127.0.0.1 (of server)
//           Format: nmea         Port: 29500
//           Status: connected
// ---------------------------
// NMEA sentences:
// <--R     230  $PSEAA,105707.24,0000.00,N,00000.00,W,1,,,0,,,105707.24*64
// <--R     230  $CPRBS,105707.24,1,15.2,15.1,15.3,0*67
// <--R     230  $GPRMC,105707.24,A,0000.00,N,00000.00,W,1.1663,0,291263,0,E*76
//  S-->    231  $PSEAC,L,,,,*24


 
bool M1_8::buildReport() 
{
  string str_max_rud  = doubleToStringX(m_max_rudder,1);
  string str_max_thr  = doubleToStringX(m_max_thrust,1);
  string str_max_both = str_max_rud + "/" + str_max_thr;
  string str_des_rud  = doubleToStringX(m_thrust.getRudder(),1);
  string str_des_thr  = doubleToStringX(m_thrust.getThrust(),1);
  string str_des_thrL = doubleToStringX(m_thrust.getThrustLeft(),1);
  string str_des_thrR = doubleToStringX(m_thrust.getThrustRight(),1);
  string str_rot_hdg_tgt = doubleToStringX(m_rot_ctrl.getHeadingTarget(), 1);

  Notify("M4_DEBUG", str_des_thr);
  
  string str_sta_thr  = doubleToStringX(m_stale_threshold,1);
  string str_sta_ena  = boolToString(m_stale_check_enabled);

  string str_nav_x   = doubleToStringX(m_nav_x,1);
  string str_nav_y   = doubleToStringX(m_nav_y,1);
  string str_nav_hdg = doubleToStringX(m_nav_hdg,1);
  string str_nav_spd = doubleToStringX(m_nav_spd,1);
  string str_voltage = doubleToStringX(m_batt_voltage,1);
  string str_sats    = uintToString(m_num_satellites);


  string pd_ruth = padString(str_max_both, 10, false);
  string pd_drmo = padString(m_drive_mode, 10, false);
  string pd_drud = padString(str_des_rud, 10, false);
  string pd_dthr = padString(str_des_thr, 10, false);
  string pd_navx = padString(str_nav_x, 10, false);
  string pd_navy = padString(str_nav_y, 10, false);
  string pd_volt = padString(str_voltage, 10, false);

  
  m_msgs << "Config:    max_r/t: " << pd_ruth << "   stale_check:  " << str_sta_ena << endl;
  m_msgs << "           dr_mode: " << pd_drmo << "   stale_thresh: " << str_sta_thr << endl;
  m_msgs << "------------------------------------------------------" << endl;
  m_msgs << "Drive:     des_rud: " << pd_drud << "   des_thrust_L: " << str_des_thrL << endl;
  m_msgs << "State:     des_thr: " << pd_dthr << "   des_thrust_R: " << str_des_thrR << endl;
  m_msgs << "------------------------------------------------------" << endl;
  m_msgs << "Nav:       nav_x:   " << pd_navx << "   nav_hdg: " << str_nav_hdg << endl;
  m_msgs << "           nav_y:   " << pd_navy << "   nav_spd: " << str_nav_spd << endl;
  m_msgs << "------------------------------------------------------" << endl;
  m_msgs << "System:    voltage: " << pd_volt << "   satellites: " << str_sats << endl;
  m_msgs << "------------------------------------------------------" << endl;
  
  if ( m_rot_ctrl.getRotateInPlace() ) {
    m_msgs << "Rotation target heading: " << str_rot_hdg_tgt << endl;
    if ( m_rot_ctrl.checkClearToRotate( m_nav_x, m_nav_y, m_curr_time ) )
      m_msgs << "All clear to rotate.                                " << endl;
    m_msgs << "------------------------------------------------------" << endl;
  }
  
  list<string> summary_lines = m_ninja.getSummary();
  list<string>::iterator p;
  for(p=summary_lines.begin(); p!=summary_lines.end(); p++) {
    string line = *p;
    m_msgs << line << endl;
  }

  return(true);
}




