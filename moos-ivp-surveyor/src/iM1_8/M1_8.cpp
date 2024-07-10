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

  m_searobot_mode     = "T";
  m_op_mode           = "unknown";
  m_legacy_controller = true;
  
  m_valid_USV_control_modes.insert("L");
  m_valid_USV_control_modes.insert("T");
  m_valid_USV_control_modes.insert("G");
  // add more if needed

  m_sent_rc_vis = false;
  m_post_rc_vis = false;
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
    else if(param == "post_rc_visuals")
      handled = setBooleanOnString(m_post_rc_vis, value); 
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
    else if(param == "legacy")
	handled = setBooleanOnString(m_legacy_controller, value);
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

    else if(param == "rev_factor") {
      handled = m_thrust.setRevFactor(value);
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
  Register("DESIRED_THRUST",  0);
  Register("DESIRED_RUDDER",  0);
  Register("DESIRED_HEADING", 0);
  Register("DESIRED_SPEED", 0);
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

	m_des_heading = dval;
    }
    else if(key == "DESIRED_SPEED") {
    
	m_des_speed = dval;
    }
    else if(key == "BLOCK_GPS") 
      setBooleanOnString(m_gps_blocked, sval);
 
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

  // post visuals of RC control mode
  if(m_post_rc_vis){
    if (m_op_mode == "RC Receiver"){
      sendRCVisuals();
      
    } else if ( (m_op_mode != "RC_Receiver") && m_sent_rc_vis){
      // clear the existing visual
      XYCircle RC_control_marker_clear(0.0, 0.0, 6.0);
      RC_control_marker_clear.set_active(false);
      Notify("VIEW_CIRCLE", RC_control_marker_clear.get_spec());
      m_sent_rc_vis = false; 
    }

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
  
  // build PSEAC message
  //  SeaRobotics proprietary sentence
  //        1    2   3   4   5
  //        |    |   |   |   |
  // $PSEAC,c--c,x.x,x.x,x.x,c--c*hh<CR><LF>
    
  string msg = "PSEAC,";

  // Position 1) - USV control mode
  // The mode is handled in checkForStalenessOrAllStop()
  // m_searobot_mode is either L, G, or T, but check again to be safe
  if (m_valid_USV_control_modes.count(m_searobot_mode))
    msg += m_searobot_mode + ",";
  else{
    reportRunWarning("Current control mode " + m_searobot_mode
		     + " is not valid : Stopping sending commands");
    return;
  }

  if (m_searobot_mode == "L"){
    msg += "0,0,,,";
    
  } else if (m_searobot_mode == "G"){
    
    // Spead/Heading Controller on SeaRobotics Surveyor M1.8
    // Position 2) - Heading   
    msg += doubleToStringX(m_des_heading) + ",";
    
    // Position 3) - Speed
    //convert from m/s to knots
    double speed_knots = m_des_speed *1.943844;
    msg += to_string(speed_knots) +",";
    
    // Position 4) - Thrust Diff - Not used in G mode
    msg += ",";
    
    // Position 5) - Command string
    // not used here
    msg += ",";
    
  } else if (m_searobot_mode == "T") {
    
    // Do the standard thruster mapping and allocation
    // to get thrustL and thrustR
    
    // We will later convert these values into thrust and
    // thrust diff to send to the frontseat.
    
    double thrustL, thrustR;
    m_thrust.setDriveMode(m_drive_mode);
    
    // Calculate the right and left thruster values
    // with whatever DriveMode is selected.
    m_thrust.calcDiffThrust();
    // Update differential thrust values
    thrustL = m_thrust.getThrustLeft();
    thrustR = m_thrust.getThrustRight();
    
    // Now convert
    double pseac_msg_thrust, pseac_msg_thrust_diff;
    convertThrustVals(thrustL, thrustR,
		      pseac_msg_thrust, pseac_msg_thrust_diff);
    
    // Position 2) - Heading
    // do we even need to send heading in T mode?  Need to check this
    msg += doubleToStringX(m_des_heading) + ",";
    //msg += ",";
    
    // Position 3) - Thrust
    msg += doubleToStringX(pseac_msg_thrust) + ",";
    
    // Position 4) - Thrust Diff
    msg += doubleToStringX(pseac_msg_thrust_diff) + ",";
    
    // Position 5) - Command string
    // not used here
    msg += ",";
    
    // Publish command to the MOOSDB for logging
    Notify("PSEAC_THRUST", pseac_msg_thrust);
    Notify("PSEAC_THRUST_DIFF", pseac_msg_thrust_diff);
    Notify("PSEAC_THRUST_L", thrustL);
    Notify("PSEAC_THRUST_R", thrustR);
    
  }
  
  
  // Pack and ship it
  msg = "$"+msg+"*"+checksumHexStr(msg) + "\r\n";
  m_ninja.sendSockMessage(msg);

  // Publish entire command to MOOSDB for logging/debugging
  Notify("PSEAC_MSG", msg); 
  
  return;  
}

//---------------------------------------------------------
// Procedure: readMessagesFromSocket()
//      Note: Messages returned from the SockNinja have been
//            confirmed to be valid NMEA format and checksum

void M1_8::readMessagesFromSocket()
{
  list<string> incoming_msgs = m_ninja.getSockMessages();
  list<string>::iterator p;
  int nothing;
  for(p=incoming_msgs.begin(); p!=incoming_msgs.end(); p++) {
    string msg = *p;
    msg = biteString(msg, '\r'); // Remove CRLF
    Notify("IM1_8_RAW_NMEA", msg);

    bool handled = false;
    if(m_ignore_msgs.count(msg.substr (0,6)) >= 1) 
      handled = true;
    else if(strBegins(msg, "$GPGGA"))
      handled = handleMsgGPGGA(msg);    
    else if(strBegins(msg, "$GPVTG"))
      handled = handleMsgGPVTG(msg); // Added on 08-09-2022 by Tyler Errico for 2022 Surveryors
    else if(strBegins(msg, "$PSEAA"))
      handled = handleMsgPSEAA_heading(msg);
    else if(strBegins(msg, "$PSEAB"))
      handled = handleMsgPSEAB(msg);
    else if(strBegins(msg, "$PSEAD"))
      handled = handleMsgPSEAD(msg);
    else
      reportBadMessage(msg, "Unknown NMEA Key");
            
    if(!handled)
      m_bad_nmea_semantic++;
  }
}

//---------------------------------------------------------
// Procedure: handleConfigIgnoreMsg()
//  Examples: ignore_msg = $GPGLL
//            ignore_msg = $GPGLL, GPGSV

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

  if (m_legacy_controller)
    m_searobot_mode = "G";
  else
    m_searobot_mode = "T";
  
  if(m_ivp_allstop) {
    m_thrust.setRudder(0);
    m_thrust.setThrust(0);
	m_searobot_mode = "L";
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
// Procedure: handleMsgGPVTG()
//      Note: Proper NMEA format and checksum prior confirmed  
//      Note: Trying to get speed
//   Example:
//   $GPVTG,000.0,T,012.7,M,000.11,N,0000.21,K,D*11
//                                                 
//  0   $GPVTG
//  1 [Course] 	Course 309.62 	degrees measured heading
//  2 [Reference] 	Reference T 	True
//  3 [Course] 	Course	Degrees measured heading
//  4 [Reference]    	M, magnetic
//  5 [Speed]   	Speed in knots, measured horizontal speed
//  6 [Units]     	N - Knots
//  7 [Speed]    	Speed in km/hr measured horizontal speed
//  8 [Units]    	K kilometers per hour
//  9 [Mode]      	A A=Autonomous, D=DGPS, E=DR


bool M1_8::handleMsgGPVTG(string msg)
{
  if(!strBegins(msg, "$GPVTG,"))
    return(false);

  // Remove the checksum info from end
  rbiteString(msg, '*');

  vector<string> flds = parseString(msg, ',');
  if(flds.size() != 10) {
    if(!m_ninja.getIgnoreCheckSum())
      return(reportBadMessage(msg, "Wrong field count"));
  }
  
  string str_speed = flds[7];
  if(!isNumber(str_speed))
    return(reportBadMessage(msg, "Bad Speed"));
  
  double double_speed = atof(str_speed.c_str()); //km/hour
  double_speed = double_speed*1000; //convert to meters
  double_speed = double_speed/3600; //convert to seconds
  
  m_nav_spd  = double_speed;
  Notify(m_nav_prefix+"_SPEED", double_speed, "GPVTG");
  
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
    Notify(m_nav_prefix+"_X", x, "GPGGA");
    Notify(m_nav_prefix+"_Y", y, "GPGGA");
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
// Procedure: handleMsgPSEAD()
//      Note: Proper NMEA format and checksum prior confirmed  
//   Example: $PSEAD,c--c,x.x,x.x,x.x,c--c,a,x,x*hh<CR><LF>
//                 0,   1,  2,  3,  4,   5,6,7,8*HH
//
//  0	PSEAD
//  1	[Control Mode]          Low level control mode
//  2	[Heading]	        Degrees Charge Remaining or Fuel Remaining
//  3	[Thrust]		Thrust
//  4	[Thrust angle]		Thrust angle or thrust difference
//  5	[Command String]       	see $PSEAC
//  6	[Transmission Position]	T,F,R,N (see documentation)
//  7	[Operator sent to]	See documentation
//  8	[Operator in control]   See doucmentation


bool M1_8::handleMsgPSEAD(string msg)
{
  if(!strBegins(msg, "$PSEAD,"))
    return(false);
  
  // Remove the checksum info from end
  rbiteString(msg, '*');
  vector<string> flds = parseString(msg, ',');
  if(flds.size() != 9) {
    if(!m_ninja.getIgnoreCheckSum()) {
      string warning = "Wrong field count:" + uintToString(flds.size());
      return(reportBadMessage(msg, warning));
    }
  }
  
  string msg_str_op_in_control= flds[8];
  if(!isNumber(msg_str_op_in_control))
    return(reportBadMessage(msg, "Bad Operator Number"));

  int int_op_in_control = atoi(msg_str_op_in_control.c_str());
  string str_op_in_control = "";

  if (int_op_in_control == 1){
    str_op_in_control = "Primary";
  } else if (int_op_in_control == 2){
    str_op_in_control = "Secondary";
  } else if (int_op_in_control == 3){
    str_op_in_control = "Backup comms link to primary";
  } else if (int_op_in_control == 5){
    str_op_in_control = "RC Receiver";
  } else if (int_op_in_control == 6){
    str_op_in_control = "Obstacle Avoid Sys";
  } else {
    return(reportBadMessage(msg, "Bad Operator Number"));
  } 

  m_op_mode = str_op_in_control; 
  
  Notify("IM_8_OP_CONTROL_MODE", m_op_mode, "PSEAD");
  return(true);
}



//---------------------------------------------------------
// Procedure: reportBadMessage()
  
bool M1_8::reportBadMessage(string msg, string reason)
{
  reportRunWarning("Bad NMEA Msg: " + reason + ": " + msg);
  Notify("IM1_8_BAD_NMEA", reason + ": " + msg);
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
//  Procedure:  convertThrustVals
//              Converts from thrust left and right to
//              thrust and thrust diff per SeaRobotics email:
//              "thrust = (port_thrust + stbd_thrust) / 2"
//              "thrust_diff = (port_thrust - stbd_thrust) / 2"
// 
void M1_8::convertThrustVals(double thrustL, double thrustR,
			     double& pseac_msg_thrust, double& pseac_msg_thrust_diff)
{
  // port is left, starboard is right
  pseac_msg_thrust      = (thrustL + thrustR) / 2.0;
  pseac_msg_thrust_diff = (thrustL - thrustR) / 2.0; 
  
  return; 
}

//------------------------------------------------------------
// Procedure:  sendRCVisuals()
//             posts helpful visuals to pMarineViewer to show
//             operators on shoreside when the RC control mode
//             is active.

void M1_8::sendRCVisuals()
{
  // build circle message
  std::string label = m_host_community + "_RC_control"; 
  XYCircle RC_control_marker(m_nav_x, m_nav_y, 6.0);
  
  RC_control_marker.set_color("edge","blue");
  RC_control_marker.set_label(label);
  RC_control_marker.set_edge_size(3.0);
  Notify("VIEW_CIRCLE", RC_control_marker.get_spec());

  m_sent_rc_vis = true;
  
  return; 
  
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
  string str_des_hdg  = doubleToStringX(m_des_heading, 1);
  string str_des_spd  = doubleToStringX(m_des_speed, 1);
  
  
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
  string pd_des_hdg = padString(str_des_hdg, 10, false);
  string pd_des_spd = padString(str_des_spd, 10, false);
  string pd_op_mode = padString(m_op_mode, 20, false); 
  

 
  m_msgs << "Config:    max_r/t: " << pd_ruth << "   stale_check:  " << str_sta_ena << endl;
  m_msgs << "           dr_mode: " << pd_drmo << "   stale_thresh: " << str_sta_thr << endl;
  
  if (m_legacy_controller){
    m_msgs << "   ctrl/thrust map: legacy       Op Mode: " << pd_op_mode   << endl;
  } else {
    m_msgs << "   ctrl/thrust map: turbo        Op Mode: " << pd_op_mode  << endl;
  }
  
  m_msgs << "------------------------------------------------------" << endl;
  m_msgs << "Drive:     des_rud: " << pd_drud << "   des_thrust_L: " << str_des_thrL << endl;
  m_msgs << "State:     des_thr: " << pd_dthr << "   des_thrust_R: " << str_des_thrR << endl;
  m_msgs << "------------------------------------------------------" << endl;
  m_msgs << "Nav:       nav_x:   " << pd_navx << "   nav_hdg: " << str_nav_hdg << endl;
  m_msgs << "           nav_y:   " << pd_navy << "   nav_spd: " << str_nav_spd << endl;
  m_msgs << "------------------------------------------------------" << endl;
  m_msgs << "System:    voltage: " << pd_volt << "   satellites: " << str_sats << endl;
  m_msgs << "------------------------------------------------------" << endl;
  m_msgs << "                                                      " << endl;
  m_msgs << "------------------------------------------------------" << endl;
  m_msgs << "Desired:   Speed: " << pd_des_spd <<"   Heading: " << str_des_hdg << endl;
  m_msgs << "------------------------------------------------------" << endl;
  m_msgs << "                                                      " << endl;
  //m_msgs << "DESIRED_HEADING: " << m_heading << "    DESIRED_SPEED: " << m_speed << endl;
  //  m_msgs << "Heading Diff: " << m_hdg_diff << " Min_Speed_Angle: " << m_s1 << " Max_Speed_Angle: " <<  m_s2 <<endl;
  // m_msgs << "Min Speed: " << m_min_speed << " Max Speed: " << m_max_speed << endl;
  

  list<string> summary_lines = m_ninja.getSummary();
  list<string>::iterator p;
  for(p=summary_lines.begin(); p!=summary_lines.end(); p++) {
    string line = *p;
    m_msgs << line << endl;
  }

  return(true);
}
