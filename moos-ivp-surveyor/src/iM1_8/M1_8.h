/*****************************************************************/
/*    NAME: Tyler Errico                                         */
/*    ORGN: Robotics Research Center, USMA, West Point, NY       */
/*    FILE: M1_8.h                                               */
/*    DATE: 03 AUGUST 2022                                       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/

#ifndef M1_8_HEADER
#define M1_8_HEADER

#include <string>
#include "SockNinja.h"
#include "Thruster.h"
//#include "VehRotController.h"
#include "MOOS/libMOOSGeodesy/MOOSGeodesy.h"
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class M1_8 : public AppCastingMOOSApp
{
public:
  M1_8();
  ~M1_8();

protected: // Standard public MOOSApp functions
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

protected: // Standard protected MOOS App functions
  bool buildReport();
  void registerVariables();

protected: // App Specific functions
  void reportWarningsEvents();
  void sendMessagesToSocket();
  void readMessagesFromSocket();

  bool handleConfigIgnoreMsg(std::string);
  bool handleMsgGPRMC(std::string);
  bool handleMsgGNRMC(std::string);
  bool handleMsgGPGGA(std::string);
  bool handleMsgGNGGA(std::string);
  //bool handleMsgCPNVG(std::string);
  //bool handleMsgCPNVG_heading(std::string);
  bool handleMsgPSEAA(std::string);
  bool handleMsgPSEAA_heading(std::string);
  //bool handleMsgCPRBS(std::string);
  bool handleMsgPSEAB(std::string);
  bool handleMsgGPVTG(std::string);
  bool handleMsgPSEAG(std::string);

  bool reportBadMessage(std::string msg, std::string reason="");
  bool GeodesySetup();
  void checkForStalenessOrAllStop();
  void convertThrustVals(double thrustL, double thrustR,
			 double& pseac_msg_thrust,
			 double& pseac_msg_thrust_diff);

  

private: // Config variables
  double       m_max_rudder;       // MAX_RUDDER
  double       m_max_thrust;       // MAX_THRUST
  std::string  m_drive_mode;       // DRIVE_MODE
  
  // heading source variables
  std::string  m_heading_source;   // gps, imu, or auto;
  double       m_stale_nvg_msg_thresh;
  double       m_last_nvg_msg_time;
  bool         m_ignore_checksum;

  std::set<std::string> m_ignore_msgs;

  // Requred mods for HydroMAN integration 
  std::string  m_nav_prefix;
  std::string  m_gps_prefix;
  std::string  m_compass_prefix;
  
private: // State variables
  CMOOSGeodesy m_geodesy;
  SockNinja    m_ninja;
  Thruster     m_thrust;
  //VehRotController m_rot_ctrl;
  std::string m_searobot_mode;

  bool         m_ivp_allstop;
  bool         m_moos_manual_override;

  // Stale Message Detection
  bool         m_stale_check_enabled;
  bool         m_stale_mode;
  double       m_stale_threshold;
  unsigned int m_count_stale;
  double       m_tstamp_des_rudder;
  double       m_tstamp_des_thrust;

  unsigned int m_num_satellites;
  double       m_batt_voltage;
  double       m_nav_x;
  double       m_nav_y;
  double       m_nav_hdg;
  double       m_nav_spd;
  double	m_heading;
  double	m_speed;
  double	m_hdg_diff;
  double	m_min_speed;
  double	m_max_speed;
  double	m_s1;
  double	m_s2;

  unsigned int m_bad_nmea_semantic;

  // GPS denied navigation related
  bool         m_gps_blocked;


  //new controller variables
  bool m_legacy_controller;
  std::set<std::string> m_valid_USV_control_modes;
  
};

#endif 


