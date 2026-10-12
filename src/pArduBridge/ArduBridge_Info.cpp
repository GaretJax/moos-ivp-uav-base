/************************************************************/
/*    NAME: Steve Carter Feujo Nomeny                       */
/*    ORGN: NTNU, MIT                                       */
/*    FILE: ArduBridge_Info.cpp                             */
/*    DATE: September 9th, 2024                             */
/************************************************************/

#include <cstdlib>
#include "ArduBridge_Info.h"
#include "ColorParse.h"

using namespace std;

//----------------------------------------------------------------
// Procedure: showSynopsis

void showSynopsis()
{
  blk("SYNOPSIS:                                                       ");
  blk("------------------------------------                            ");
  blk("  pArduBridge connects a MOOS community to an ArduPilot vehicle ");
  blk("  through MAVSDK/MAVLink. It publishes navigation, health, GPS, ");
  blk("  landed-state, policy, and command-result data, and translates ");
  blk("  MOOS helm setpoints and explicit commands for Plane or Copter.");
}

//----------------------------------------------------------------
// Procedure: showHelpAndExit

void showHelpAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("Usage: pArduBridge file.moos [OPTIONS]                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("Options:                                                        ");
  mag("  --alias","=<ProcessName>                                      ");
  blk("      Launch pArduBridge with the given process name         ");
  blk("      rather than pArduBridge.                           ");
  mag("  --example, -e                                                 ");
  blk("      Display example MOOS configuration block.                 ");
  mag("  --help, -h                                                    ");
  blk("      Display this help message.                                ");
  mag("  --interface, -i                                               ");
  blk("      Display MOOS publications and subscriptions.              ");
  mag("  --version,-v                                                  ");
  blk("      Display the release version of pArduBridge.        ");
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
  blu("pArduBridge Example MOOS Configuration                   ");
  blu("=============================================================== ");
  blk("                                                                ");
  blk("ProcessConfig = pArduBridge                              ");
  blk("{                                                               ");
  blk("  AppTick   = 10                                                ");
  blk("  CommsTick = 10                                                ");
  blk("                                                                ");
  blk("  url          = 0.0.0.0:14550 // or ttyACM0:115200 for serial ");
  blk("  url_protocol = udp             // udp, tcp, or serial          ");
  blk("  vehicle_type = copter          // copter or plane              ");
  blk("  passive      = false           // telemetry-only observer mode ");
  blk("  takeoff_altitude = 10          // meters AGL; Copter default   ");
  blk("  telemetry_position_hz = 5      // defaults apply to all vehicles");
  blk("  telemetry_attitude_hz = 5      // override per mission if needed");
  blk("  telemetry_velocity_hz = 5      // must be greater than zero     ");
  blk("  helm_setpoint_timeout = 2      // Copter stale-setpoint hold (s)");
  blk("  guided_mode_timeout = 30       // ENTER_GUIDED retry timeout (MOOS s)");
  blk("  precision_loiter_enter_loiter = true // false requires FC Loiter already ");
  blk("  show_loiter_visuals = true    // false suppresses bridge loiter markers ");
  blk("  command_groundspeed = true     // forced true for Copter       ");
  blk("  require_verified_rally = false // true blocks ARM until rally readback ");
  blk("  is_sim = false                 // Plane SITL mission setup only");
  blk("  geodesy = local_grid           // or utm, to match USE_UTM apps ");
  blk("  logger = false                                               ");
  blk("  prefix = UAV                                                 ");
  blk("  vname  = alpha                                               ");
  blk("  vcolor = yellow                                              ");
  blk("                                                                ");
  blk("}                                                               ");
  blk("                                                                ");
  exit(0);
}


//----------------------------------------------------------------
// Procedure: showInterfaceAndExit

void showInterfaceAndExit()
{
  blk("                                                                ");
  blu("=============================================================== ");
  blu("pArduBridge INTERFACE                                    ");
  blu("=============================================================== ");
  blk("                                                                ");
  showSynopsis();
  blk("                                                                ");
  blk("SUBSCRIPTIONS:                                                  ");
  blk("------------------------------------                            ");
  blk("  ARDU_COMMAND accepts:                                         ");
  blk("    FLY_WAYPOINT | DO_TAKEOFF | RESET_SPEED_MIN                ");
  blk("    RETURN_TO_LAUNCH | HOLD_POSITION | LOITER | BRAKE_AND_HOLD ");
  blk("    LOITER_FC | ENTER_GUIDED | PRECISION_LOITER                ");
  blk("    PRECISION_LOITER_OFF | AUTOLAND | SURVEY | DO_VORONOI      ");
  blk("    VIZ_HOME                                                    ");
  blk("                                                                ");
  blk("  ARM_UAV = true|false                                         ");
  blk("  RETURN_TO_LAUNCH = true|false                                ");
  blk("  AUTOLAND = true|false                                        ");
  blk("  NEXT_WAYPOINT = lat=...,lon=...,x=...,y=...,vname=...        ");
  blk("  HELM_STATUS, AUTOPILOT_MODE, MOOS_MANUAL_OVERRIDE            ");
  blk("  CHANGE_SPEED, CHANGE_COURSE, CHANGE_ALTITUDE                 ");
  blk("  DEAD_MAN_POST_INTERRUPT                                      ");
  blk("  SITL_SPEEDUP = 1..100 // simulation-only SIM_SPEEDUP parameter");
  blk("  YAW_ALIGN_REQUEST = id=...#angle=...#rate=...                 ");
  blk("    Copter only; signed relative degrees [-180,180], deg/s.     ");
  blk("  RALLY_UPLOAD = id=...#x=...#y=...#altitude=...                ");
  blk("    One grounded/disarmed rally point; altitude is whole metres ");
  blk("    above Home and may not be closer than Home. ARM can require ");
  blk("    a verified FC ACK/readback with require_verified_rally=true.");
  blk("                                                                ");
  blk("  Compatibility registrations: FLY_WAYPOINT, DO_TAKEOFF, LOITER,");
  blk("    SURVEY, RESET_SPEED_MIN, VIZ_HOME (prefer ARDU_COMMAND).    ");
  blk("                                                                ");
  blk("  DESIRED_HEADING                                             ");
  blk("    Plane: desired course over ground.                         ");
  blk("    Copter: absolute yaw heading.                              ");
  blk("  DESIRED_SPEED                                               ");
  blk("    Plane: airspeed by default. Copter: ground speed.          ");
  blk("  DESIRED_ALTITUDE                                            ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  <PREFIX> is the configured prefix value (UAV by default).     ");
  blk("                                                                ");
  blk("  Navigation                                                    ");
  blk("    <PREFIX>_LAT, <PREFIX>_LON, <PREFIX>_X, <PREFIX>_Y         ");
  blk("    <PREFIX>_ALTITUDE, <PREFIX>_DEPTH, <PREFIX>_SPEED          ");
  blk("    <PREFIX>_YAW      // FC yaw, clockwise from true north      ");
  blk("    <PREFIX>_HEADING  // FC yaw converted to local-grid bearing");
  blk("    <PREFIX>_COG      // GPS course converted to local grid    ");
  blk("                                                                ");
  blk("  FC health, state, and battery                                 ");
  blk("    <PREFIX>_HEALTH_AVAILABLE // detailed health received      ");
  blk("    <PREFIX>_HEALTH_AGE       // age in seconds                 ");
  blk("    <PREFIX>_HEALTH_GYRO, <PREFIX>_HEALTH_ACCEL                ");
  blk("    <PREFIX>_HEALTH_MAG, <PREFIX>_HEALTH_LOCAL_POSITION        ");
  blk("    <PREFIX>_HEALTH_GLOBAL_POSITION, <PREFIX>_HEALTH_HOME_POSITION");
  blk("      // MAVSDK flags; use GPS fields to assess the native fix ");
  blk("    <PREFIX>_HEALTH_ALL_OK, <PREFIX>_IS_ARMABLE                ");
  blk("    <PREFIX>_IS_ARMED, <PREFIX>_GUIDED_MODE_ACTIVE             ");
  blk("      // GUIDED_MODE_ACTIVE requires native mode confirmation  ");
  blk("    <PREFIX>_BATTERY_SOC, <PREFIX>_BATTERY_VOLTAGE             ");
  blk("    <PREFIX>_BATTERY_DATA_VALID, <PREFIX>_BATTERY_DATA_AGE     ");
  blk("      // valid requires finite charge data no more than 3 s old");
  blk("                                                                ");
  blk("  Command and safety policy                                     ");
  blk("    <PREFIX>_COMMAND_RESULT // all ARDU_COMMAND outcomes;       ");
  blk("                            // takeoff also reports COMPLETED   ");
  blk("    <PREFIX>_ARM_POLICY_READY, <PREFIX>_ARM_POLICY_REASON       ");
  blk("    <PREFIX>_DISARM_POLICY_READY, <PREFIX>_DISARM_POLICY_REASON ");
  blk("    <PREFIX>_LAND_POLICY_READY, <PREFIX>_LAND_POLICY_REASON     ");
  blk("    <PREFIX>_GUIDED_OWNERSHIP_STATE                            ");
  blk("      // GROUNDED_DISARMED, FC_TAKEOFF_ACKNOWLEDGED,            ");
  blk("      // MISSING_OWNER, TELEMETRY_UNCERTAIN, NOT_APPLICABLE     ");
  blk("    <PREFIX>_GUIDED_OWNERSHIP_WARNING // grace: 2 MOOS seconds ");
  blk("    <PREFIX>_SITL_SPEEDUP // last successful requested value   ");
  blk("    <PREFIX>_YAW_ALIGN_RESULT // ACCEPTED means FC ACK only     ");
  blk("                                                                ");
  blk("  Rally and brake-and-hold transactions                         ");
  blk("    <PREFIX>_RALLY_RESULT, <PREFIX>_RALLY_STATE                ");
  blk("    <PREFIX>_RALLY_ID, <PREFIX>_RALLY_READY                    ");
  blk("      // ACK/readback status and connection-sensitive readiness");
  blk("    <PREFIX>_BRAKE_HOLD_PHASE, <PREFIX>_BRAKE_HOLD_REASON      ");
  blk("    <PREFIX>_BRAKE_HOLD_SPEED, <PREFIX>_BRAKE_HOLD_MOTION_AGE ");
  blk("      // native horizontal m/s and real receive age            ");
  blk("    <PREFIX>_BRAKE_HOLD_SAMPLES, <PREFIX>_BRAKE_HOLD_ID       ");
  blk("                                                                ");
  blk("  Native attitude                                               ");
  blk("    <PREFIX>_ROLL, <PREFIX>_PITCH // right-bank/nose-up positive");
  blk("    <PREFIX>_ATTITUDE_AVAILABLE, <PREFIX>_ATTITUDE_AGE         ");
  blk("      // finite sample received; AGE is real seconds, else -1  ");
  blk("    <PREFIX>_ATTITUDE_SAMPLE_TIME // original MOOS receipt time");
  blk("    <PREFIX>_ATTITUDE_TIME_USEC   // original FC boot time     ");
  blk("    <PREFIX>_ATTITUDE_SAMPLES     // atomic native sample window");
  blk("                                                                ");
  blk("  GPS, landed state, and rangefinder                            ");
  blk("    <PREFIX>_GPS_AVAILABLE // telemetry received, fix may fail ");
  blk("    <PREFIX>_GPS_FIX_TYPE // 0=None, 2=2D, 3=3D, 6=RTK fixed  ");
  blk("    <PREFIX>_GPS_SATELLITES, <PREFIX>_GPS_HDOP                ");
  blk("    <PREFIX>_GPS_VDOP, <PREFIX>_GPS_AGE                       ");
  blk("    <PREFIX>_LANDED_STATE_AVAILABLE, <PREFIX>_LANDED_STATE     ");
  blk("      // ON_GROUND, IN_AIR, TAKING_OFF, LANDING, or UNKNOWN    ");
  blk("    <PREFIX>_LANDED_STATE_AGE                                  ");
  blk("    <PREFIX>_RANGEFINDER_AVAILABLE, <PREFIX>_RANGEFINDER_AGE   ");
  blk("      // available means a finite reading no more than 2 s old ");
  blk("    <PREFIX>_RANGEFINDER_DISTANCE                              ");
  blk("    <PREFIX>_RANGEFINDER_MIN_DISTANCE                          ");
  blk("    <PREFIX>_RANGEFINDER_MAX_DISTANCE                          ");
  blk("    <PREFIX>_RANGEFINDER_ORIENTATION_ROLL/PITCH/YAW            ");
  blk("                                                                ");
  blk("  Landing-target telemetry                                      ");
  blk("    <PREFIX>_LANDING_TARGET_AVAILABLE // fresh within 0.5 s    ");
  blk("    <PREFIX>_LANDING_TARGET_AGE                                ");
  blk("    <PREFIX>_LANDING_TARGET_TIME_USEC                          ");
  blk("    <PREFIX>_LANDING_TARGET_SYSTEM_ID                           ");
  blk("    <PREFIX>_LANDING_TARGET_COMPONENT_ID, <PREFIX>_LANDING_TARGET_TARGET_NUM");
  blk("    <PREFIX>_LANDING_TARGET_FRAME, <PREFIX>_LANDING_TARGET_TYPE ");
  blk("    <PREFIX>_LANDING_TARGET_POSITION_VALID                      ");
  blk("    <PREFIX>_LANDING_TARGET_DISTANCE                            ");
  blk("    <PREFIX>_LANDING_TARGET_ANGLE_X, <PREFIX>_LANDING_TARGET_ANGLE_Y");
  blk("    <PREFIX>_LANDING_TARGET_SIZE_X, <PREFIX>_LANDING_TARGET_SIZE_Y");
  blk("    <PREFIX>_LANDING_TARGET_X, <PREFIX>_LANDING_TARGET_Y       ");
  blk("    <PREFIX>_LANDING_TARGET_Z                                  ");
  blk("    <PREFIX>_LANDING_TARGET_Q_W, <PREFIX>_LANDING_TARGET_Q_X   ");
  blk("    <PREFIX>_LANDING_TARGET_Q_Y, <PREFIX>_LANDING_TARGET_Q_Z   ");
  blk("                                                                ");
  blk("  Unprefixed bridge and Helm publications                       ");
  blk("    PARDUBRIDGE_PASSIVE_STATE, PARDUBRIDGE_PASSIVE_CONNECTED   ");
  blk("    AUTOPILOT_MODE       // bridge Helm state, not raw FC mode  ");
  blk("    CONTROL_AUTHORITY    // FC/manual or MOOS Helm ownership    ");
  blk("    HELM_SETPOINT_FRESH, HELM_SETPOINT_AGE                     ");
  blk("    HELM_SETPOINT_TIMEOUT_LATCHED                              ");
  blk("    MOOS_MANUAL_OVERRIDE, RETURN_UPDATE, TOWAYPT_UPDATE        ");
  blk("    SURVEY_UPDATE, CONST_ALTITUDE_UPDATE, VIEW_*                ");
  blk("                                                                ");
  exit(0);
}

//----------------------------------------------------------------
// Procedure: showReleaseInfoAndExit

void showReleaseInfoAndExit()
{
  //showReleaseInfo("pArduBridge", "gpl");
  exit(0);
}
