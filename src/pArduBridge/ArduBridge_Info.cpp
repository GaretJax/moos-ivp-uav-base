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
  blk("  command_groundspeed = true     // forced true for Copter       ");
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
  blk("  ARDU_COMMAND = FLY_WAYPOINT | DO_TAKEOFF | RESET_SPEED_MIN   ");
  blk("                 RETURN_TO_LAUNCH | HOLD_POSITION | LOITER     ");
  blk("                 BRAKE_AND_HOLD | LOITER_FC                    ");
  blk("                 PRECISION_LOITER | PRECISION_LOITER_OFF        ");
  blk("                 AUTOLAND | SURVEY | DO_VORONOI | VIZ_HOME      ");
  blk("  ARM_UAV = true|false                                         ");
  blk("  RETURN_TO_LAUNCH = true|false                                ");
  blk("  AUTOLAND = true|false                                        ");
  blk("  NEXT_WAYPOINT = lat=...,lon=...,x=...,y=...,vname=...        ");
  blk("  HELM_STATUS, AUTOPILOT_MODE, MOOS_MANUAL_OVERRIDE            ");
  blk("  CHANGE_SPEED, CHANGE_COURSE, CHANGE_ALTITUDE, DEAD_MAN_POST_INTERRUPT ");
  blk("  SITL_SPEEDUP = 1..100        // Simulation-only SIM_SPEEDUP parameter   ");
  blk("  Compatibility registrations: FLY_WAYPOINT, DO_TAKEOFF, LOITER, ");
  blk("    SURVEY, RESET_SPEED_MIN, VIZ_HOME (use ARDU_COMMAND path)   ");
  blk("                                                                ");
  blk("  DESIRED_HEADING                                                              ");
  blk("    Plane: desired course over ground. Copter: absolute yaw heading.            ");
  blk("  DESIRED_SPEED                                                              ");
  blk("    Plane: airspeed by default. Copter: ground speed.                          ");
  blk("  DESIRED_ALTITUDE                                                           ");
  blk("                                                                ");
  blk("PUBLICATIONS:                                                   ");
  blk("------------------------------------                            ");
  blk("  <PREFIX>_LAT, <PREFIX>_LON, <PREFIX>_X, <PREFIX>_Y           ");
  blk("  <PREFIX>_YAW       // FC attitude yaw, degrees clockwise from true north ");
  blk("  <PREFIX>_HEADING   // FC attitude yaw converted to local-grid bearing, degrees ");
  blk("  <PREFIX>_COG       // FC GPS course over ground converted to local grid, degrees ");
  blk("  <PREFIX>_SPEED     // FC GPS ground speed, m/s                  ");
  blk("  <PREFIX>_ALTITUDE, <PREFIX>_DEPTH                            ");
  blk("  UAV_HEALTH_AVAILABLE       // 1 after MAVSDK health telemetry is received          ");
  blk("  UAV_IS_ARMED               // Current armed state reported by the flight controller ");
  blk("  UAV_HEALTH_GYRO            // 1 when healthy                                        ");
  blk("  UAV_HEALTH_ACCEL           // 1 when healthy                                        ");
  blk("  UAV_HEALTH_MAG             // 1 when healthy                                        ");
  blk("  UAV_HEALTH_LOCAL_POSITION  // MAVSDK health flag; not proof of GPS fix              ");
  blk("  UAV_HEALTH_GLOBAL_POSITION // MAVSDK health flag; check UAV_GPS_* too                ");
  blk("  UAV_HEALTH_HOME_POSITION   // 1 when healthy                                        ");
  blk("  UAV_IS_ARMABLE             // pre-arm status reported by MAVSDK                    ");
  blk("  UAV_HEALTH_ALL_OK          // MAVSDK aggregate health                               ");
  blk("  UAV_HEALTH_AGE             // Seconds since the last detailed health sample         ");
  blk("  UAV_GUIDED_MODE_ACTIVE     // 1 when flight-mode telemetry confirms Guided           ");
  blk("  UAV_SITL_SPEEDUP           // Last successfully commanded SITL speed multiplier       ");
  blk("  UAV_BATTERY_SOC            // Estimated remaining charge, percent                    ");
  blk("  UAV_BATTERY_VOLTAGE        // Primary battery voltage, V                             ");
  blk("  UAV_BATTERY_DATA_VALID     // 1 when remaining charge is finite and no more than 3 s old ");
  blk("  UAV_BATTERY_DATA_AGE       // Seconds since the last MAVSDK battery sample            ");
  blk("  UAV_ARM_POLICY_READY       // 1 when bridge policy permits a new ARM submission      ");
  blk("  UAV_ARM_POLICY_REASON      // Stable reason token, e.g. READY or HEALTH_STALE        ");
  blk("  UAV_DISARM_POLICY_READY    // 1 when bridge policy permits a new DISARM submission   ");
  blk("  UAV_DISARM_POLICY_REASON   // Stable reason token, e.g. READY or NOT_ON_GROUND       ");
  blk("  UAV_LAND_POLICY_READY      // 1 when mode policy permits a new LAND submission       ");
  blk("  UAV_LAND_POLICY_REASON     // Stable reason token; all unlisted modes are denied      ");
  blk("  UAV_COMMAND_RESULT         // Every ARDU_COMMAND reports an outcome; takeoff adds COMPLETED ");
  blk("  UAV_BRAKE_HOLD_PHASE, UAV_BRAKE_HOLD_REASON // Route BRAKE_AND_HOLD progress/blockers ");
  blk("  UAV_BRAKE_HOLD_SPEED, UAV_BRAKE_HOLD_MOTION_AGE // Native horizontal m/s, real receive s ");
  blk("  UAV_BRAKE_HOLD_SAMPLES, UAV_BRAKE_HOLD_ID // Distinct stopped FC frames, transaction ID ");
  blk("  UAV_ROLL, UAV_PITCH         // Native snapshot degrees, right bank / nose up positive ");
  blk("  UAV_ATTITUDE_AVAILABLE     // 1 after a finite native Euler sample; check AGE too ");
  blk("  UAV_ATTITUDE_AGE           // Native receipt age, real seconds; -1 if unavailable ");
  blk("  UAV_ATTITUDE_SAMPLE_TIME   // Original native receipt time, MOOS seconds (not repost time) ");
  blk("  UAV_ATTITUDE_TIME_USEC     // Original FC attitude boot timestamp, microseconds ");
  blk("  UAV_ATTITUDE_SAMPLES       // Atomic boot_usec,roll,pitch,yaw,receipt_moos;... native window ");
  blk("  UAV_YAW_ALIGN_RESULT       // id#heading#status#detail; ACCEPTED means FC ACK, not aligned ");
  blk("  UAV_GPS_AVAILABLE          // 1 after GPS telemetry is received; fix may be invalid ");
  blk("  UAV_GPS_FIX_TYPE           // 0=No GPS,1=No Fix,2=2D,3=3D,...6=RTK fixed          ");
  blk("  UAV_GPS_SATELLITES         // Satellites used/visible                               ");
  blk("  UAV_GPS_HDOP               // Horizontal dilution of precision                     ");
  blk("  UAV_GPS_VDOP               // Vertical dilution of precision                       ");
  blk("  UAV_GPS_AGE                // Seconds since the last GPS sample                     ");
  blk("  UAV_LANDED_STATE_AVAILABLE // 1 after flight-controller landed state is received    ");
  blk("  UAV_LANDED_STATE           // ON_GROUND, IN_AIR, TAKING_OFF, LANDING, or UNKNOWN    ");
  blk("  UAV_LANDED_STATE_AGE       // Seconds since the last landed-state sample             ");
  blk("  UAV_LANDING_TARGET_AVAILABLE // 1 when LANDING_TARGET is no more than 0.5 s old       ");
  blk("  UAV_LANDING_TARGET_AGE, UAV_LANDING_TARGET_TIME_USEC                                   ");
  blk("  UAV_LANDING_TARGET_SYSTEM_ID, UAV_LANDING_TARGET_COMPONENT_ID                         ");
  blk("  UAV_LANDING_TARGET_TARGET_NUM, UAV_LANDING_TARGET_FRAME, UAV_LANDING_TARGET_TYPE       ");
  blk("  UAV_LANDING_TARGET_POSITION_VALID, UAV_LANDING_TARGET_DISTANCE                         ");
  blk("  UAV_LANDING_TARGET_ANGLE_X, UAV_LANDING_TARGET_ANGLE_Y                                 ");
  blk("  UAV_LANDING_TARGET_SIZE_X, UAV_LANDING_TARGET_SIZE_Y                                   ");
  blk("  UAV_LANDING_TARGET_X, UAV_LANDING_TARGET_Y, UAV_LANDING_TARGET_Z                       ");
  blk("  UAV_LANDING_TARGET_Q_W, UAV_LANDING_TARGET_Q_X                                         ");
  blk("  UAV_LANDING_TARGET_Q_Y, UAV_LANDING_TARGET_Q_Z                                         ");
  blk("  PARDUBRIDGE_PASSIVE_STATE     // WAITING_FOR_SYSTEM, RECEIVING, etc.             ");
  blk("  PARDUBRIDGE_PASSIVE_CONNECTED // 1 while the discovered MAVLink system is connected ");
  blk("  AUTOPILOT_MODE             // pArduBridge helm state, not raw FC mode ");
  blk("  CONTROL_AUTHORITY          // FC/manual or MOOS Helm ownership summary ");
  blk("  UAV_GUIDED_OWNERSHIP_STATE // GROUNDED_DISARMED, FC_TAKEOFF_ACKNOWLEDGED, ");
  blk("                             // MISSING_OWNER, TELEMETRY_UNCERTAIN, NOT_APPLICABLE ");
  blk("  UAV_GUIDED_OWNERSHIP_WARNING // 1 after unowned/uncertain grace of 2 MOOS s ");
  blk("  HELM_SETPOINT_FRESH        // 1 when all Helm setpoints are within timeout ");
  blk("  HELM_SETPOINT_AGE          // Age of oldest Helm setpoint; -1 until complete ");
  blk("  HELM_SETPOINT_TIMEOUT_LATCHED // 1 until a fresh confirmed route handoff ");
  blk("  MOOS_MANUAL_OVERRIDE, RETURN_UPDATE, TOWAYPT_UPDATE          ");
  blk("  SURVEY_UPDATE, CONST_ALTITUDE_UPDATE, VIEW_*                  ");
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
