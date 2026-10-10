/************************************************************/
/*    NAME: Steve Carter Feujo Nomeny                       */
/*    ORGN: NTNU, MIT                                       */
/*    FILE: ArduBridge.h                                    */
/*    DATE: September 9th, 2024                             */
/************************************************************/

#ifndef ArduBridge_HEADER
#define ArduBridge_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include "MOOSGeodesy.h"
#include <mavsdk/cli_arg.h>

#include "WarningSystem.h"
#include "UAV_Model.h"
#include "SetpointManager.h"
#include "GuidedOwnershipMonitor.h"

#include <map>
#include <functional>
#include <optional>
#include <utility> // for std::pair

#include <future>
#include <atomic>
#include <deque>
#include <mutex>
#include <set>

class ArduBridge : public AppCastingMOOSApp
{
public:
  ArduBridge();
  ~ArduBridge();

protected: // Standard MOOSApp functions to overload
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();

protected: // Standard AppCastingMOOSApp function to overload
  bool buildReport();

protected:
  void registerVariables();

  // Notify to DB
  void postTelemetryUpdate(const std::string &prefix);
  void postHealthUpdate();
  void postLandingTargetUpdate();
  void postCommandResult();
  void queueCommandResult(const UAV_Model::CommandResult &result);

  void postSpeedUpdateToBehaviors(double speed);
  // Send command to UAV
  void sendDesiredValuesToUAV(UAV_Model &uav, bool forceSend = false);
  void handleHelmSetpointTimeout();
  void postHelmSetpointStatus();
  void pollGuidedHandoffConfirmation();
  void pollBrakeAndHold();
  double m_last_brake_post_time = -1;
  std::string m_last_brake_post_state;

private: // Configuration variables
  std::string m_uav_prefix;

  static inline std::map<mavsdk::CliArg::Protocol, std::string> protocol2str = {
      {mavsdk::CliArg::Protocol::None, "None"},
      {mavsdk::CliArg::Protocol::Udp, "Udp"},
      {mavsdk::CliArg::Protocol::Tcp, "Tcp"},
      {mavsdk::CliArg::Protocol::Serial, "Serial"}};
  mavsdk::CliArg m_cli_arg;

  bool m_geo_ok;
  // MOOS-IvP apps built with USE_UTM (the default) use local UTM; match
  // them with geodesy=utm so NAV_X/Y agree with pNodeReporter/pMarineViewer.
  bool m_geo_utm = false;
  CMOOSGeodesy m_geodesy;

private: // Autopilot Helm states
  enum class AutopilotHelmMode
  {
    HELM_UNKOWN,
    HELM_PARKED, // Helm is parked

    // Inactive states where the Helm is not in control (MOOS_MANUAL_OVERRIDE = true) but still in Drive
    HELM_INACTIVE,
    HELM_INACTIVE_LOITERING,
    HELM_INACTIVE_BRAKING,

    // Active states where the Helm is in control
    HELM_ACTIVE, // Helm has nothing to do

    // Helm is commanding
    HELM_TOWAYPT,
    HELM_RETURNING,
    HELM_SURVEYING,
    HELM_VORONOI,

  };

  // map containing the stateName
  const std::vector<std::pair<AutopilotHelmMode, std::string>> stateStringPairs = {
      {AutopilotHelmMode::HELM_PARKED, "HELM_PARKED"},
      {AutopilotHelmMode::HELM_INACTIVE, "HELM_INACTIVE"},
      {AutopilotHelmMode::HELM_INACTIVE_LOITERING, "HELM_INACTIVE_LOITERING"},
      {AutopilotHelmMode::HELM_INACTIVE_BRAKING, "HELM_INACTIVE_BRAKING"},
      {AutopilotHelmMode::HELM_ACTIVE, "HELM_ACTIVE"},
      {AutopilotHelmMode::HELM_TOWAYPT, "HELM_TOWAYPT"},
      {AutopilotHelmMode::HELM_RETURNING, "HELM_RETURNING"},
      {AutopilotHelmMode::HELM_SURVEYING, "HELM_SURVEYING"},
      {AutopilotHelmMode::HELM_VORONOI, "HELM_VORONOI"},
      {AutopilotHelmMode::HELM_UNKOWN, "HELM_UNKOWN"},
  };

  std::string helmModeToString(AutopilotHelmMode state) const
  {
    for (const auto &[stateEnum, stateStr] : stateStringPairs)
    {
      if (stateEnum == state)
      {
        return stateStr;
      }
    }
    return "HELM_UNKOWN";
  }

  std::string controlAuthorityToString(AutopilotHelmMode state) const;

  AutopilotHelmMode stringToHelmMode(const std::string &stateStr) const
  {
    for (const auto &[stateEnum, stateStrVal] : stateStringPairs)
    {
      if (stateStrVal == stateStr)
      {
        return stateEnum;
      }
    }
    return AutopilotHelmMode::HELM_UNKOWN;
  }

  using StateTransition = std::pair<AutopilotHelmMode, AutopilotHelmMode>;
  std::map<StateTransition, std::function<void()>> m_statetransition_functions;
  void initializeStateTransitionFunctions();
  void updateControlAuthorityMonitoring();
  void handleYawAlignmentRequest(const CMOOSMsg &msg);
  void cancelEnterGuided(const std::string &detail);

private:
  const double MARKER_WIDTH = 10.0;
  const double COURSE_POINT_SIZE = 5;

  bool visualizeHomeLocation();
  void visualizeLoiterLocation(const XYPoint &loiter_coord, bool visualize = true);
  void visualizeCourseWaypoint(const XYPoint &course_coord, bool visualize = true);
  void visualizeCourseVector(double x, double y, double magnitude, double angle, bool visualize = true);

  void visualizeCourseHoldTarget(bool visualize = true);

  // bool evaluateBoolFromString(const std::string& str) const { bool b; setBooleanOnString(b,str); return b;}
  bool parseCoordinateString(const std::string &input, double &lat, double &lon, double &x, double &y, std::string &vname) const;

  // AutopilotHelmMode getTransitionAutopilotHelmState() const;

private: // Helperfunctions
  std::string generateMissionPathSpec(const std::vector<XYPoint> &points) const;

  std::string xypointToString(const XYPoint &point) const;
  bool transformXYToLatLon(double x, double y, double &latitude,
                           double &longitude);
  XYPoint transformLatLonToXY(const XYPoint &lat_lon);
  // Local-frame bearing of true north at a position; non-zero only for UTM,
  // whose grid north differs from true north by the convergence angle.
  double gridBearingOfTrueNorth(double lat, double lon);
  bool isHelmOn() const { return (static_cast<int>(m_autopilot_mode) >= static_cast<int>(AutopilotHelmMode::HELM_ACTIVE)); };
  bool isHelmCommanding() const { return (static_cast<int>(m_autopilot_mode) > static_cast<int>(AutopilotHelmMode::HELM_ACTIVE)); };
  bool isHelmOn_NothingTodo() const { return (m_autopilot_mode == AutopilotHelmMode::HELM_ACTIVE); };
  // bool isHelmOn_Busy() const { return (isHelmOn() && !isHelmOn_NothingTodo()); };

  void goToHelmMode(AutopilotHelmMode state, bool fromGCS = false);

  bool tryDoTakeoff();
  bool tryFlyToWaypoint();
  bool tryRTL(); // Non-blocking
  bool tryloiterAtPos(const XYPoint &loiter_coord = XYPoint(0, 0), bool holdCurrentAltitude = false);

protected: // async
  template <typename T>
  class PromiseFuture
  {
  public:
    std::promise<T> prom;
    std::future<T> fut = prom.get_future();

    void reset()
    {
      prom = std::promise<T>();
      fut = prom.get_future();
    }
  };

  struct ResultPair
  {
    bool success;
    std::string message;
    double display_time = WARNING_DURATION;
  };

  /// @brief Future and promise for async functions
  template <typename T>
  std::optional<T> future_poll_result(std::future<T> &future)
  {
    if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
    {
      return future.get(); // Retrieve the value if it's ready
    }
    return std::nullopt; // Return an empty optional if not ready
  }

  void flyToWaypoint_async();
  PromiseFuture<ResultPair> m_fly_to_waypoint_promfut;

  void loiterAtPos_async(const XYPoint &loiter_coord = XYPoint(0, 0),
                         bool holdCurrentAltitude = false,
                         const std::string &command_name = "LOITER");
  PromiseFuture<ResultPair> m_loiter_at_pos_promfut;

  void rtl_async(bool force_native_rtl = false);
  PromiseFuture<ResultPair> m_rtl_promfut;

  void autoland_async();
  PromiseFuture<ResultPair> m_autoland_promfut;

  void precisionLoiter_async(bool enable);
  PromiseFuture<ResultPair> m_precision_loiter_promfut;

  void flightControllerLoiter_async();
  PromiseFuture<ResultPair> m_fc_loiter_promfut;

private: // State variables
  // For UAV

  std::string m_vname;
  std::string m_vcolor;
  bool m_is_simulation;
  bool m_passive;
  bool m_command_groundSpeed;
  bool m_precision_loiter_enter_loiter;
  bool m_show_loiter_visuals;
  double m_helm_setpoint_timeout;
  double m_last_health_post_time;
  double m_last_helm_setpoint_status_post_time;
  std::deque<UAV_Model::CommandResult> m_command_results;
  std::mutex m_command_results_mutex;
  std::string m_last_command_result;

  GuidedOwnershipMonitor m_guided_ownership_monitor;
  double m_last_guided_hold_refresh_time;
  std::atomic<bool> m_helm_setpoint_timeout_detected;
  std::atomic<bool> m_helm_setpoint_timeout_latched;
  std::atomic<bool> m_guided_handoff_waiting;
  ModeConfirmationTracker m_guided_handoff_tracker;
  std::mutex m_guided_handoff_mutex;

  std::shared_ptr<WarningSystem> m_warning_system_ptr;
  UAV_Model m_uav_model;
  ThreadSafeVariable<SetpointManager> m_helm_desiredValues;

  std::pair<bool, double> m_do_change_speed_pair;
  std::pair<bool, double> m_do_change_course_pair;
  std::pair<bool, double> m_do_change_altitude_pair;
  bool m_do_reset_speed;
  bool m_do_return_to_launch;
  bool m_force_native_rtl;
  bool m_do_autoland;
  std::optional<bool> m_arm_request;
  std::optional<double> m_sitl_speedup_request;
  bool m_enter_guided_pending;
  double m_enter_guided_started;
  double m_enter_guided_timeout;
  double m_last_enter_guided_attempt;
  uint64_t m_enter_guided_command_id;
  bool m_do_takeoff;
  bool m_do_fly_to_waypoint;
  bool m_do_helm_survey;
  bool m_do_helm_voronoi;

  std::pair<bool, std::string> m_do_loiter_pair;
  std::string m_loiter_command_name;
  bool m_loiter_hold_current_altitude;
  std::pair<bool, bool> m_do_precision_loiter_pair;
  bool m_do_fc_loiter;

  AutopilotHelmMode m_autopilot_mode;

  // Helm utility
  XYPoint m_tonext_waypointXY;
  std::vector<XYPoint> m_waypointsXY_mission;

  // Declared after the model: async command joins before model destruction.
  std::future<bool> m_yaw_future;
  std::set<std::string> m_seen_yaw_requests;
  std::string m_yaw_request_id, m_last_yaw_result;
  double m_yaw_goal = 0, m_yaw_bridge_started = 0;
};

#endif
