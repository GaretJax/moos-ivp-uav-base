#include "UAV_Model.h"

bool UAV_Model::verifyRallyIntParameter(const std::string &name, int value,
                                       std::string &reason)
{
  const auto grounded = [this]() {
    return m_system_ptr && m_system_ptr->is_connected() && m_param_ptr &&
           !isArmed() && hasLandedStateTelemetry() &&
           getLandedStateTelemetryAge() <= LANDED_STATE_TELEMETRY_MAX_AGE_S &&
           getLandedState() == mavsdk::Telemetry::LandedState::OnGround;
  };
  if (!grounded()) {
    reason = "rally_parameter_requires_fresh_grounded_disarmed:" + name;
    return false;
  }
  // GET requests go to the FC, not the parameter cache. Defaults/profile
  // values may already satisfy the policy; do not rewrite them gratuitously.
  const auto before = m_param_ptr->get_param_int(name);
  if (before.first != mavsdk::Param::Result::Success) {
    std::ostringstream detail;
    detail << "rally_parameter_read_failed:" << name << ":" << before.first;
    reason = detail.str();
    return false;
  }
  if (!grounded()) {
    reason = "vehicle_changed_during_rally_parameter_read:" + name;
    return false;
  }
  if (before.second == value)
    return true;

  const auto write = m_param_ptr->set_param_int(name, value);
  // ArduPilot can echo an integer SET as REAL32. MAVSDK's strict variant
  // comparison then exhausts retries and calls it Timeout. A new typed GET,
  // not that echo or a timeout alone, is the authority for the actual value.
  const auto after = m_param_ptr->get_param_int(name);
  if (!grounded() ||
      (write != mavsdk::Param::Result::Success && write != mavsdk::Param::Result::Timeout) ||
      after.first != mavsdk::Param::Result::Success || after.second != value) {
    std::ostringstream detail;
    detail << "rally_parameter_readback_failed:" << name << ":write=" << write
           << ":read=" << after.first << ":actual=" << after.second
           << ":expected=" << value;
    reason = detail.str();
    return false;
  }
  if (write == mavsdk::Param::Result::Timeout)
    reportEventFromCallback("Rally parameter " + name +
                            ": SET echo timed out; independent FC readback verified");
  return true;
}

bool UAV_Model::readRallyPoint(mavsdk::MissionRaw::MissionItem &point)
{
  // Installed MAVSDK supports typed rally upload, but not rally download.
  // Request the RALLY mission list explicitly; never download the flight plan.
  // Shared storage remains alive if a callback races with unsubscription.
  struct Readback {
    std::mutex mutex;
    std::condition_variable ready;
    bool have_count = false, have_point = false;
    uint16_t count = 0;
    mavlink_mission_item_int_t point{};
  };
  const auto state = std::make_shared<Readback>();
  const uint8_t system = m_mavPass_ptr->get_target_sysid();
  const uint8_t component = m_mavPass_ptr->get_target_compid();
  const auto count_handle = m_mavPass_ptr->subscribe_message(MAVLINK_MSG_ID_MISSION_COUNT,
      [state, system, component](const mavlink_message_t &message) {
        mavlink_mission_count_t count{};
        mavlink_msg_mission_count_decode(&message, &count);
        if (message.sysid != system || message.compid != component ||
            count.mission_type != MAV_MISSION_TYPE_RALLY) return;
        std::lock_guard<std::mutex> lock(state->mutex);
        state->count = count.count;
        state->have_count = true;
        state->ready.notify_all();
      });
  const auto item_handle = m_mavPass_ptr->subscribe_message(MAVLINK_MSG_ID_MISSION_ITEM_INT,
      [state, system, component](const mavlink_message_t &message) {
        mavlink_mission_item_int_t item{};
        mavlink_msg_mission_item_int_decode(&message, &item);
        if (message.sysid != system || message.compid != component ||
            item.mission_type != MAV_MISSION_TYPE_RALLY || item.seq != 0) return;
        std::lock_guard<std::mutex> lock(state->mutex);
        state->point = item;
        state->have_point = true;
        state->ready.notify_all();
      });
  bool success = false;
  for (int attempt = 0; attempt < 4 && m_running; ++attempt) {
    bool request_item;
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->have_count && state->count != 1) break;
      if (state->have_count && state->have_point) { success = true; break; }
      request_item = state->have_count;
    }
    const auto sent = m_mavPass_ptr->queue_message([=](MavlinkAddress own, uint8_t channel) {
      mavlink_message_t message{};
      if (request_item)
        mavlink_msg_mission_request_int_pack_chan(own.system_id, own.component_id, channel,
          &message, system, component, 0, MAV_MISSION_TYPE_RALLY);
      else
        mavlink_msg_mission_request_list_pack_chan(own.system_id, own.component_id, channel,
          &message, system, component, MAV_MISSION_TYPE_RALLY);
      return message;
    });
    if (sent != mavsdk::MavlinkPassthrough::Result::Success) break;
    std::unique_lock<std::mutex> lock(state->mutex);
    state->ready.wait_for(lock, std::chrono::seconds(2), [&] {
      return request_item ? state->have_point : state->have_count;
    });
    if (state->have_count && state->count == 1 && state->have_point) success = true;
    if (success) break;
  }
  m_mavPass_ptr->unsubscribe_message(MAVLINK_MSG_ID_MISSION_COUNT, count_handle);
  m_mavPass_ptr->unsubscribe_message(MAVLINK_MSG_ID_MISSION_ITEM_INT, item_handle);
  if (!success) return false;
  const auto received = state->point;
  point.command = received.command; point.frame = received.frame;
  point.x = received.x; point.y = received.y; point.z = received.z;
  const auto acknowledged = m_mavPass_ptr->queue_message([=](MavlinkAddress own, uint8_t channel) {
    mavlink_message_t message{};
    mavlink_msg_mission_ack_pack_chan(own.system_id, own.component_id, channel,
      &message, system, component, MAV_MISSION_ACCEPTED, MAV_MISSION_TYPE_RALLY, 0);
    return message;
  });
  return acknowledged == mavsdk::MavlinkPassthrough::Result::Success;
}
