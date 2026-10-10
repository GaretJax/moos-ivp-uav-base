#pragma once

#include <cmath>

// Diagnostic only. Never requests a mode, hands control to Helm, or proves
// arrival/readiness. Bridge authority is distinct from pHelmIvP PARK/DRIVE.
class GuidedOwnershipMonitor
{
public:
  enum class State
  {
    NotApplicable,
    GroundedDisarmed,
    TakeoffOwned,
    MissingOwner,
    TelemetryUncertain
  };

  struct Inputs
  {
    bool copter = false;
    bool guided = false;
    bool bridge_parked = false;
    bool connected = false;
    bool armed = false;
    bool landed_fresh = false;
    bool on_ground = false;
    bool taking_off_or_in_air = false;
    // Actual FC command ACK with a still-live activation/completion deadline.
    // A request, FSM intent, rejected command, or historical ACK is not enough.
    bool takeoff_ack_active = false;
  };

  static State classify(const Inputs &in)
  {
    if (!in.copter || !in.guided || !in.bridge_parked)
      return State::NotApplicable;
    if (!in.connected || !in.landed_fresh)
      return State::TelemetryUncertain;
    if (!in.armed && in.on_ground)
      return State::GroundedDisarmed;
    if (in.armed && in.takeoff_ack_active && (in.on_ground || in.taking_off_or_in_air))
      return State::TakeoffOwned;
    return State::MissingOwner;
  }

  void update(const Inputs &in, double now)
  {
    const State next = classify(in);
    const bool caution = next == State::MissingOwner || next == State::TelemetryUncertain;
    if (!caution || !std::isfinite(now))
      m_warning_since = -1;
    else if (m_warning_since < 0 || now < m_warning_since)
      m_warning_since = now;
    m_state = next;
  }

  void reset()
  {
    m_state = State::NotApplicable;
    m_warning_since = -1;
  }

  bool warningDue(double now) const
  {
    return m_warning_since >= 0 && std::isfinite(now) && now - m_warning_since > 2.0;
  }

  State state() const
  {
    return m_state;
  }

  static const char *stateName(State state)
  {
    switch (state) {
    case State::NotApplicable:
      return "NOT_APPLICABLE";
    case State::GroundedDisarmed:
      return "GROUNDED_DISARMED";
    case State::TakeoffOwned:
      return "FC_TAKEOFF_ACKNOWLEDGED";
    case State::MissingOwner:
      return "MISSING_OWNER";
    case State::TelemetryUncertain:
      return "TELEMETRY_UNCERTAIN";
    }
    return "UNKNOWN";
  }

private:
  State m_state = State::NotApplicable;
  double m_warning_since = -1;
};
