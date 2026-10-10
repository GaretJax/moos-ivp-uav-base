#include "../GuidedOwnershipMonitor.h"
#include "../TakeoffConfirmationTracker.h"

#include <cassert>
#include <chrono>
#include <iostream>
#include <limits>
#include <string>

using Monitor = GuidedOwnershipMonitor;
using State = Monitor::State;

Monitor::Inputs grounded()
{
  Monitor::Inputs in;
  in.copter = in.guided = in.bridge_parked = in.connected = true;
  in.landed_fresh = in.on_ground = true;
  return in;
}

void classifications()
{
  auto in = grounded();
  assert(Monitor::classify(in) == State::GroundedDisarmed);
  in.armed = true;
  assert(Monitor::classify(in) == State::MissingOwner); // intent is not ACK
  in.takeoff_ack_active = true;
  assert(Monitor::classify(in) == State::TakeoffOwned);
  in.on_ground = false;
  in.taking_off_or_in_air = true;
  assert(Monitor::classify(in) == State::TakeoffOwned);
  in.takeoff_ack_active = false;
  assert(Monitor::classify(in) == State::MissingOwner); // airborne, no owner
  in.takeoff_ack_active = true;
  in.landed_fresh = false;
  assert(Monitor::classify(in) == State::TelemetryUncertain);
  in = grounded();
  in.landed_fresh = false;
  assert(Monitor::classify(in) == State::TelemetryUncertain); // stale ground
  in = grounded();
  in.connected = false;
  assert(Monitor::classify(in) == State::TelemetryUncertain);
  in = grounded();
  in.armed = in.takeoff_ack_active = true;
  in.on_ground = false; // LANDING/unknown is not a takeoff owner
  assert(Monitor::classify(in) == State::MissingOwner);
  in.taking_off_or_in_air = true;
  in.armed = false; // even a live ACK cannot own disarmed airborne flight
  assert(Monitor::classify(in) == State::MissingOwner);
  in.bridge_parked = false; // actual Helm DRIVE alone does not enter this path
  assert(Monitor::classify(in) == State::NotApplicable);
  in = grounded();
  in.guided = false; // LAND, RTL, etc. have separate monitoring
  assert(Monitor::classify(in) == State::NotApplicable);
  in = grounded();
  in.copter = false;
  assert(Monitor::classify(in) == State::NotApplicable);
}

void timing()
{
  Monitor monitor;
  auto in = grounded();
  monitor.update(in, 10);
  monitor.update(in, 100);
  assert(!monitor.warningDue(100)); // long disarmed preflight is informational
  in.armed = true;
  monitor.update(in, 101);
  assert(!monitor.warningDue(103));
  monitor.update(in, 103.01);
  assert(monitor.warningDue(103.01));
  in.takeoff_ack_active = true;
  monitor.update(in, 104);
  assert(!monitor.warningDue(104)); // old warning retracts on a real lease
  in.on_ground = false;
  in.taking_off_or_in_air = true;
  monitor.update(in, 130);
  assert(!monitor.warningDue(130));
  in.takeoff_ack_active = false; // completion/abort/timeout revoke ownership
  monitor.update(in, 131);
  assert(!monitor.warningDue(131));
  monitor.update(in, 133.01);
  assert(monitor.warningDue(133.01));
  in.landed_fresh = false;
  monitor.update(in, 134);
  assert(monitor.state() == State::TelemetryUncertain);
  assert(monitor.warningDue(134)); // uncertainty must not restart grace
  in.bridge_parked = false;
  monitor.update(in, 135);
  assert(!monitor.warningDue(135));
  in.bridge_parked = true;
  monitor.update(in, 136);
  assert(!monitor.warningDue(136));
  monitor.update(in, 139);
  assert(monitor.warningDue(139));
  monitor.update(in, 10); // MOOS time moved backwards
  assert(!monitor.warningDue(10));
  monitor.reset();
  assert(!monitor.warningDue(100));
  assert(monitor.state() == State::NotApplicable);
  monitor.update(in, std::numeric_limits<double>::quiet_NaN());
  assert(!monitor.warningDue(100));
}

void leases()
{
  using Tracker = TakeoffConfirmationTracker;
  const auto start = Tracker::Clock::time_point{};
  const auto at = [&](int seconds) { return start + std::chrono::seconds(seconds); };
  Tracker tracker;
  assert(!tracker.hasActiveAcknowledgement(start));
  tracker.accept(1, 8, true, start);
  assert(tracker.hasActiveAcknowledgement(start));
  assert(!tracker.hasActiveAcknowledgement(at(-1)));
  assert(tracker.hasActiveAcknowledgement(at(4)));
  assert(!tracker.hasActiveAcknowledgement(at(5))); // no polling needed to expire
  auto outcome = tracker.evaluate(false, false, false, at(5));
  assert(outcome.timeout_phase == Tracker::TimeoutPhase::Activation);
  assert(!tracker.hasActiveAcknowledgement(at(6)));

  tracker.accept(2, 8, true, start);
  outcome = tracker.evaluate(true, false, false, at(1));
  assert(outcome.confirmed);
  assert(tracker.hasActiveAcknowledgement(at(59)));
  assert(!tracker.hasActiveAcknowledgement(at(60)));
  outcome = tracker.evaluate(true, false, false, at(60));
  assert(outcome.timeout_phase == Tracker::TimeoutPhase::Completion);
  assert(!tracker.hasPending());

  tracker.accept(3, 8, true, start);
  outcome = tracker.evaluate(true, true, false, at(1));
  assert(outcome.completed);
  assert(!tracker.hasActiveAcknowledgement(at(1)));

  tracker.accept(4, 8, true, start);
  outcome = tracker.evaluate(false, false, true, at(1));
  assert(outcome.aborted);
  assert(!tracker.hasActiveAcknowledgement(at(1)));

  Monitor monitor;
  auto in = grounded();
  in.armed = true;
  in.on_ground = false;
  in.taking_off_or_in_air = true;
  tracker.accept(5, 8, true, start);
  tracker.evaluate(true, false, false, at(1));
  in.takeoff_ack_active = tracker.hasActiveAcknowledgement(at(59));
  monitor.update(in, 59);
  assert(monitor.state() == State::TakeoffOwned);
  in.takeoff_ack_active = tracker.hasActiveAcknowledgement(at(60));
  monitor.update(in, 60);
  assert(monitor.state() == State::MissingOwner);
  monitor.update(in, 62.01);
  assert(monitor.warningDue(62.01)); // airborne warnings cannot be masked forever
}

int main(int argc, char **argv)
{
  assert(argc == 2);
  const std::string test = argv[1];
  if (test == "classification")
    classifications();
  else if (test == "timing")
    timing();
  else if (test == "leases")
    leases();
  else
    return 1;
  std::cout << test << ": OK\n";
}
