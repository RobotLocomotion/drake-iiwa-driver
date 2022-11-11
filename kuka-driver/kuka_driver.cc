#include <poll.h>
#include <sched.h>
#include <stdlib.h>
#include <sys/mman.h>

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>

#include "friClientApplication.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"

#include "drake/common/drake_assert.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_iiwa_status_telemetry.hpp"

#include "low_pass_filter.h"


using drake::lcmt_iiwa_command;
using drake::lcmt_iiwa_status;
using drake::lcmt_iiwa_status_telemetry;

namespace {

const int kNumJoints = 7;
const int kDefaultPort = 30200;
const char* kLcmStatusChannel = "IIWA_STATUS";
const char* kLcmStatusTelemetryChannel = "IIWA_STATUS_TELEMETRY";
const char* kLcmCommandChannel = "IIWA_COMMAND";
const double kJointLimitSafetyMarginDegree = 1;
const double kJointTorqueSafetyMarginNm = 60;
const double kJointTorqueSafetyMarginScale[kNumJoints] = {
    1, 1, 1, 0.5, 0.2, 0.2, 0.1};
const double kTorqueOnlyKp[kNumJoints] = {
    1000, 1000, 1000, 500, 500, 500, 500};
const double kTorqueOnlyKd[kNumJoints] = {
    50, 50, 50, 50, 35, 35, 35};

double ToRadians(double degrees) {
  return degrees * M_PI / 180.;
}

void PrintVector(const std::vector<double>& array, int start, int length,
                 std::ostream& out) {
  const int end = std::min(start + length, static_cast<int>(array.size()));
  for (int i = start; i < end; i++) {
    out << array.at(i);
    if (i != end - 1)
      out << ", ";
    else
      out << "\n";
  }
}

int64_t micros() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

}  // namespace

DEFINE_double(ext_trq_limit, kJointTorqueSafetyMarginNm,
              "Maximal external torque that triggers safety freeze");
DEFINE_string(joint_ext_trq_limit, "", "Specify the maximum external torque "
              "that triggers safety freeze on a per-joint basis.  "
              "This is a comma separated list of numbers e.g. "
              "100,100,53.7,30,30,28.5,10.  Overrides ext_trq_limit.");
DEFINE_int32(fri_port, kDefaultPort, "First UDP port for FRI messages");
DEFINE_string(lcm_url, "", "LCM URL for Kuka driver");
DEFINE_int32(num_robots, 1, "Number of robots to control");
DEFINE_string(lcm_command_channel, kLcmCommandChannel,
              "Channel to receive LCM command messages on");
DEFINE_string(lcm_status_channel, kLcmStatusChannel,
              "Channel to send LCM status messages on");
DEFINE_string(lcm_status_telemetry_channel, kLcmStatusTelemetryChannel,
              "Channel to send LCM status telemetry messages on");
DEFINE_bool(realtime, false, "Use realtime priority");
DEFINE_bool(mlockall, false, "Prevent memory from being paged out. This is "
            "automatically enabled if realtime prority is enabled.");
DEFINE_bool(sched_fifo, true, "Use FIFO realtime scheduling. This assumes that "
            "this driver is the most important realtime-sensitive task running "
            "and should be provided as much scheduler time as it needs.");
DEFINE_int32(priority, 90, "Priority for realtime scheduling");
DEFINE_bool(restart_fri, false,
            "Restart robot motion after the FRI signal has degraded and "
            "been restored.");
DEFINE_double(start_command_guard, 0.250, "Duration after publishing the "
              "first status message for which the driver must receive no "
              "command messages.  This is intended to guard against "
              "controllers which were accidentally left running after a "
              "previous invocation of this driver.");
DEFINE_double(time_step, 0.005, "Desired time step.");

DEFINE_bool(
    torque_only, false, "Send torques only; should set --time_step=0.001");
DEFINE_double(
    torque_only_kp_scale, 1.0,
    "Scaling for position gains when inactive.");
DEFINE_double(
    torque_only_kd_scale, 1.0,
    "Scaling for velocity gains when inactive.");
DEFINE_double(
    command_expire, 0.05,
    "Time since last receipt of message to indicate expiration. This will "
    "make the driver inhibit motion. At present, only used for "
    "--torque_only=true.");

namespace kuka_driver {

class TimeSyncFilter {
 public:
  /**
   * Constructor for a naive synchronizer for iiwa's and the host machine's
   * clock. This uses a exponential filter to estimates the time difference
   * between the clocks (host - iiwa), and uses it to offset the iiwa's
   * time stamp.
   * @param num_measurement Number of initial measurements to build a
   * reasonable estimate of the time difference.
   */
  explicit TimeSyncFilter(uint64_t num_measurement)
    : num_measurement_(num_measurement), filter_(0.99) {}

  /**
   * Returns the current estimated time difference: host - remote.
   */
  int64_t get_diff() const {
    return static_cast<int64_t>(filter_.get_filtered());
  }

  /**
   * Converts the remote time stamp to host. If number of measurements is
   * smaller than the specified amount to the constructor, return -1.
   */
  int64_t to_host(const int64_t remote) const {
    if (!is_init()) return -1;
    return get_diff() + remote;
  }

  /**
   * Adds a measurement given by `remote` and `host`.
   */
  void add_measurement(const int64_t remote, const int64_t host) {
    double dt_utime = static_cast<double>(host - remote);
    if (ctr_ == 0) {
      filter_.set_initial(dt_utime);
    }

    filter_.filter(dt_utime);

    if (!is_init()) {
      ctr_++;
    }
  }

 private:
  bool is_init() const {
    return ctr_ >= num_measurement_;
  }

  const uint64_t num_measurement_;
  uint64_t ctr_{0};
  DiscreteTimeLowPassFilter<double> filter_;
};

class KukaLCMClient  {
 public:
  KukaLCMClient(int num_robots, const std::string& lcm_url)
      : num_joints_(num_robots * kNumJoints), lcm_(lcm_url), sync_(200) {
    // Use -1 as a sentinal to indicate that no status has been
    // sent (or received from the robot).
    lcm_status_.utime = -1;
    lcm_status_.num_joints = num_joints_;
    lcm_status_.joint_position_measured.resize(num_joints_, 0);
    lcm_status_.joint_position_commanded.resize(num_joints_, 0);
    lcm_status_.joint_position_ipo.resize(num_joints_, 0);
    lcm_status_.joint_velocity_estimated.resize(num_joints_, 0);
    lcm_status_.joint_torque_measured.resize(num_joints_, 0);
    lcm_status_.joint_torque_commanded.resize(num_joints_, 0);
    lcm_status_.joint_torque_external.resize(num_joints_, 0);
    vel_filtered_.resize(num_joints_, 0);

    // Use -1 as a sentinal to indicate that no command has been
    // received.
    lcm_command_.utime = -1;

    // Set torque limits.
    if (FLAGS_joint_ext_trq_limit.empty()) {
      for (int i = 0; i < kNumJoints; ++i) {
        external_torque_limit_.push_back(
            FLAGS_ext_trq_limit * kJointTorqueSafetyMarginScale[i]);
      }
    } else {
      std::string remain = FLAGS_joint_ext_trq_limit;
      for (int i = 0; i < kNumJoints - 1; ++i) {
        const auto next_comma = remain.find(",");
        if (next_comma == std::string::npos) {
          throw std::runtime_error(
              "--joint_ext_trq_limit must contain 7 comma delimited values");
        }
        std::string next = remain.substr(0, next_comma);
        external_torque_limit_.push_back(std::stod(next));
        remain = remain.substr(next_comma + 1);
      }
      if (remain.empty() || remain.find(",") != std::string::npos) {
        throw std::runtime_error(
            "--joint_ext_trq_limit must contain 7 comma delimited values");
      }
      external_torque_limit_.push_back(std::stod(remain));
    }

    std::cerr << "Joint torque limits: ";
    PrintVector(external_torque_limit_, 0, kNumJoints, std::cerr);

    // Initialize filters.
    const double cutoff_hz = 40;
    vel_filters_.resize(
        num_joints_, DiscreteTimeLowPassFilter<double>(
            cutoff_hz, FLAGS_time_step));
    utime_last_.resize(num_robots, -1);

    lcm::Subscription* sub = lcm_.subscribe(FLAGS_lcm_command_channel,
        &KukaLCMClient::HandleCommandMessage, this);
    // Only pay attention to the latest command.
    sub->setQueueCapacity(1);
  }

  void UpdateRobotState(int robot_id, const KUKA::FRI::LBRState& state) {
    const int joint_offset = robot_id * kNumJoints;
    assert(joint_offset + kNumJoints <= num_joints_);

    // Current time stamp for this robot.
    const int64_t remote_utime =
        state.getTimestampSec() * 1e6 + state.getTimestampNanoSec() / 1e3;
    const int64_t host_utime = micros();

    // Get delta time for this robot.
    double robot_dt = 0.;
    if (utime_last_.at(robot_id) != -1) {
      robot_dt = (remote_utime - utime_last_.at(robot_id)) / 1e6;
      // Check timing
      if (std::abs(robot_dt - FLAGS_time_step) > 1e-3) {
        std::cout << "Warning: dt " << robot_dt
                  << ", FLAGS_time_step " << FLAGS_time_step << "\n";
      }
    }
    utime_last_.at(robot_id) = remote_utime;

    // The choice of robot id 0 for the timestamp is arbitrary.
    if (robot_id == 0) {
      // Estimate time difference between host and iiwa.
      sync_.add_measurement(remote_utime, host_utime);
      // Set the iiwa_status' time stamp using the estimated time difference.
      lcm_status_.utime = sync_.to_host(remote_utime);

      // Save telemetry information.
      lcm_status_telemetry_.host_utime = host_utime;
      lcm_status_telemetry_.iiwa_utime = remote_utime;
      lcm_status_telemetry_.estimated_dt_host_minus_iiwa = sync_.get_diff();
    }

    // Velocity filtering.
    if (robot_dt != 0.) {
      for (int i = 0; i < kNumJoints; i++) {
        const int index = joint_offset + i;
        const double q_diff = state.getMeasuredJointPosition()[i] -
                              lcm_status_.joint_position_measured[index];
        vel_filtered_[index] = vel_filters_[index].filter(q_diff / robot_dt);
        lcm_status_.joint_velocity_estimated[index] = vel_filtered_[index];
      }
    }

    // Set other joint states.
    std::memcpy(lcm_status_.joint_position_measured.data() + joint_offset,
                state.getMeasuredJointPosition(), kNumJoints * sizeof(double));
    std::memcpy(lcm_status_.joint_position_commanded.data() + joint_offset,
                state.getCommandedJointPosition(), kNumJoints * sizeof(double));

    // In Sunrise 1.13, getIpoJointPosition changed from returning
    // NULL if it wasn't available to throwing an exception.  Try to
    // avoid triggering the exception, but catch and ignore it if it
    // happens.  Initalize everything to NaN first in case we don't
    // make it.
    for (int i = joint_offset; i < joint_offset + kNumJoints; i++) {
      lcm_status_.joint_position_ipo[i] =
          std::numeric_limits<double>::quiet_NaN();
    }
    const KUKA::FRI::ESessionState session_state = state.getSessionState();
    if (session_state == KUKA::FRI::COMMANDING_WAIT ||
        session_state == KUKA::FRI::COMMANDING_ACTIVE) {
      try {
        if (state.getIpoJointPosition() != NULL) {
          std::memcpy(lcm_status_.joint_position_ipo.data() + joint_offset,
                      state.getIpoJointPosition(), kNumJoints * sizeof(double));
        }
      } catch (...) {
        // I (sam.creasey) would prefer to catch the specific
        // exception here, but it's not clear how to detect which
        // version of FRI you're compiling against to determine if
        // it's possible to include FRIException.h.
      }
    }

    std::memcpy(lcm_status_.joint_torque_measured.data() + joint_offset,
                state.getMeasuredTorque(), kNumJoints * sizeof(double));
    std::memcpy(lcm_status_.joint_torque_commanded.data() + joint_offset,
                state.getCommandedTorque(), kNumJoints * sizeof(double));
    std::memcpy(lcm_status_.joint_torque_external.data() + joint_offset,
                state.getExternalTorque(), kNumJoints * sizeof(double));
  }

  const double* GetVelocityFiltered() const {
    return vel_filtered_.data();
  }

  /// @returns true if robot @p robot_id is in a safe state. Currently only
  /// checks the external torques field.
  ///
  /// Note: Should only be called after UpdateRobotState.
  bool CheckSafety(int robot_id) const {
    const int joint_offset = robot_id * kNumJoints;
    assert(joint_offset + kNumJoints <= num_joints_);

    // Check external torque for each joint.
    for (int i = 0; i < kNumJoints; i++) {
      const double ext_torque =
          lcm_status_.joint_torque_external[joint_offset + i];
      if (std::fabs(ext_torque) > external_torque_limit_[i]) {
        return false;
      }
    }

    return true;
  }

  // TODO(eric.cousineau): Use YAML archive serialization.
  void PrintRobotState(int robot_id, std::ostream& out) const {
    const int joint_offset = robot_id * kNumJoints;
    assert(joint_offset + kNumJoints <= num_joints_);

    out << "Robot: " << robot_id << ", at time: " << utime_last_.at(robot_id)
        << "\n";
    out << "Position: ";
    PrintVector(lcm_status_.joint_position_measured,
                joint_offset, kNumJoints, out);
    out << "Velocity: ";
    PrintVector(lcm_status_.joint_velocity_estimated,
                joint_offset, kNumJoints, out);
    out << "Ext Torque: ";
    PrintVector(lcm_status_.joint_torque_external,
                joint_offset, kNumJoints, out);
    out << "Torque: ";
    PrintVector(lcm_status_.joint_torque_measured,
                joint_offset, kNumJoints, out);

    out << "Commanded position: ";
    PrintVector(lcm_status_.joint_position_commanded,
                joint_offset, kNumJoints, out);

    out << "Commanded torque: ";
    PrintVector(lcm_status_.joint_torque_commanded,
                joint_offset, kNumJoints, out);
  }

  /// @return true if valid command data was present, or false if no
  /// command is available (in which case the array is not modified).
  bool GetPositionCommand(int robot_id, double* pos) const {
    const int joint_offset = robot_id * kNumJoints;
    assert(joint_offset + kNumJoints <= num_joints_);

    if (lcm_command_.utime == -1) {
      return false;
    }

    assert(lcm_command_.num_joints >= num_joints_);
    memcpy(pos, lcm_command_.joint_position.data() + joint_offset,
           kNumJoints * sizeof(double));
    return true;
  }

  /// @return true if valid command data was present, or false if no
  /// command is available (in which case the array is not modified).
  bool GetTorqueCommand(int robot_id, double* torque) const {
    const int joint_offset = robot_id * kNumJoints;
    assert(joint_offset + kNumJoints <= num_joints_);

    if (lcm_command_.utime == -1) {
      return false;
    }

    if (lcm_command_.num_torques == 0) {
      return false;
    }

    assert(lcm_command_.num_torques >= num_joints_);
    memcpy(torque, lcm_command_.joint_torque.data() + joint_offset,
           kNumJoints * sizeof(double));
    return true;
  }

  int PollForCommandMessage() {
    return lcm_.handleTimeout(0);
  }

  int GetLcmFileno() {
    return lcm_.getFileno();
  }

  void PublishStateUpdate() {
    const uint64_t now =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

    if (first_publish_attempt_us_ < 0) {
      first_publish_attempt_us_ = now;
    }

    const uint64_t command_guard = FLAGS_start_command_guard * 1000000;
    if (now - first_publish_attempt_us_ < command_guard) {
      command_started_ = false;
    } else {
      command_started_ = true;
    }
    lcm_.publish(FLAGS_lcm_status_channel, &lcm_status_);
    lcm_.publish(FLAGS_lcm_status_telemetry_channel, &lcm_status_telemetry_);
  }

  bool IsCommandExpired() const {
    const uint64_t expire_utime = FLAGS_command_expire * 1e6;
    const uint64_t stale_utime = micros() - command_receipt_utime_;
    return stale_utime > expire_utime;
  }

 private:
  void HandleCommandMessage(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const lcmt_iiwa_command* command) {
    if (!command_started_) {
      throw std::runtime_error("First command received within guard time.  "
                               "Aborting.");
    }
    lcm_command_ = *command;
    command_receipt_utime_ = micros();
  }

  const int num_joints_;
  lcm::LCM lcm_;
  lcmt_iiwa_status lcm_status_{};
  lcmt_iiwa_status_telemetry lcm_status_telemetry_{};
  lcmt_iiwa_command lcm_command_{};
  uint64_t command_receipt_utime_{};
  std::vector<double> external_torque_limit_;

  // Filters
  std::vector<DiscreteTimeLowPassFilter<double>> vel_filters_;
  std::vector<double> vel_filtered_;
  std::vector<int64_t> utime_last_;

  TimeSyncFilter sync_;

  // Implement a guard against receiving commands which are unlikely to be
  // valid on startup.
  int64_t first_publish_attempt_us_{-1};
  bool command_started_{false};
};

class KukaFRIClient : public KUKA::FRI::LBRClient {
 public:
  KukaFRIClient(int robot_id, KukaLCMClient* lcm_client)
      : robot_id_(robot_id),
        lcm_client_(lcm_client) {
    // Joint limits derived from visual inspection of KUKA controller
    // output display.  Values in +/- degrees from center.
    joint_limits_.push_back(ToRadians(170));
    joint_limits_.push_back(ToRadians(120));
    joint_limits_.push_back(ToRadians(170));
    joint_limits_.push_back(ToRadians(120));
    joint_limits_.push_back(ToRadians(170));
    joint_limits_.push_back(ToRadians(120));
    joint_limits_.push_back(ToRadians(175));
  }

  ~KukaFRIClient() {}

  void onStateChange(
      KUKA::FRI::ESessionState oldState,
      KUKA::FRI::ESessionState newState) override {
    KUKA::FRI::LBRClient::onStateChange(oldState, newState);

    const KUKA::FRI::LBRState& state = robotState();
    const uint64_t time = state.getTimestampSec() * 1e6 +
        state.getTimestampNanoSec() / 1e3;

    std::cerr << "onStateChange ( " << time << "): old " << oldState
              << " new " << newState << std::endl;

    std::cerr << "onStateChange ( " << time
              << "): quality " << state.getConnectionQuality()
              << " safety " << state.getSafetyState()
              << " oper " << state.getOperationMode()
              << " drive " << state.getDriveState()
              << " control " << state.getControlMode()
              << " command " << state.getClientCommandMode()
              << " overlay " << state.getOverlayType()
              << std::endl;

    if (newState == KUKA::FRI::COMMANDING_ACTIVE) {
      joint_position_when_command_entered_.resize(kNumJoints, 0.);
      std::memcpy(joint_position_when_command_entered_.data(),
                  state.getMeasuredJointPosition(),
                  kNumJoints * sizeof(double));

      if (!has_entered_command_state_) {
        has_entered_command_state_ = true;
      } else {
        // We've been in command state before, something bad happened
        // (we did switch to another state, after all), so just stop
        // doing anything.  There's a flag to override this if it's
        // really what the user wants.
        std::cerr << "Re-entering command state." << std::endl;
        if (FLAGS_restart_fri) {
          std::cerr << "Allowing robot motion again due to --restart_fri"
                    << std::endl;
        } else {
          inhibit_motion_in_command_state_ = true;
          std::cerr << "Holding position and ignoring LCM commands.\n"
                    << std::endl;
        }
      }
    }
  }

  void monitor() override {
    KUKA::FRI::LBRClient::monitor();
    lcm_client_->UpdateRobotState(robot_id_, robotState());
    if (!lcm_client_->CheckSafety(robot_id_)) {
      lcm_client_->PrintRobotState(robot_id_, std::cerr);
      throw std::runtime_error("Robot" + std::to_string(robot_id_) +
                               " is in an unsafe state.");
    }
  }

  void waitForCommand() override {
    KUKA::FRI::LBRClient::waitForCommand();

    // The value of the torques sent in waitForCommand doesn't matter,
    // but we have to send something.
    if (robotState().getClientCommandMode() == KUKA::FRI::TORQUE) {
      double torque[kNumJoints] = { 0., 0., 0., 0., 0., 0., 0.};
      robotCommand().setTorque(torque);
    }

    lcm_client_->UpdateRobotState(robot_id_, robotState());
    if (!lcm_client_->CheckSafety(robot_id_)) {
      lcm_client_->PrintRobotState(robot_id_, std::cerr);
      throw std::runtime_error("Robot" + std::to_string(robot_id_) +
                               " is in an unsafe state.");
    }
  }

  void command() override {
    lcm_client_->UpdateRobotState(robot_id_, robotState());
    if (!lcm_client_->CheckSafety(robot_id_)) {
      lcm_client_->PrintRobotState(robot_id_, std::cerr);
      throw std::runtime_error("Robot" + std::to_string(robot_id_) +
                               " is in an unsafe state.");
    }

    // TODO(sam.creasey): Is there a sensible torque limit to saturate with or
    // fault based on here?

    if (FLAGS_torque_only) {
      DRAKE_DEMAND(robotState().getClientCommandMode() == KUKA::FRI::TORQUE);
      const double *pos_measured = robotState().getMeasuredJointPosition();
      // Loopback current state. We should have zero position and velocity gains
      // but I (Eric) think this makes the controller happy (won't get "Illegal
      // axis delta" or what not).
      double pos[kNumJoints] = { 0., 0., 0., 0., 0., 0., 0.};
      memcpy(pos, pos_measured, kNumJoints * sizeof(double));
      robotCommand().setJointPosition(pos);

      double torque[kNumJoints] = { 0., 0., 0., 0., 0., 0., 0.};

      bool command_valid = lcm_client_->GetTorqueCommand(robot_id_, torque);
      if (command_valid && lcm_client_->IsCommandExpired()) {
        command_valid = false;
        if (!warned_about_expiration_) {
          std::cerr
              << "Torque command expiration! Engaging holding controller."
              << std::endl;
          // Update hold position.
          std::memcpy(
              joint_position_when_command_entered_.data(),
              pos_measured, kNumJoints * sizeof(double));
          warned_about_expiration_ = true;
        }
      } else {
        if (warned_about_expiration_) {
          std::cerr
              << "Received fresh command. Resuming nominal control."
              << std::endl;
          warned_about_expiration_ = false;
        }
      }

      if (inhibit_motion_in_command_state_ || !command_valid) {
        // Position control holding current position.
        const double* pos_desired =
            joint_position_when_command_entered_.data();
        const double* vel_estimated = lcm_client_->GetVelocityFiltered();
        for (int i = 0; i < kNumJoints; ++i) {
          const double kp_i = kTorqueOnlyKp[i] * FLAGS_torque_only_kp_scale;
          const double kd_i = kTorqueOnlyKd[i] * FLAGS_torque_only_kd_scale;
          const double pos_error_i = pos_measured[i] - pos_desired[i];
          const double vel_error_i = vel_estimated[i];
          torque[i] = -kp_i * pos_error_i - kd_i * vel_error_i;
        }
      }
      robotCommand().setTorque(torque);
    } else {
      double pos[kNumJoints] = { 0., 0., 0., 0., 0., 0., 0.};
      const bool command_valid =
          lcm_client_->GetPositionCommand(robot_id_, pos);
      if (inhibit_motion_in_command_state_ || !command_valid) {
        // No command received, just command the position when we
        // entered command state.
        assert(joint_position_when_command_entered_.size() == kNumJoints);
        memcpy(pos, joint_position_when_command_entered_.data(),
               kNumJoints * sizeof(double));
      } else {
        // Only apply the joint limits when we're responding to LCM.  If
        // we don't want to command motion, don't change anything.
        ApplyJointLimits(pos);
      }
      robotCommand().setJointPosition(pos);

      // Check if we're in torque mode, and send torque commands too if
      // we are.
      if (robotState().getClientCommandMode() == KUKA::FRI::TORQUE) {
        double torque[kNumJoints] = { 0., 0., 0., 0., 0., 0., 0.};
        if (command_valid && !inhibit_motion_in_command_state_) {
          lcm_client_->GetTorqueCommand(robot_id_, torque);
        }
        // TODO(sam.creasey): Is there a sensible torque limit to apply here?
        robotCommand().setTorque(torque);
      }
    }
  }

 private:
  void ApplyJointLimits(double* pos) const {
    const double joint_tol = ToRadians(kJointLimitSafetyMarginDegree);
    for (int i = 0; i < kNumJoints; i++) {
      pos[i] = std::max(std::min(pos[i], (joint_limits_[i] - joint_tol)),
                        ((-joint_limits_[i]) + joint_tol));
    }
  }

  int robot_id_;
  KukaLCMClient* lcm_client_;
  std::vector<double> joint_limits_;
  // What was the joint position when we entered command state?
  // (provided so that we can keep holding that position).
  std::vector<double> joint_position_when_command_entered_;
  bool has_entered_command_state_{false};
  bool inhibit_motion_in_command_state_{false};
  bool warned_about_expiration_{};
};

int do_main() {
  assert(FLAGS_ext_trq_limit > 0);

  if (FLAGS_mlockall || FLAGS_realtime) {
    // Lock memory to prevent the OS from paging us out.
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
      perror("mlockall failed");
      return EXIT_FAILURE;
    }
    std::cout << "Locked memory to prevent paging out" << std::endl;
  } else {
    std::cerr << "Memory is not locked" << std::endl;
  }

  if (FLAGS_realtime) {
    // Set realtime priority.
    struct sched_param scheduler_options = {};
    // 90 is as high as you generally want to go on PREEMPT_RT machines.
    // Any higher than this and you will have higher priority than the kernel
    // threads that serve interrupts, and the system will stop responding.
    const int realtime_priority = std::min(90, std::max(0, FLAGS_priority));
    scheduler_options.sched_priority = realtime_priority;
    const int policy = (FLAGS_sched_fifo) ? SCHED_FIFO : SCHED_RR;
    if (sched_setscheduler(0, policy, &scheduler_options) != 0) {
      perror("sched_setscheduler failed");
      return EXIT_FAILURE;
    }
    std::cout << "Got realtime priority" << std::endl;
  } else {
    std::cerr << "Running without realtime priority" << std::endl;
  }

  std::vector<KUKA::FRI::UdpConnection> connections;
  connections.reserve(FLAGS_num_robots);
  std::vector<KukaFRIClient> clients;
  clients.reserve(FLAGS_num_robots);
  std::vector<KUKA::FRI::ClientApplication> apps;
  apps.reserve(FLAGS_num_robots);
  KukaLCMClient lcm_client(FLAGS_num_robots, FLAGS_lcm_url);
  // One fd entry for each of the robot FRI connections plus the LCM
  // client.
  std::vector<struct pollfd> fds(FLAGS_num_robots + 1);

  for (int i = 0; i < FLAGS_num_robots; i++) {
    connections.emplace_back();
    clients.emplace_back(i, &lcm_client);
    apps.emplace_back(connections[i], clients[i]);
    apps[i].connect(FLAGS_fri_port + i, NULL);

    fds[i].fd = connections[i].udpSock();
    fds[i].events = POLLIN;
    fds[i].revents = 0;
    std::cerr << "Listening for robot " << i
              << " port " << FLAGS_fri_port + i
              << std::endl;
  }
  fds.back().fd = lcm_client.GetLcmFileno();
  fds.back().events = POLLIN;
  fds.back().revents = 0;

  bool success = true;
  while (success) {
    int result = poll(fds.data(), fds.size(), -1);
    if (result < 0) {
      perror("poll failed");
      break;
    }

    // Handle any incoming LCM command messages before running the FRI
    // control loop.
    if (fds.back().revents != 0) {
      fds.back().revents = 0;  // TODO(sam.creasey) do I actually need
                               // to clear that?
      lcm_client.PollForCommandMessage();
    }

    bool robot_stepped = false;
    for (int i = 0; i < FLAGS_num_robots; i++) {
      if (fds[i].revents != 0) {
        robot_stepped = true;
        fds[i].revents = 0;  // TODO(sam.creasey) do I actually need
                             // to clear that?
        success = apps[i].step();
        if (!success) { break; }
      }
    }
    if (!success) { break; }
    if (robot_stepped) {
      lcm_client.PublishStateUpdate();
    }
  }

  for (int i = 0; i < FLAGS_num_robots; i++) {
    apps[i].disconnect();
  }

  return 0;
}

}  // namespace kuka_driver

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return kuka_driver::do_main();
}
