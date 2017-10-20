
#include "poll.h"

#include <cassert>
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

#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"

#include "low_pass_filter.h"

using drake::lcmt_iiwa_command;
using drake::lcmt_iiwa_status;

namespace {

const int kNumJoints = 7;
const int kDefaultPort = 30200;
const char* kLcmStatusChannel = "IIWA_STATUS";
const char* kLcmCommandChannel = "IIWA_COMMAND";
const double kTimeStep = 0.005;
const double kJointLimitSafetyMarginDegree = 1;
const double kJointTorqueSafetyMarginNm = 60;
const double kJointTorqueSafetyMarginScale[kNumJoints] = {1, 1, 1, 0.5,
                                                          0.2, 0.2, 0.1};

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

}  // namespace

DEFINE_double(ext_trq_limit, kJointTorqueSafetyMarginNm,
              "Maximal external torque that triggers safety freeze");
DEFINE_int32(fri_port, kDefaultPort, "First UDP port for FRI messages");
DEFINE_int32(num_robots, 1, "Number of robots to control");
DEFINE_string(lcm_command_channel, kLcmCommandChannel,
              "Channel to receive LCM command messages on");
DEFINE_string(lcm_status_channel, kLcmStatusChannel,
              "Channel to send LCM status messages on");

namespace kuka_driver {

class KukaLCMClient  {
 public:
  explicit KukaLCMClient(int num_robots)
      : num_joints_(num_robots * kNumJoints) {

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

    // Use -1 as a sentinal to indicate that no command has been
    // received.
    lcm_command_.utime = -1;

    // Initialize filters.
    const double cutoff_hz = 40;
    vel_filters_.resize(
        num_joints_, DiscreteTimeLowPassFilter<double>(cutoff_hz, kTimeStep));
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
    const int64_t utime_now =
        state.getTimestampSec() * 1e6 + state.getTimestampNanoSec() / 1e3;
    // Get delta time for this robot.
    double robot_dt = 0.;
    if (utime_last_.at(robot_id) != -1) {
      robot_dt = (utime_now - utime_last_.at(robot_id)) / 1e6;
      // Check timing
      if (std::abs(robot_dt - kTimeStep) > 1e-3) {
        std::cout << "Warning: dt " << robot_dt
                  << ", kTimeStep " << kTimeStep << "\n";
      }
    }
    utime_last_.at(robot_id) = utime_now;

    // The choice of robot id 0 for the timestamp is arbitrary.
    if (robot_id == 0) {
      lcm_status_.utime = utime_now;
    }

    // Velocity filtering.
    if (robot_dt != 0.) {
      for (int i = 0; i < kNumJoints; i++) {
        const int index = joint_offset + i;
        const double q_diff = state.getMeasuredJointPosition()[i] -
                              lcm_status_.joint_position_measured[index];
        lcm_status_.joint_velocity_estimated[index] =
            vel_filters_[index].filter(q_diff / robot_dt);
      }
    }

    // Set other joint states.
    std::memcpy(lcm_status_.joint_position_measured.data() + joint_offset,
                state.getMeasuredJointPosition(), kNumJoints * sizeof(double));
    std::memcpy(lcm_status_.joint_position_commanded.data() + joint_offset,
                state.getCommandedJointPosition(), kNumJoints * sizeof(double));
    if (state.getIpoJointPosition() != NULL) {
      std::memcpy(lcm_status_.joint_position_ipo.data() + joint_offset,
                  state.getIpoJointPosition(), kNumJoints * sizeof(double));
    } else {
      for (int i = joint_offset; i < joint_offset + kNumJoints; i++) {
        lcm_status_.joint_position_ipo[i] =
            std::numeric_limits<double>::quiet_NaN();
      }
    }
    std::memcpy(lcm_status_.joint_torque_measured.data() + joint_offset,
                state.getMeasuredTorque(), kNumJoints * sizeof(double));
    std::memcpy(lcm_status_.joint_torque_commanded.data() + joint_offset,
                state.getCommandedTorque(), kNumJoints * sizeof(double));
    std::memcpy(lcm_status_.joint_torque_external.data() + joint_offset,
                state.getExternalTorque(), kNumJoints * sizeof(double));
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
      const double safety_thresh =
          FLAGS_ext_trq_limit * kJointTorqueSafetyMarginScale[i];
      if (std::fabs(ext_torque) > safety_thresh) {
        return false;
      }
    }

    return true;
  }

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

    assert(lcm_command_.num_torques >= num_joints_);
    memcpy(torque, lcm_command_.joint_torque.data() + joint_offset,
           kNumJoints * sizeof(double));
    return true;
  }

  void PublishStateUpdate() {
    lcm_.publish(FLAGS_lcm_status_channel, &lcm_status_);
    // Also poll for new messages.
    lcm_.handleTimeout(0);
  }

 private:
  void HandleCommandMessage(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const lcmt_iiwa_command* command) {
    lcm_command_ = *command;
  }

  const int num_joints_;
  lcm::LCM lcm_;
  lcmt_iiwa_status lcm_status_{};
  lcmt_iiwa_command lcm_command_{};

  // Filters
  std::vector<DiscreteTimeLowPassFilter<double>> vel_filters_;
  std::vector<int64_t> utime_last_;
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

  virtual void onStateChange(KUKA::FRI::ESessionState oldState,
                             KUKA::FRI::ESessionState newState) {
    KUKA::FRI::LBRClient::onStateChange(oldState, newState);

    const KUKA::FRI::LBRState& state = robotState();
    const uint64_t time = state.getTimestampSec() * 1e6 +
        state.getTimestampNanoSec() / 1e3;

    if (newState == KUKA::FRI::COMMANDING_ACTIVE) {
      joint_position_when_command_entered_.resize(kNumJoints, 0.);
      std::memcpy(joint_position_when_command_entered_.data(),
                  state.getMeasuredJointPosition(),
                  kNumJoints * sizeof(double));
      //lcm_command_.utime = -1;
    }

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
  }

  virtual void monitor() {
    KUKA::FRI::LBRClient::monitor();
    lcm_client_->UpdateRobotState(robot_id_, robotState());
    if (!lcm_client_->CheckSafety(robot_id_)) {
      lcm_client_->PrintRobotState(robot_id_, std::cerr);
      throw std::runtime_error("Robot" + std::to_string(robot_id_) +
                               " is in an unsafe state.");
    }
  }

  virtual void waitForCommand() {
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

  virtual void command() {
    lcm_client_->UpdateRobotState(robot_id_, robotState());
    if (!lcm_client_->CheckSafety(robot_id_)) {
      lcm_client_->PrintRobotState(robot_id_, std::cerr);
      throw std::runtime_error("Robot" + std::to_string(robot_id_) +
                               " is in an unsafe state.");
    }

    double pos[kNumJoints] = { 0., 0., 0., 0., 0., 0., 0.};
    const bool command_valid =
        lcm_client_->GetPositionCommand(robot_id_, pos);
    if (!command_valid) {
      // No command received, just command the position when we
      // entered command state.
      assert(joint_position_when_command_entered_.size() == kNumJoints);
      memcpy(pos, joint_position_when_command_entered_.data(),
             kNumJoints * sizeof(double));
    }
    ApplyJointLimits(pos);
    robotCommand().setJointPosition(pos);

    // Check if we're in torque mode, and send torque commands too if
    // we are.
    if (robotState().getClientCommandMode() == KUKA::FRI::TORQUE) {
      double torque[kNumJoints] = { 0., 0., 0., 0., 0., 0., 0.};
      lcm_client_->GetTorqueCommand(robot_id_, torque);
      // TODO(sam.creasey): Is there a sensible torque limit to apply here?
      robotCommand().setTorque(torque);
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
};

int do_main() {
  assert(FLAGS_ext_trq_limit > 0);

  std::vector<KUKA::FRI::UdpConnection> connections;
  connections.reserve(FLAGS_num_robots);
  std::vector<KukaFRIClient> clients;
  clients.reserve(FLAGS_num_robots);
  std::vector<KUKA::FRI::ClientApplication> apps;
  apps.reserve(FLAGS_num_robots);
  KukaLCMClient lcm_client(FLAGS_num_robots);
  std::vector<struct pollfd> fds(FLAGS_num_robots);

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

  bool success = true;
  while (success) {
    int result = poll(fds.data(), fds.size(), -1);
    if (result < 0) {
      perror("poll failed");
      break;
    }

    for (int i = 0; i < FLAGS_num_robots; i++) {
      if (fds[i].revents != 0) {
        fds[i].revents = 0;  // TODO(sam.creasey) do I actually need
                             // to clear that?
        success = apps[i].step();
        if (!success) { break; }
      }
    }
    if (!success) { break; }
    lcm_client.PublishStateUpdate();
  }

  for (int i = 0; i < FLAGS_num_robots; i++) {
    apps[i].disconnect();
  }

  return 0;
}

} // namespace kuka_driver

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return kuka_driver::do_main();
}
