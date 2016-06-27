#include <cmath>
#include <cstring>

#include <iostream>
#include <limits>

#include <lcm/lcm-cpp.hpp>

#include "friClientApplication.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"

#include "lcmtypes/drake/lcmt_iiwa_command.hpp"
#include "lcmtypes/drake/lcmt_iiwa_status.hpp"
#include "lcmtypes/drake/lcmt_iiwa_trajectory.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* lcm_status_channel = "IIWA_STATUS";
const char* lcm_plan_channel = "IIWA_PLAN";

double ToRadians(double degrees) {
  return degrees * M_PI / 180.;
}

class KukaLCMClient : public KUKA::FRI::LBRClient {
 public:
  KukaLCMClient()
      : plan_start_time_(0),
        cur_plan_step_(-1) {
    lcm_status_.timestamp = 0;
    lcm_status_.num_joints = num_joints_;
    lcm_status_.joint_position_measured.resize(num_joints_);
    lcm_status_.joint_position_commanded.resize(num_joints_);
    lcm_status_.joint_position_ipo.resize(num_joints_);
    lcm_status_.joint_torque_measured.resize(num_joints_);
    lcm_status_.joint_torque_commanded.resize(num_joints_);
    lcm_status_.joint_torque_external.resize(num_joints_);

    // Joint limits derived from visual inspection of KUKA controller
    // output display.  Values in +/- degrees from center.
    joint_limits_.push_back(ToRadians(170));
    joint_limits_.push_back(ToRadians(120));
    joint_limits_.push_back(ToRadians(170));
    joint_limits_.push_back(ToRadians(120));
    joint_limits_.push_back(ToRadians(170));
    joint_limits_.push_back(ToRadians(120));
    joint_limits_.push_back(ToRadians(175));

    lcm_.subscribe(lcm_plan_channel, &KukaLCMClient::HandlePlanMessage, this);
  }

  ~KukaLCMClient() {}

  virtual void onStateChange(KUKA::FRI::ESessionState oldState,
                             KUKA::FRI::ESessionState newState) {
    KUKA::FRI::LBRClient::onStateChange(oldState, newState);

    std::cerr << "onStateChange: old " << oldState
              << " new " << newState << std::endl;
  }

  virtual void monitor() {
    KUKA::FRI::LBRClient::monitor();
    PublishStateUpdate();
  }

  virtual void waitForCommand() {
    KUKA::FRI::LBRClient::waitForCommand();
    PublishStateUpdate();
  }
  virtual void command() {
    KUKA::FRI::LBRClient::command();
    //    double pos[num_joints_] = { 0., 0., 0., 0., 0., 0., 0.};
    //robotCommand().setJointPosition(pos);
    PublishStateUpdate();
    AdvancePlan();
  }

 private:
  void ApplyJointLimits(double* pos) const {
      const double joint_tol = ToRadians(5);
      for (int i = 0; i < num_joints_; i++) {
        pos[i] = std::max(std::min(pos[i], (joint_limits_[i] - joint_tol)),
                          ((-joint_limits_[i]) + joint_tol));
      }

  }

  void HandlePlanMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                         const lcmt_iiwa_trajectory* plan) {
    std::cerr << "Got plan, " << plan->num_states << " states" << std::endl;
    plan_ = *plan;
    cur_plan_step_ = 0;
    plan_start_time_ = lcm_status_.timestamp;
  }

  void PublishStateUpdate() {
    const KUKA::FRI::LBRState& state = robotState();

    lcm_status_.timestamp = state.getTimestampSec() * 1e3 +
        state.getTimestampNanoSec() / 1e6;
    std::memcpy(lcm_status_.joint_position_measured.data(),
                state.getMeasuredJointPosition(), num_joints_ * sizeof(double));
    std::memcpy(lcm_status_.joint_position_commanded.data(),
                state.getCommandedJointPosition(), num_joints_ * sizeof(double));
    if (state.getIpoJointPosition() != NULL) {
      std::memcpy(lcm_status_.joint_position_ipo.data(),
                  state.getIpoJointPosition(), num_joints_ * sizeof(double));
    } else {
      lcm_status_.joint_position_ipo.assign(
          num_joints_, std::numeric_limits<double>::quiet_NaN());

    }
    std::memcpy(lcm_status_.joint_torque_measured.data(),
                state.getMeasuredTorque(), num_joints_ * sizeof(double));
    std::memcpy(lcm_status_.joint_torque_commanded.data(),
                state.getCommandedTorque(), num_joints_ * sizeof(double));
    std::memcpy(lcm_status_.joint_torque_external.data(),
                state.getExternalTorque(), num_joints_ * sizeof(double));
    lcm_.publish(lcm_status_channel, &lcm_status_);
    // Also poll for new messages.
    lcm_.handleTimeout(0);
  }

  void AdvancePlan() {
    if (lcm_status_.timestamp == 0) { return; }

    double pos[num_joints_] = { 0., 0., 0., 0., 0., 0., 0.};
    if (cur_plan_step_ < 0) {
        // Startup, just command the current position.
      memcpy(pos, lcm_status_.joint_position_measured.data(),
             num_joints_ * sizeof(double));
    } else {
      std::cerr << "plan step " << cur_plan_step_ << std::endl;
      const int64_t next_step_time =
          plan_start_time_ + plan_.plan[cur_plan_step_].timestamp;
      double time_to_next_step = next_step_time - lcm_status_.timestamp;
            std::cerr << "next_step_time " << next_step_time
                << " time_to_next_step " << time_to_next_step
                << std::endl;
      if (time_to_next_step <= 0.) {
        std::cerr << "didn't reach next step in time!" << std::endl;
        // TODO(sam.creasey) 0.1s is totally arbitrary.
        time_to_next_step = 100;
      }

      const double step_size = 5.;
      const double num_steps = time_to_next_step / step_size;
      std::cerr << "remaining steps: " << num_steps << std::endl;
      std::cerr << "commanding " << plan_.plan[cur_plan_step_].num_joints
                << " joints at step " << cur_plan_step_
                << std::endl;
      double target_pos[num_joints_];
      memcpy(target_pos, plan_.plan[cur_plan_step_].joint_position.data(),
             plan_.plan[cur_plan_step_].num_joints * sizeof(double));

      // TODO(sam.creasey) this is totally arbitrary
      //const double max_joint_rate = 1.;  // rad/s
      const double max_delta = std::min(0.1, 0.0005 * num_steps);  // rad
      const double target_tol = 1e-2;  // rad
      bool at_target = true;


      std::cerr << "desired rate (command): ";
      for (int i = 0; i < num_joints_; i++) {
        std::cerr << target_pos[i];
        double joint_delta = target_pos[i] - lcm_status_.joint_position_measured[i];
        joint_delta = std::max(-max_delta,
                               std::min(max_delta, joint_delta));
#if 0
        double joint_step = joint_delta / num_steps;
        const double max_joint_step = (max_joint_rate * 1e3) / step_size;
        joint_step = std::min(max_joint_step,
                              std::max(-max_joint_step, joint_step));
        std::cerr << " " << joint_step;
#endif
#if 0
        double joint_rate = joint_delta / time_to_next_step;
        if (std::fabs(joint_rate) > max_joint_rate) {
          std::cerr << "capping rate of joint " << i << " commanded "
                    << joint_rate << std::endl;
          pos[i] = std::min(
              std::max(pos[i], -(max_joint_rate * time_to_next_step)),
              (max_joint_rate * time_to_next_step));
        }
#endif
        pos[i] = lcm_status_.joint_position_measured[i] + joint_delta;
        std::cerr << "(" << pos[i] << ") ";

        if (std::fabs(joint_delta) > target_tol) {
          at_target = false;
        }
      }
      std::cerr << std::endl;

      if (at_target) {
        std::cerr << "Reached target for step " << cur_plan_step_
                  << std::endl;
        if (time_to_next_step < 200) {
          cur_plan_step_++;
        }
        if (cur_plan_step_ >= plan_.num_states) {
          cur_plan_step_ = plan_.num_states - 1;
        }
      }
    }

    // for (int i = 0; i < num_joints_; i++) {
    //   pos[i] = 0;
    // }
    ApplyJointLimits(pos);
    robotCommand().setJointPosition(pos);
  }

  static const int num_joints_ = 7;

  lcm::LCM lcm_;
  lcmt_iiwa_status lcm_status_;
  std::vector<double> joint_limits_;
  int64_t plan_start_time_;
  int cur_plan_step_;
  lcmt_iiwa_trajectory plan_;
};

int do_main(int argc, const char* argv[]) {

  KUKA::FRI::UdpConnection connection;
  KukaLCMClient client;
  KUKA::FRI::ClientApplication app(connection, client);

  // TODO(sam.creasey) make host/port configurable
  const int default_port = 30200;
  app.connect(default_port, NULL);

  bool success = true;
  while (success) {
    success = app.step();
  }
  app.disconnect();

  return 0;
}

} // namespace
} // namespace kuka_iiwa_arm
} // namespace examples
} // namespace drake

int main(int argc, const char* argv[]) {
  return drake::examples::kuka_iiwa_arm::do_main(argc, argv);
}
