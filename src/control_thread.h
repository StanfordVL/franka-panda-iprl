/**
 * control_thread.h
 *
 * Copyright 2018. All Rights Reserved.
 * Stanford IPRL
 *
 * Created: December 20, 2018
 * Authors: Toki Migimatsu
 */

#ifndef FRANKA_DRIVER_CONTROL_THREAD_H_
#define FRANKA_DRIVER_CONTROL_THREAD_H_

#include <exception>   // std::runtime_error
#include <functional>  // std::function
#include <memory>      // std::shared_ptr
#include <sstream>     // std::stringstream
#include <string>      // std::string
#include <cmath>	   // M_PI 

#include <franka/model.h>
#include <franka/robot.h>

// for reset motion generation
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot_state.h>
#include <Eigen/Core>

#include "args.h"

namespace franka_driver {

struct SharedMemory;

// For resetting robot to neutral joint position.
const std::array<double, 7> NEUTRAL_JOINT_POSITION {{0., -M_PI/6., 0., -5.*M_PI/6., 0., 2.*M_PI/3., 0.}};


enum class ControlMode {
  IDLE, FLOATING, TORQUE, JOINT_POSITION, JOINT_VELOCITY, CARTESIAN_POSE, DELTA_CARTESIAN_POSE, CARTESIAN_VELOCITY, RESET
};

std::stringstream& operator<<(std::stringstream& ss, ControlMode mode);
std::stringstream& operator>>(std::stringstream& ss, ControlMode& mode);
std::string ControlModeToString(ControlMode mode);

enum class ControlStatus {
  RUNNING, FINISHED, ERROR
};

std::stringstream& operator<<(std::stringstream& ss, ControlStatus status);

class SwitchControllerException : public std::runtime_error {
 public:
  SwitchControllerException(const std::string msg) : std::runtime_error(msg) {}
  ~SwitchControllerException() {}
};

class CommandTimeoutException: public std::runtime_error { 
 public: 
 	CommandTimeoutException(const std::string msg) : std::runtime_error(msg) {}
 	~CommandTimeoutException() {}
};

void RunControlLoop(const Args& args, const std::shared_ptr<SharedMemory>& globals,
                    franka::Robot& robot, const franka::Model& model);

std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
CreateTorqueController(const Args& args, const std::shared_ptr<SharedMemory>& globals,
                       const franka::Model& model);

std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration)>
CreateCartesianPoseController(const Args& args, const std::shared_ptr<SharedMemory>& globals,
                              franka::Robot& robot, const franka::Model& model);
/**
 * Sets a default collision behavior, joint impedance, Cartesian impedance, and filter frequency.
 *
 * @param[in] robot Robot instance to set behavior on.
 */
void setDefaultBehavior(franka::Robot& robot);

/**
 * An example showing how to generate a joint pose motion to a goal position. Adapted from:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 */
class JointMotionGenerator {
 public:
  /**
   * Creates a new MotionGenerator instance for a target q.
   *
   * @param[in] speed_factor General speed factor in range [0, 1].
   * @param[in] q_goal Target joint positions.
   */
  JointMotionGenerator(double speed_factor, const std::array<double, 7> q_goal);

  /**
   * Sends joint position calculations
   *
   * @param[in] robot_state Current state of the robot.
   * @param[in] period Duration of execution.
   *
   * @return Joint positions for use inside a control loop.
   */
  franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);

 private:
  using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
  using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

  bool calculateDesiredValues(double t, Vector7d* delta_q_d) const;
  void calculateSynchronizedValues();

  static constexpr double kDeltaQMotionFinished = 1e-6;
  const Vector7d q_goal_;

  Vector7d q_start_;
  Vector7d delta_q_;

  Vector7d dq_max_sync_;
  Vector7d t_1_sync_;
  Vector7d t_2_sync_;
  Vector7d t_f_sync_;
  Vector7d q_1_;

  double time_ = 0.0;

  Vector7d dq_max_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
  Vector7d ddq_max_start_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
  Vector7d ddq_max_goal_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();


};  

}  // namespace franka_driver

#endif  // FRANKA_DRIVER_CONTROL_THREAD_H_
