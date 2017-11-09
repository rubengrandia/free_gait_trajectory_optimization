//
// Created by ruben on 02.11.17.
//


#include <free_gait_trajectory_optimization/ReceiveTrajectory.hpp>
#include <free_gait_trajectory_optimization/SolveTrajectoryOptimization.hpp>


int main(int argc, char** argv)
{
  // Get gait and goal from user
  std::vector<int> contactcodes = getGaitFromUser();
  Eigen::VectorXd base0(6), baseGoalDelta(6), baseGoal(6);
  getBaseGoal(baseGoalDelta);

  // Get initial conditions
  LRD::state_vector_t initial_state;
  AnymalKinematics lrkin;
  lrkin.GetCanonicalPoseState(initial_state);
  base0 = initial_state.head(6);
  Eigen::Matrix<double, 12, 1> joint0 = initial_state.segment(6, 12);
  double zGround = lrkin.getDefaultGroundDistance();
  baseGoal.head(2) = baseGoalDelta.head(2);
  baseGoal.tail(4) = baseGoalDelta.tail(4) + base0.tail(4);

  LRD::state_vector_array_t x_;
  LRD::control_vector_array_t u_;
  std::vector<double> t_;
  std::vector<int> c_;

  std::cout << "base0: " << base0.transpose() << std::endl;
  std::cout << "joint0: " << joint0.transpose() << std::endl;
  std::cout << "zGround: " << zGround << std::endl;
  std::cout << "baseGoal: " << baseGoal.transpose() << std::endl;
  solveTrajectoryOptimization(contactcodes, base0, joint0, zGround, baseGoal, x_, u_, t_, c_, true);

  return 0;
}