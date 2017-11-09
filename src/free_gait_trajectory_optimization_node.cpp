#include <ros/ros.h>
#include <free_gait_trajectory_optimization/ReceiveTrajectory.hpp>
#include <free_gait_trajectory_optimization/SolveTrajectoryOptimization.hpp>
#include <free_gait_trajectory_optimization/SendTrajectory.hpp>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "free_gait_trajectory_optimization");
  ros::NodeHandle nodeHandle("~");
  free_gait_trajectory_optimization::SendTrajectory sendTrajectory(nodeHandle);

  // Get gait and goal from user
  std::vector<int> contactcodes = getGaitFromUser();
  Eigen::VectorXd base0(6), baseGoalDelta(6), baseGoal(6);
  getBaseGoal(baseGoalDelta);

  // Get initial conditions
  base0 = sendTrajectory.getBase0();
  Eigen::Matrix<double, 12, 1> joint0 = sendTrajectory.getJoints0();
  double zGround = sendTrajectory.getGroundZ();
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
  solveTrajectoryOptimization(contactcodes, base0, joint0, zGround, baseGoal, x_, u_, t_, c_, false);

  auto cFlags = contactCodesToFlags(c_);
  sendTrajectory.executeTrajectory(t_, cFlags, x_, u_);
//  sendTrajectory.start();


  ros::spin();
  return 0;
}
