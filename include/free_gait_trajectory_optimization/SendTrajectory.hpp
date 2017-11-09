#pragma once


#include <Eigen/Dense>
#include <Eigen/StdVector>

// ROS
#include <ros/ros.h>

// Free Gait
#include <free_gait_core/free_gait_core.hpp>
#include <free_gait_ros/FreeGaitActionClient.hpp>
#include <free_gait_ros/AdapterRos.hpp>
#include <free_gait_ros/StepFrameConverter.hpp>
#include <free_gait_ros/StepRosConverter.hpp>

namespace free_gait_trajectory_optimization {

class SendTrajectory
{
    typedef Eigen::Matrix<double, 36, 1> StateVector;
    typedef Eigen::Matrix<double, 18, 1> GeneralizedCoordinates;
    typedef Eigen::Matrix<double, 12, 1> JointTorques;
    typedef Eigen::Matrix<bool, 4, 1> ContactFlags;
    typedef std::vector<StateVector, Eigen::aligned_allocator<StateVector>> StateVectorTrajectory;
    typedef std::vector<GeneralizedCoordinates, Eigen::aligned_allocator<GeneralizedCoordinates> > GeneralizedCoordinatesTrajectory;
    typedef std::vector<JointTorques, Eigen::aligned_allocator<JointTorques> > JointTorquesTrajectory;
    typedef std::vector<ContactFlags, Eigen::aligned_allocator<ContactFlags> > ContactFlagsTrajectory;
    typedef std::vector<double> TimeTrajectory;


public:
    explicit SendTrajectory(ros::NodeHandle& nodeHandle);
    virtual ~SendTrajectory();
    Eigen::Matrix<double, 6, 1> getBase0();
    Eigen::Matrix<double, 12, 1> getJoints0();
    double getGroundZ();

//    bool start();
    bool executeTrajectory(const SendTrajectory::TimeTrajectory &timeTrajectory,
                           const SendTrajectory::ContactFlagsTrajectory &contactFlagsTrajectory,
                           const SendTrajectory::StateVectorTrajectory &stateVectorTrajectory,
                           const SendTrajectory::JointTorquesTrajectory &jointTorquesTrajectory);


private:

    void doneCallback(const actionlib::SimpleClientGoalState& state,
                      const free_gait_msgs::ExecuteStepsResult& result);
    void toGoalMessage(const SendTrajectory::TimeTrajectory &timeTrajectory,
                       const SendTrajectory::ContactFlagsTrajectory &contactFlagsTrajectory,
                       const SendTrajectory::StateVectorTrajectory &stateVectorTrajectory,
                       const SendTrajectory::JointTorquesTrajectory &jointTorquesTrajectory,
                       free_gait_msgs::ExecuteStepsGoal &goal);
    bool isEqualContact(const ContactFlags &c0, const ContactFlags &c1);

  ros::NodeHandle& nodeHandle_;
  //! Action client.
  free_gait::FreeGaitActionClient actionClient_;
  //! Free Gait tools.
  free_gait::AdapterRos adapterRos_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;
  free_gait::StepRosConverter rosConverter_;
  free_gait::StepFrameConverter frameConverter_;
};

}; 

