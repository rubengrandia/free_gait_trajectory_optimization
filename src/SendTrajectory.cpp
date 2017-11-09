#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "free_gait_trajectory_optimization/SendTrajectory.hpp"

using namespace free_gait;

namespace free_gait_trajectory_optimization {

    SendTrajectory::SendTrajectory(ros::NodeHandle& nodeHandle)
            : nodeHandle_(nodeHandle),
              actionClient_(nodeHandle),
              adapterRos_(nodeHandle),
              tfListener_(tfBuffer_),
              rosConverter_(adapterRos_.getAdapter()),
              frameConverter_(tfBuffer_)
    {


      // Register callback.
      actionClient_.registerCallback(nullptr, nullptr,
                                     std::bind(&SendTrajectory::doneCallback, this, std::placeholders::_1, std::placeholders::_2));

      adapterRos_.subscribeToRobotState();

      ros::Time timeout = ros::Time::now() + ros::Duration(10.0);
      ros::Duration sleepDuration(0.1);
      while (ros::Time::now() < timeout) {
        ros::spinOnce();
        if (adapterRos_.isReady()) {
          break;
        }
        sleepDuration.sleep();
      }

      ROS_INFO("free_gait_trajectory_optimization initialized");
    }

    SendTrajectory::~SendTrajectory() {}

//    bool SendTrajectory::start(){
//        // Get Initial state
//        adapterRos_.updateAdapterWithState();
//        auto x0_base = adapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame();
//        std::vector<JointPositionsLeg> qj_vector;
//        for (const auto limb : adapterRos_.getAdapter().getLimbs()){
//            qj_vector.emplace_back(adapterRos_.getAdapter().getJointPositionsForLimb(limb));
//        }
//        GeneralizedCoordinates q0;
//        q0 << 0, 0, 0, x0_base.toImplementation(),
//                qj_vector[0].toImplementation(),
//                qj_vector[1].toImplementation(),
//                qj_vector[2].toImplementation(),
//                qj_vector[3].toImplementation();
//
//        // Generate motion.
//        ROS_INFO("Generating sample motion.");
//        ContactFlags c0, c1, c2;
//        c0 << true, true, true, true;
//        c1 << true, true, false, true;
//        c2 << true, true, true, false;
//
//        TimeTrajectory t_traj;
//        ContactFlagsTrajectory c_traj;
//        GeneralizedCoordinatesTrajectory q_traj;
//        GeneralizedCoordinates dq1, dq2, dq3;
//        dq1 << 0, 0, 0,     0.05,  0.00, -0.05,     0, 0, 0,    0, 0, 0,    0.0,  0.0,  0.0,    0.0,  0.0,  0.0;
//        dq2 << 0, 0, 0,     0.00, -0.00,  0.00,     0, 0, 0,    0, 0, 0,    0.0, -0.0, +0.5,    0.0,  0.0,  0.0;
//
//        // Standing
//        t_traj.push_back(0.0); c_traj.push_back(c0); q_traj.push_back(q0);
//        t_traj.push_back(1.0); c_traj.push_back(c0); q_traj.push_back(q0 + dq1);
//        // Lift leg
//        t_traj.push_back(1.0); c_traj.push_back(c1); q_traj.push_back(q0 + dq1);
//        t_traj.push_back(2.0); c_traj.push_back(c1); q_traj.push_back(q0 + dq1 + dq2);
//        t_traj.push_back(3.0); c_traj.push_back(c1); q_traj.push_back(q0 + dq1);
//        // Standing
//        t_traj.push_back(3.0); c_traj.push_back(c0); q_traj.push_back(q0 + dq1);
//        t_traj.push_back(4.0); c_traj.push_back(c0); q_traj.push_back(q0);
//
//        JointTorquesTrajectory tau_traj;
//        JointTorques tau;
//        tau.setZero();
//        for (int i=0; i<q_traj.size(); i++) tau_traj.push_back(tau);
//
//        // Add step to Goal
//        free_gait_msgs::ExecuteStepsGoal goal;
//        toGoalMessage(t_traj, c_traj, q_traj, tau_traj, goal);
//        actionClient_.sendGoal(goal);
//        return true;
//    }

    Eigen::Matrix<double, 6, 1> SendTrajectory::getBase0(){
        // Get Initial state
        adapterRos_.updateAdapterWithState();
        auto position = adapterRos_.getAdapter().getPositionWorldToBaseInWorldFrame();
        auto quatOrienation = adapterRos_.getAdapter().getOrientationBaseToWorld();
        kindr::EulerAnglesXyz<double> eulerOrientation(quatOrienation);
        Eigen::Matrix<double, 6, 1> base0;
        base0.head(3) = eulerOrientation.toImplementation();
        base0.tail(3) = position.toImplementation();
        return base0;
    };

    Eigen::Matrix<double, 12, 1> SendTrajectory::getJoints0(){
        auto j0 = adapterRos_.getAdapter().getAllJointPositions();
        return j0.toImplementation();
    };

    double SendTrajectory::getGroundZ(){
        Position p_foot;
        double z_min = 1e20;
        for (const auto limb : adapterRos_.getAdapter().getLimbs()){
            p_foot = adapterRos_.getAdapter().getPositionWorldToFootInWorldFrame(limb);
            z_min = std::min(z_min, p_foot.z());
        }
    };

    bool SendTrajectory::executeTrajectory(const SendTrajectory::TimeTrajectory &timeTrajectory,
                                           const SendTrajectory::ContactFlagsTrajectory &contactFlagsTrajectory,
                                           const SendTrajectory::StateVectorTrajectory &stateVectorTrajectory,
                                           const SendTrajectory::JointTorquesTrajectory &jointTorquesTrajectory){
        ROS_INFO("executeTrajectory()");

        // Add step to Goal
        free_gait_msgs::ExecuteStepsGoal goal;
        toGoalMessage(timeTrajectory, contactFlagsTrajectory, stateVectorTrajectory, jointTorquesTrajectory, goal);
        actionClient_.sendGoal(goal);
        return true;

    };

    void SendTrajectory::doneCallback(const actionlib::SimpleClientGoalState& state,
                                      const free_gait_msgs::ExecuteStepsResult& result)
    {
        ROS_INFO("Finished, shutting down.");
        nodeHandle_.shutdown();
    }

    void SendTrajectory::toGoalMessage(const SendTrajectory::TimeTrajectory &timeTrajectory,
                                       const SendTrajectory::ContactFlagsTrajectory &contactFlagsTrajectory,
                                       const SendTrajectory::StateVectorTrajectory &stateVectorTrajectory,
                                       const SendTrajectory::JointTorquesTrajectory &jointTorquesTrajectory,
                                       free_gait_msgs::ExecuteStepsGoal &goal) {
        // Message declarations
        free_gait_msgs::BaseTrajectory baseTrajectoryMessage;
        std::vector<free_gait_msgs::JointTrajectory> jointMessages;
        free_gait_msgs::Step stepMessage;

        goal.steps.clear();
        ContactFlags current_contact = contactFlagsTrajectory.front();
        double t0ThisStep = timeTrajectory.front();
        for (size_t k=0; k<timeTrajectory.size(); k++){

            // Prepare step
            if ((k==0) or (not isEqualContact(contactFlagsTrajectory[k], current_contact))){
                current_contact = contactFlagsTrajectory[k];
                t0ThisStep = timeTrajectory[k];

                // Message declarations
                baseTrajectoryMessage = free_gait_msgs::BaseTrajectory();
                jointMessages = std::vector<free_gait_msgs::JointTrajectory>();
                stepMessage = free_gait_msgs::Step();

                // High level information: Frames, Legnames
                baseTrajectoryMessage.trajectory.header.frame_id = adapterRos_.getAdapter().getWorldFrameId();
                for (const auto limb : adapterRos_.getAdapter().getLimbs()){
                    free_gait_msgs::JointTrajectory jointMessage;
                    jointMessage.name = getLimbStringFromLimbEnum(limb);
                    jointMessages.push_back(jointMessage);
                }
            }

            // Fill step
            // Base
            trajectory_msgs::MultiDOFJointTrajectoryPoint point;
            geometry_msgs::Transform T;
            T.rotation = tf::createQuaternionMsgFromRollPitchYaw(stateVectorTrajectory[k][0],
                                                                 stateVectorTrajectory[k][1],
                                                                 stateVectorTrajectory[k][2]);
            T.translation.x = stateVectorTrajectory[k][3];
            T.translation.y = stateVectorTrajectory[k][4];
            T.translation.z = stateVectorTrajectory[k][5];
            point.transforms.push_back(T);
            point.time_from_start = ros::Duration(timeTrajectory[k]-t0ThisStep);
            baseTrajectoryMessage.trajectory.joint_names = std::vector<std::string>{"base"};
            baseTrajectoryMessage.trajectory.points.push_back(point);

            // Legs
            int legNum = 0;
            for (auto &jointMessage : jointMessages) {
                // Joint reference
                jointMessage.trajectory.joint_names = std::vector<std::string>{"HAA", "HFE", "KFE"};  // Important to specify
                trajectory_msgs::JointTrajectoryPoint jointPoint;
                jointPoint.positions.push_back(stateVectorTrajectory[k][6+3*legNum+0]);  // HAA
                jointPoint.positions.push_back(stateVectorTrajectory[k][6+3*legNum+1]);  // HFE
                jointPoint.positions.push_back(stateVectorTrajectory[k][6+3*legNum+2]);  // KFE
//                jointPoint.effort.push_back(jointTorquesTrajectory[k][3*legNum+0]);  // HAA
//                jointPoint.effort.push_back(jointTorquesTrajectory[k][3*legNum+1]);  // HFE
//                jointPoint.effort.push_back(jointTorquesTrajectory[k][3*legNum+2]);  // KFE
                jointPoint.time_from_start = ros::Duration(timeTrajectory[k]-t0ThisStep);
                jointMessage.trajectory.points.push_back(jointPoint);

                jointMessage.ignore_contact = static_cast<unsigned char>(true);
                legNum++;
            }

            // Append step
            if ((k==(timeTrajectory.size()-1)) or (not isEqualContact(contactFlagsTrajectory[k+1], current_contact))){
                stepMessage.base_trajectory.push_back(baseTrajectoryMessage);
                for (int i=0; i<4; i++){
                    if (not contactFlagsTrajectory[k][i]) // Only adding the legs not in contact
                        stepMessage.joint_trajectory.push_back(jointMessages[i]);
                }

                goal.steps.push_back(stepMessage);
            }
        }
    }

    bool SendTrajectory::isEqualContact(const ContactFlags &c0, const ContactFlags &c1){
        for (int i=0; i<4; ++i){
            if (c0[i] != c1[i]) return false;
        }
        return true;
    };
};
