//
// Created by ruben on 30.10.17.
//

#pragma once

#include <memory>
#include <iostream>

// Base::Optimization
#include <optimization/constraints/ConstraintsBase.hpp>
#include <optimization/costs/CostFunctionBase.hpp>
#include <optimization/constraints/InterPhaseBaseConstraint.hpp>
#include <optimization/constraints/GuardBase.hpp>

// Base::Dynamics
#include <dynamical_systems/base/LeggedRobotDynamics.hpp>  //*
#include <dynamical_systems/base/DerivativesNumDiffBase.hpp>

//HyQ Headers
#include <dynamical_systems/systems/anymal/AnymalKinematics.hpp>
#include <dynamical_systems/systems/anymal/AnymalDimensions.hpp>
#include <dynamical_systems/systems/anymal/AnymalProjectedDynamics.hpp>
#include <dynamical_systems/systems/anymal/AnymalDerivatives.hpp>
#include <dynamical_systems/systems/anymal/AnymalState.hpp>

// DTO Package
#include <direct_trajectory_optimization_problem.hpp>
#include <optimization/costs/ZeroCostBolza.hpp>

// LRT Package
#include <legged_robot_task_dt/constraints/lrt_singleConstraintBase.hpp>

// HyQ DT Tasks
#include <legged_robot_task_dt/constraints/lrt_setOfConstraints.hpp>
#include <legged_robot_task_dt/constraints/lrt_frictionConeConstraint.hpp>
#include <legged_robot_task_dt/constraints/lrt_surfacePenetrationConstraint.hpp>
#include <legged_robot_task_dt/constraints/lrt_lastPhaseConstraint.hpp>

// Hybrid Control constraints
#include <legged_robot_task_dt/constraints/lrt_contact_guard.hpp>
#include <legged_robot_task_dt/constraints/lrt_liftLegConstraint.hpp>
#include <legged_robot_task_dt/constraints/lrt_impactDynamicsInterPhaseImpulseCone.hpp>

// Interpolate Results
#include <legged_robot_task_dt/tools/DTTrajectorySpliner.hpp>

// Ros visual
#include <ds_visual_base/RobotConfigVis.hpp>

// Analysis tool
#include <dynamical_systems/tools/HybridLRTrajectory.hpp>

// namespaces
using LRD = robotDimensions;

void SetBounds(LRD::GeneralizedCoordinates_t & q_min, LRD::GeneralizedCoordinates_t & q_max,
               LRD::GeneralizedCoordinates_t & qd_min, LRD::GeneralizedCoordinates_t & qd_max,
               LRD::control_vector_t & u_min, LRD::control_vector_t & u_max);

struct Phasecontainer;
struct Interphasecontainer;

void linearInitialTrajectory(double dt, int number_of_nodes, const LRD::state_vector_t &x0, const LRD::state_vector_t &xf,
                             const std::vector<Phasecontainer> &phases,
                             LRD::state_vector_array_t &x, LRD::control_vector_array_t &u, Eigen::VectorXd &h);

void perturbTrajectory(LRD::state_vector_array_t &x);

void h_to_time(const std::vector<int> &phaseId, const Eigen::VectorXd &h, Eigen::VectorXd &t);

bool solveTrajectoryOptimization(const std::vector<int> &contactcodes,
                                 const Eigen::VectorXd &base0,
                                 const Eigen::VectorXd &joints0,
                                 double zGround,
                                 const Eigen::VectorXd &base_goal,
                                 LRD::state_vector_array_t &x_, LRD::control_vector_array_t &u_,
                                 std::vector<double> &t_, std::vector<int> &c_,
                                 bool visualize_ds_rviz);


