//
// Created by ruben on 30.10.17.
//

#pragma once

#include <memory>
#include <iostream>
#include <vector>

#include <Eigen/Dense>

// Base::Dynamics
#include <dynamical_systems/base/LeggedRobotDynamics.hpp>  //*
#include <dynamical_systems/systems/anymal/AnymalDimensions.hpp>



bool userAccepts();

std::vector<int> getGaitFromUser();

void getBaseGoal(Eigen::VectorXd &base_goal);

std::vector<Eigen::Matrix<bool, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<bool, 4, 1>>>
contactCodesToFlags(const std::vector<int> &cCodes);