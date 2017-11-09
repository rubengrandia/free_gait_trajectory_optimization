//
// Created by ruben on 30.10.17.
//

#include <free_gait_trajectory_optimization/ReceiveTrajectory.hpp>

using LRD = robotDimensions;

bool userAccepts(){
  std::cout << "accept y/n:" << std::endl;
  char ch;
  std::cin >> ch;
  std::cin.clear();
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  return ch == 'y';
}

std::vector<int> getGaitFromUser(){
  // Print list of options
  std::cout << "Contact numbers:" << std::endl;
  std::cout << "\t[LF]\t[RF]\t[LH]\t[RH]" << std::endl;
  auto contactlist = LRD::get_all_contact_configurations();
  for (int c=0; c<contactlist.size(); ++c) {
    std::cout << c << ": ";
    for (int leg = 0; leg<4; ++leg) {
      std::cout << '\t' << (int)contactlist[c][leg];
    }
    std::cout << std::endl;
  }

  // Get user input
  std::cout << "Build your gait:" << std::endl;
  std::vector<int> contactcodes;
  std::string line;
  getline(std::cin, line);
  std::istringstream iss(line);
  std::string field;
  while ( getline(iss, field, ',') )
    contactcodes.emplace_back(std::stoi(field));

  // Print gait
  std::cout << "Gait set to:" << std::endl;
  for (const auto & code : contactcodes){
    std::cout << code << ", ";
  }
  std::cout << std::endl;

  if (userAccepts()){
    return contactcodes;
  }
  else {
    return getGaitFromUser();
  }
}

void getBaseGoal(Eigen::VectorXd &base_goal){
  base_goal.setZero();
  // Get user input
  std::cout << "set base goal (roll, pitch, yaw, x, y, z):" << std::endl;
  std::cout << "{roll, pitch} are absolute, {yaw, x, y, z} are relative" << std::endl;
  std::string line;
  getline(std::cin, line);
  std::istringstream iss(line);
  std::string field;
  int i = 0;
  while ( getline(iss, field, ',') && i<6 ){
    base_goal(i) = std::stof(field);
    i++;
  }

  // Print goal
  std::cout << "Goal set to:" << std::endl;
  std::cout << base_goal.transpose() << std::endl;
  if (!userAccepts()){
    getBaseGoal(base_goal);
  }
}

std::vector<Eigen::Matrix<bool, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<bool, 4, 1>>>
contactCodesToFlags(const std::vector<int> &cCodes) {
  std::vector<Eigen::Matrix<bool, 4, 1>, Eigen::aligned_allocator<Eigen::Matrix<bool, 4, 1>>> cFlags;

  auto contactlist = LRD::get_all_contact_configurations();
  Eigen::Matrix<bool, 4, 1> cFlags_i;
  for (const auto& c_i : cCodes){
    for (int leg=0; leg<4; leg++){
      cFlags_i[leg] = contactlist[c_i][leg];
    }
    cFlags.push_back(cFlags_i);
  }

  return cFlags;
}