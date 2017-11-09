//
// Created by ruben on 30.10.17.
//

#include <free_gait_trajectory_optimization/SolveTrajectoryOptimization.hpp>

using namespace DirectTrajectoryOptimization;

struct Phasecontainer {
    int phasecode;
    int num_nodes;
    LRD::ContactConfiguration_t contactconfiguration;
    std::shared_ptr<AnymalProjectedDynamics> pdynamics_dyn, pdynamics_con;  // Don't know why we have separate ones
    std::shared_ptr<DynamicsBase<LRD>> dynamics;
    std::shared_ptr<DerivativesBaseDS<LRD>> derivatives;
    std::shared_ptr<lrt::LRTSetOfConstraints<LRD>> hyqsof;
    std::shared_ptr<lrt::LRTSingleConstraintBase<LRD>> cone, penetration, lastphase;

    Phasecontainer(int p, int n, double mu, double contact_surface_tolerance, double ground_z) :
        phasecode(p), num_nodes(n),
        contactconfiguration(LRD::get_all_contact_configurations()[p]),
        pdynamics_dyn(new AnymalProjectedDynamics(contactconfiguration)),
        pdynamics_con(new AnymalProjectedDynamics(contactconfiguration)),
        dynamics(pdynamics_dyn),
        derivatives(new AnymalDerivatives(contactconfiguration)),
        hyqsof(new lrt::LRTSetOfConstraints<LRD>(pdynamics_con)),
        cone(new lrt::LRTFrictionConeConstraint<LRD>(mu))
    {
      if (phasecode > 0 ){ // No friction cone constraint when all feet in the air
        hyqsof->addConstraint(cone);
      }
      if (phasecode < 15){ // No free leg penetration constraint when in the air
        penetration = std::shared_ptr<lrt::LRTSingleConstraintBase<LRD>>(new lrt::LRTSurfacePenetrationConstraintMinimal<LRD>(contactconfiguration, contact_surface_tolerance, ground_z));
        hyqsof->addConstraint(penetration);
      }
      hyqsof->initialize();
//        std::cout << "Phasecontainer initialized" << std::endl;
    }

    // Constructor for last phase
    Phasecontainer(int p, int n, double mu, double contact_surface_tolerance, double ground_z,
                   const Eigen::VectorXi & index, const Eigen::VectorXd & min_values, const Eigen::VectorXd & max_values) :
        phasecode(p), num_nodes(n),
        contactconfiguration(LRD::get_all_contact_configurations()[p]),
        pdynamics_dyn(new AnymalProjectedDynamics(contactconfiguration)),
        pdynamics_con(new AnymalProjectedDynamics(contactconfiguration)),
        dynamics(pdynamics_dyn),
        derivatives(new AnymalDerivatives(contactconfiguration)),
        hyqsof(new lrt::LRTSetOfConstraints<LRD>(pdynamics_con)),
        cone(new lrt::LRTFrictionConeConstraint<LRD>(mu)),
        lastphase(new lrt::LRTLastPhaseConstraint<LRD>(index, max_values, min_values))
    {
      if (phasecode > 0 ){ // No friction cone constraint when all feet in the air
        hyqsof->addConstraint(cone);
      }
      if (phasecode < 15){ // No free leg penetration constraint when in the air
        penetration = std::shared_ptr<lrt::LRTSingleConstraintBase<LRD>>(new lrt::LRTSurfacePenetrationConstraintMinimal<LRD>(contactconfiguration, contact_surface_tolerance, ground_z));
        hyqsof->addConstraint(penetration);
      }
      hyqsof->addConstraint(lastphase);
      hyqsof->initialize();
//        std::cout << "Final Phasecontainer initialized" << std::endl;
    }

};

struct Interphasecontainer {
    std::shared_ptr<BaseClass::InterPhaseBase<LRD>> interPhase;
    std::shared_ptr<BaseClass::GuardBase<LRD>> guard;
    std::shared_ptr<AnymalProjectedDynamics> pdynamics0, pdynamics1;  // Need new dynamics because impacts needs dynamics

    Interphasecontainer(Phasecontainer &p0, Phasecontainer &p1,
                        double mu, double maxZ, double guard_tolerance, double ground_z) :
        pdynamics0(new AnymalProjectedDynamics(p0.contactconfiguration)),
        pdynamics1(new AnymalProjectedDynamics(p1.contactconfiguration))
    {
      if ((p0.phasecode == p1.phasecode) or (p0.phasecode == 15) or (p1.phasecode == 0)){
        interPhase = std::shared_ptr<BaseClass::InterPhaseBase<LRD>>(
            new BaseClass::InterPhaseBase<LRD>(true));
        guard = std::shared_ptr<BaseClass::GuardBase<LRD>>(
            new BaseClass::GuardBase<LRD>(true));
      } else {
        interPhase = std::shared_ptr<BaseClass::InterPhaseBase<LRD>>(
            new lrt::LRTImpactDynamicsInterPhaseImpulseCone<LRD>(p0.contactconfiguration, p1.contactconfiguration, pdynamics1, mu, maxZ));
        guard = std::shared_ptr<BaseClass::GuardBase<LRD>> (
            new lrt::LRTContactGuardMinimal<LRD>(pdynamics1, p0.contactconfiguration, p1.contactconfiguration, guard_tolerance, ground_z));
      }
      interPhase->initialize();
      guard->initialize();
//        std::cout << "interPhasecontainer initialized" << std::endl;
    }
};

void
linearInitialTrajectory(double dt, int number_of_nodes, const LRD::state_vector_t &x0, const LRD::state_vector_t &xf,
                        const std::vector<Phasecontainer> &phases, LRD::state_vector_array_t &x,
                        LRD::control_vector_array_t &u, Eigen::VectorXd &h) {
  int num_points = number_of_nodes - phases.size();  // Each new phase creates overlapping nodes
  h = dt * Eigen::VectorXd::Ones(num_points);

  LRD::state_vector_t dx = 1.0/num_points*(xf-x0);
  LRD::state_vector_t curr_x = x0;
  LRD::control_vector_t u_inv_dyn;

  x.clear();
  u.clear();
  for (auto p : phases){
    for (int n=0; n<p.num_nodes; ++n){
      if (n>0) { curr_x += dx; };  // Don't increment the state on overlapping points
      p.pdynamics_dyn->updateState(curr_x);
      p.pdynamics_dyn->InverseDynamicsControl(LRD::GeneralizedCoordinates_t::Zero(), u_inv_dyn);
      x.push_back(curr_x);
      u.push_back(u_inv_dyn);
    }
  }
};

void perturbTrajectory(LRD::state_vector_array_t &x){
  // Pertubation ranges
  LRD::state_vector_t dx_range;
  dx_range.setZero();
  dx_range.segment(0, 3).setConstant(0.01);
  dx_range.segment(3, 3).setConstant(0.01);
  dx_range.segment(6, 12).setConstant(0.05);

  bool first = true;
  for (auto& x_k : x){
    if (first) {
      first = false;
      continue;
    }
    x_k += dx_range.cwiseProduct(LRD::state_vector_t::Random());
  }

};

void h_to_time(const std::vector<int> &phaseId, const Eigen::VectorXd &h, Eigen::VectorXd &t){
  int node = 0;
  t.resize(phaseId.size());
  t(0) = 0.0;
  for(int k = 1; k < phaseId.size(); ++k) {
    if(phaseId[k] == phaseId[k-1]) {
      t(k) = t(k-1) + h(node);
      node++;
    } else {
      t(k) = t(k-1);
    }
  }
};

bool solveTrajectoryOptimization(const std::vector<int> &contactcodes,
                                 const Eigen::VectorXd &base0,
                                 const Eigen::VectorXd &joints0,
                                 double zGround,
                                 const Eigen::VectorXd &base_goal,
                                 LRD::state_vector_array_t &x_, LRD::control_vector_array_t &u_,
                                 std::vector<double> &t_, std::vector<int> &c_,
                                 bool visualize_ds_rviz) {
  // ======================== Problem definitions ========================
  Eigen::VectorXi nodeSplit(contactcodes.size());
  nodeSplit = 4 * Eigen::VectorXi::Ones(contactcodes.size());

  double mu = 0.5;
  double maxZ = 70;
  double contact_surface_tolerance = 5.0e-3;
  double guard_tolerance = 5.0e-3;

  double dt0 = 0.10;
  double Tmin = 0.01;
  double Tmax = 1e20;

  int dircol_method = 1;
  bool method_verbose = false;
  bool solver_verbose = false;
  bool even_time_increments = false;
  bool feasible_only = false;
  bool automatic_differentiation = false;
  int derivatives_verification = -1;
  bool use_print_file = false;
  std::string solver_output_file = "solver_file.dto";
  // ========================================================================

  // Problem geometry: Initial, final state, ground
  LRD::state_vector_t initial_state, final_state;
  AnymalKinematics lrkin;
  lrkin.GetCanonicalPoseState(initial_state);
  lrkin.GetCanonicalPoseState(final_state);
  initial_state.head(6) = base0;
  initial_state.segment(6, 12) = joints0;
  final_state.head(6) = base_goal;
  final_state.segment(6, 12) = joints0;
  double ground_z = zGround;

  // Final state bounds
  Eigen::VectorXi index(LRD::kTotalDof);
  index.setLinSpaced(LRD::kTotalDof, 0, LRD::kTotalDof);
  Eigen::VectorXd min_values(LRD::kTotalDof), max_values(LRD::kTotalDof);
  max_values.segment(0, 6) = final_state.segment(0, 6).array() + 0.01;
  min_values.segment(0, 6) = final_state.segment(0, 6).array() - 0.01;
  max_values.segment(6, 12) = final_state.segment(6, 12).array() + 0.1;
  min_values.segment(6, 12) = final_state.segment(6, 12).array() - 0.1;

  // Create phases
  int number_of_nodes = nodeSplit.sum();
  int num_phases = contactcodes.size();
  std::vector<Phasecontainer> phases;
  std::vector<std::shared_ptr<DynamicsBase<LRD>>> dynamicsVector;
  std::vector<std::shared_ptr<DerivativesBaseDS<LRD>>> derivativesVector;
  std::vector<std::shared_ptr<BaseClass::ConstraintsBase<LRD>>> constraintsVector;
  for (int i = 0; i < num_phases; ++i) {
    if (i == (num_phases - 1)) { // last phase needs special care
      phases.emplace_back(Phasecontainer(contactcodes[i], nodeSplit[i], mu, contact_surface_tolerance, ground_z,
                                         index, min_values, max_values));
    } else {
      phases.emplace_back(Phasecontainer(contactcodes[i], nodeSplit[i], mu, contact_surface_tolerance, ground_z));
    }
    dynamicsVector.push_back(phases.back().dynamics);
    derivativesVector.push_back(phases.back().derivatives);
    constraintsVector.push_back(phases.back().hyqsof);
  }

  // Create interphases
  std::vector<Interphasecontainer> interphases;
  std::vector<std::shared_ptr<BaseClass::InterPhaseBase<LRD>>> interPhaseVector;
  std::vector<std::shared_ptr<BaseClass::GuardBase<LRD>>> guardsVector;
  for (int i = 0; i < (num_phases - 1); ++i) {
    interphases.emplace_back(Interphasecontainer(phases[i], phases[i + 1], mu, maxZ, guard_tolerance, ground_z));
    interPhaseVector.push_back(interphases.back().interPhase);
    guardsVector.push_back(interphases.back().guard);
  }

  // Initial (x, u, h)
  Eigen::VectorXd h_initial;
  LRD::state_vector_array_t y_initial;
  LRD::control_vector_array_t u_initial;
  linearInitialTrajectory(dt0, number_of_nodes, initial_state, final_state, phases, y_initial, u_initial, h_initial);
//  perturbTrajectory(y_initial);

  // State and input bounds
  LRD::control_vector_t u_min, u_max;
  LRD::GeneralizedCoordinates_t q_min, q_max, qd_min, qd_max;
  LRD::state_vector_t x_min, x_max;
  SetBounds(q_min, q_max, qd_min, qd_max, u_min, u_max);
  lrkin.StateVectorFromGeneralizedCoordinates(q_min, qd_min, x_min);
  lrkin.StateVectorFromGeneralizedCoordinates(q_max, qd_max, x_max);

  // Setup problem
  std::shared_ptr<BaseClass::CostFunctionBase<LRD> > costFunction(new Costs::ZeroCostBolza<LRD>());
  Eigen::Vector2d duration;
  duration << Tmin, Tmax;
  auto nlp_solver = static_cast<DirectTrajectoryOptimization::Solvers::Solver>(0);
  auto method_type = static_cast<DirectTrajectoryOptimization::Methods::Method>(0);

  std::unique_ptr<DirectTrajectoryOptimization::DirectTrajectoryOptimizationProblem<LRD> > multipleDynamicsDTOP =
      DirectTrajectoryOptimization::DirectTrajectoryOptimizationProblem<LRD>::CreateWithMultipleDynamics(
          dynamicsVector, derivativesVector, costFunction, constraintsVector, duration,
          number_of_nodes, nlp_solver, method_type, solver_verbose, method_verbose, feasible_only
      );

  // Problem values
  multipleDynamicsDTOP->SetInitialState(initial_state);
  multipleDynamicsDTOP->SetInitialControl(u_initial.front());
  multipleDynamicsDTOP->_method->SetControlBounds(u_min, u_max);
  multipleDynamicsDTOP->_method->SetStateBounds(x_min, x_max);
  multipleDynamicsDTOP->_method->SetInterPhaseFunctions(guardsVector, interPhaseVector);
  multipleDynamicsDTOP->_method->SetNodesPerPhase(nodeSplit);

  // Problem settings
  multipleDynamicsDTOP->SetDircolMethod(dircol_method);
  multipleDynamicsDTOP->SetEvenTimeIncrements(even_time_increments);
  multipleDynamicsDTOP->_solver->SetVerifyLevel(derivatives_verification);
  multipleDynamicsDTOP->_solver->UsePrintFile(use_print_file);
  multipleDynamicsDTOP->InitializeSolver("pfl", solver_output_file, "", automatic_differentiation);

  // Solve the problem
  multipleDynamicsDTOP->_solver->InitializeDecisionVariablesFromTrajectory(y_initial, u_initial, h_initial);
  bool solver_flag = false;
  multipleDynamicsDTOP->Solve(solver_flag);

  //Getting the solution
  LRD::state_vector_array_t y_trajectory;
  LRD::control_vector_array_t u_trajectory;
  Eigen::VectorXd h_trajectory;
  std::vector<int> phaseId;
  multipleDynamicsDTOP->GetDTOSolution(y_trajectory, u_trajectory, h_trajectory, phaseId);

  // Show Solution
  std::cout << "h_index : " << std::endl << h_trajectory.transpose() << std::endl;
  std::vector<int> hyqPhaseCode;
  for (auto pid : phaseId) { hyqPhaseCode.push_back(phases[pid].phasecode); };
  std::cout << std::endl << "... end of App." << std::endl;

  Eigen::VectorXd time_t(y_trajectory.size());
  h_to_time(phaseId, h_trajectory, time_t);

  if (not visualize_ds_rviz) {
    x_ = y_trajectory;
    u_ = u_trajectory;
    c_ = hyqPhaseCode;

    t_.clear();
    for (int k = 0; k < time_t.size(); k++) {
      t_.push_back(time_t[k]);
    };
  } else {
    //  Interpolate result
    std::vector<int> p_;
    Eigen::VectorXd t;
    std::shared_ptr<DTTrajectorySpliner<LRD>> spliner(new DTTrajectorySpliner<LRD>(phases.front().pdynamics_dyn));
    bool hybrid_ok = spliner->setHybridTrajectory(y_trajectory, u_trajectory, time_t, phaseId, hyqPhaseCode);
    spliner->interpolateFromHybird();
    bool is_ready = spliner->getInterpolatedTrajectory(x_, u_, t, p_, c_);
    t_.clear();
    for (int k = 0; k < t.size(); k++) {
      t_.push_back(t[k]);
    };

    // Play visualization
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "playbackAnymalDT");
    RobotConfigVis<LRD> pfl_vis("ds_anymal");
    while (getchar() == '\n') {
      for (const auto &x: x_) {
        pfl_vis.visualize_pose(x);
        pfl_vis.sleep();
      }
    }

  }




};

void SetBounds(LRD::GeneralizedCoordinates_t & q_min, LRD::GeneralizedCoordinates_t & q_max,
               LRD::GeneralizedCoordinates_t & qd_min, LRD::GeneralizedCoordinates_t & qd_max,
               LRD::control_vector_t & u_min, LRD::control_vector_t & u_max) {
  //ToDo: Create a Class for this that returns different type of
  //bounds (aggresive, conservative, standard) and that also contains
  //the values of the actual limits of the robot

  /* Actuation limits */
  u_min.setConstant(-35);
  u_max.setConstant(35);

  /* joints positions */
  /* joints velocities*/
  for(int a = 0 ; a < 4 ; ++a){
    int nj = LRD::kBaseDof + 3*a;
    q_min[nj] = -0.698132; //-40 deg
    q_max[nj] =  0.698132;
    q_min[nj+1]  = -2.5;  //-143
    q_max[nj+1]  =  2.5;  // 143
    q_min[nj+2]  = -2.5;  //-143
    q_max[nj+2]  =  2.5;  // 143
  }
  for(int i = 0 ; i < LRD::kTotalJoints ; ++i) {
    int k=i+LRD::kBaseDof;
    qd_min[k] = -12;
    qd_max[k] =  12;
  }

  /* base limits */
  q_min[0] =  -M_PI/4;
  q_min[1] =  -M_PI/4;
  q_min[2] =  -2*M_PI;
  q_min[3] =  -5.0; //x
  q_min[4] =  -5.0; //y
  q_min[5] =  -0.5; //z

  q_max[0] = M_PI/4;
  q_max[1] = M_PI/4;
  q_max[2] = 2*M_PI;
  q_max[3] = 5.0;  //x
  q_max[4] = 5.0;  //y
  q_max[5] = 2.0;  //z

  qd_min[0] = -10; //alpha
  qd_min[1] = -10; //beta
  qd_min[2] = -10; //gamma
  qd_min[3] = -10; //x
  qd_min[4] = -10; //y
  qd_min[5] = -10; //z

  qd_max[0] = 10;  //alpha
  qd_max[1] = 10;  //beta
  qd_max[2] = 10;  //gamma
  qd_max[3] = 10;  //x
  qd_max[4] = 10;  //y
  qd_max[5] = 10;  //z
};