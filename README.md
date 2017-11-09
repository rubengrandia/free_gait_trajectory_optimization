# free_gait_trajectory_optimization

#### Summary
This repo containts the prototype for connecting free gait with trajectory optimization.

#### Trajectory optimization
The implemented trajectory optimization is described in:

Pardo, Diego, et al. "Hybrid direct collocation and control in the constraint-consistent subspace for dynamic legged robot locomotion." Robotics Science and Systems. 2017.

#### Dependencies 
(repo | branch)

RSL:
- ANYmal beth simulation: installation as described in the [docs](https://anybotics-anymal-sim.readthedocs-hosted.com/en/latest/index.html)
- [free_gait](https://github.com/rubengrandia/free_gait) | dev/trajectory_optimization 

Note:
For the [rbdl](https://bitbucket.org/leggedrobotics/rbdl) I had to "make install" into /opt/ros/kinetic

TO:
- [c_dynamic_systems](https://bitbucket.org/adrlab/c_dynamical_systems) | free_gait_dev 
- [ds_visual](https://bitbucket.org/adrlab/ds_visual) | free_gait_dev 
- [ds_anymal_visual](https://bitbucket.org/adrlab/ds_anymal_visual) | free_gait_dev 
- [legged_robot_task_dt](https://bitbucket.org/adrlab/legged_robot_task_dt) | free_gait_dev 
- [direct_trajectoryoptimization](https://bitbucket.org/adrlab/direct_trajectoryoptimization) | free_gait_dev 

SNOPT:
- [Snopt7.5](https://bitbucket.org/adrlab/snopt_lib) | LicenseUpgrade
(headers in /usr/local/include/snopt7; libraries in /usr/local/lib/snopt7)

#### Build
catkin build free_gait_trajectory_optimization
(Do not use the Eigen flags -mavx -mfma)

#### Usage
##### Trajectory optimization + rviz visualization
1. roslaunch ds_anymal_visualization anymal.launch
2. rosrun free_gait_trajectory_optimization free_gait_trajectory_optimization_rviz

##### Trajectory optimization + gazebo simulation
1. roslaunch anymal_beth_sim sim.launch
2. Activate the freegait impedance controller
3. Turn of the freegait preview features in the rviz panel (they currently crash the setup)
4. roslaunch free_gait_trajectory_optimization free_gait_trajectory_optimization.launch
