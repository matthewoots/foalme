# Fast Obstacle Avoidance on Locally Mapped Environment (FOALME)

## Installation
`foalme` serves a complete trajectory and optimization server with avoidance on ROS using the `bs-traj-server` which acts as a wrapper to pass data into the module. In addition, `simple_quad_simulator` consist of a quadcopter simulator, which can spawn several agents to test the performance of this trajectory planner package.

### Dependencies
 
- bspline_trajectory_ros (https://github.com/matthewoots/bspline_trajectory_ros.git)
    - bs-traj-server (https://github.com/matthewoots/bs-traj-server.git)
    - LBFGSpp (https://github.com/matthewoots/LBFGSpp.git)
    - libbspline (https://github.com/matthewoots/libbspline.git)
    - librrtserver (https://github.com/matthewoots/librrtserver.git)
- simple_quad_simulator (https://github.com/matthewoots/simple_quad_simulator.git)


### Setup
For starters who do not know how to use ROS and do not have a prior workspace, just run the commands below and all will be fine

Please have an SSH-key for your machine by using `ssh-keygen` and finding your key in `id_rsa.pub`
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone git@github.com:matthewoots/foalme.git --recurse-submodules
cd ..
catkin build
# Or you can compile only the needed packages
# catkin build bs_trajectory_ros quad user_server
```