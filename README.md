# DDDMR P2P Move Base

<p align='center'>
    <img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/p2p_move_base/bt_vs_fsm.png" width="500" height="480"/>
</p>

This repo is the finite state machine for [ddd_navigation](https://github.com/dddmobilerobot/dddmr_navigation) that can control a mobile robot to move from one pose to another pose in 3D space.
The p2p_move_base is different from the move_base in following aspects:
- The robot will rotate in place to the rough heading and then move out, which is similar to [Nav2_Rotation_Shim_Controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_rotation_shim_controller)
- The robot will wait for n seconds (waiting_patience) when the prune plan is being blocked, after n seconds if the obstacles remain, it will compute a new global plan to avoid the obstacles.
- Recovery behaviors are implemented as action servers, therefore, when a goal is being cancelled, it will first cancels recovery behavior (Move base does not allow user to interrupt recovery).
- Recovery behaviors are plugin-based see: [dddme_local_planner](https://github.com/dddmobilerobot/dddmr_local_planner), allow user to customized recovery behaviors.
  - [ ] TODO: make recovery behaviors configurable in yaml file. Current version only use rotate in place as recovery behavior.



## Run The Demo
### 1. Create docker image
The package runs in the docker, so we need to build the image first. We support both x64 (tested in intel NUC) and arm64 (tested in nvidia jetson jpack6).
```
cd ~
git clone https://github.com/dddmobilerobot/dddmr_navigation.git
cd ~/dddmr_navigation && git submodule init && git submodule update
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```
### 2. Run demo
#### Create a docker container
> [!NOTE]
> The following command will create an interactive docker container using the image we built. The we can launch the demo in the container.
```
cd ~/dddmr_navigation/dddmr_docker && ./run_demo.bash
```
#### Launch p2p move base
```
cd ~/dddmr_navigation && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch p2p_move_base p2p_move_base_bag.launch
```
#### Play bag file in the container
We need another terminal to play the bag file. Open another terminal and run following command to get into the container:
```
docker exec -it dddmr_ros2_dev bash
```
Once you are in the container, run:
```
cd ~/dddmr_navigation && source install/setup.bash
cd ~/dddmr_bags && ros2 bag play benanli_detention_basin_localization
```
#### Use Plugin on Rviz2
Use 3D Pose Estimate to provide initial pose and use 3D Goal Pose to prove a goal.
<p align='center'>
    <img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/p2p_move_base/p2p_move_base_annotated.png" width="720" height="420"/>
</p>
