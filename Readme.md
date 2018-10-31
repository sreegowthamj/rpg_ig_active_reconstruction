Our fork of the original repository - https://github.com/sreegowthamj/rpg_ig_active_reconstruction

*Please find our modifications to the original  code in the the above repo.*

#### Dependencies
If you are using ROS, most dependencies should already be installed. Packages were tested on Ubuntu 14.04 under ROS Indigo and Ubuntu 16.04 with ROS Kinetic. 

* **ig_active_reconstruction:** Eigen
* **ig_active_reconstruction_octomap:** Boost, PCL 1.7+, Eigen, Octomap
* **ig_active_reconstruction_ros:** -
*  **example/flying_gazebo_stereo_cam:** Gazebo, stereo_image_proc


#### Installation for ROS Kinetic/Ubuntu 16.04
To install all of these dependencies as system packages, run:
```
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control \
                     ros-kinetic-stereo-image-proc ros-kinetic-octomap libboost-all-dev \
                     libpcl-dev libeigen3-dev
```

All packages are written to be compiled using *catkin* or *catkin_tools*, just clone this repository to your catkin workspace:
```
cd catkin_ws/src
git clone https://github.com/uzh-rpg/rpg_ig_active_reconstruction.git
```
And compile:
```
catkin_make (or catkin build)
```

### Running the demo - 
To use the gazebo model of the bunny, run the below command -

```
export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/rpg_ig_active_reconstruction/example/flying_gazebo_stereo_cam/model:$GAZEBO_MODEL_PATH
```

If everything compiled, run the following in four different terminals to start the reconstruction procedure:
* **roslaunch flying_gazebo_stereo_cam robot_interface.launch**  
Launches Gazebo and loads the bunny (if you put it in the models folder), starts a viewspace model and a robot interface ROS node offering their services to other ig_active_reconstruction components
* **roslaunch flying_gazebo_stereo_cam flying_gazebo_stereo_cam.launch**
Launches the node that spawns the stereo camera (You should see two small boxes pointing at the origin) in Gazebo. This needs to be started after robot_interface.
* **roslaunch ig_active_reconstruction_octomap octomap_world_representation.launch**  
Launches a world representation ROS node using Octomap as a container and offering information gain calculation capabilities.
* **roslaunch ig_active_reconstruction_ros	basic_view_planner.launch**  
Launches a basic view planner node with a simple command line user interface that allows you to start, pause and stop the procedure.  **To start the demo, press 'g' and then 'Enter' in this terminal.**


### Plotting the graph

To plot the graph available in Fig. 4 of the Literature review, you will need to execute the above commands and wait till the execution gets completed. Once the execution is completed, you will need to go to ``` ~/.ros/ ``` folder and copy the ``` rabbit_weighted_gain.csv ``` file to ``` rpg_ig_active_reconstruction ``` folder. Then, for plotting the graph, you will need to run the command - 
```
	python plot_results.py
```
