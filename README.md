# droonituuled

https://dev.px4.io/en/setup/dev_env_linux.html

Do steps Development Toolchain, Pixhawk/NuttX, jMAVSim/Gazebo Simulation, Gazebo with ROS



## .bashrc Setup for ROS / GAZEBO / PX4 / ARDUPILOT
Either run these every time manually or add them to .bashrc. Assumes default paths, change according to your setup.

```bash
source /opt/ros/kinetic/setup.bash
source $HOME/catkin_ws/devel/setup.bash

export PATH=/usr/lib/ccache:$PATH
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
export PATH=$PATH:$HOME/jsbsim/src
export PATH=$PATH:$HOME/gcc-arm-none-eabi-7-2017-q4-major/bin
export PATH=$PATH:/opt/gcc-arm-none-eabi-4_9-2015q3/bin

export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/gazebo_models
export GAZEBO_MODEL_PATH=$HOME/catkin_ws/src/droonituuled/sim:$GAZEBO_MODEL_PATH

export GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/gazebo_worlds:$GAZEBO_RESOURCE_PATH
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-7:$GAZEBO_RESOURCE_PATH
export GAZEBO_RESOURCE_PATH=$HOME/catkin_ws/src/droonituuled/sim/worlds:$GAZEBO_RESOURCE_PATH
```

## Running Simulation
Start scripts are provided in the `start` directory. To start the whole simulation, run the following commands in separate terminal windows (note, the dots in the beginning are important):

```
. start_sim_vehicle
. start_gazebo
. start_mavros_apm
. start_robotex_cam
. start_robotex_flight
```

After `sim_vehicle`, wait until it has mostly started up. After `gazebo`, wait until it has opened up the main window. If gazebo or mavros are started too early, they will throw an error.

By default the flight controller will wait for the vehicle to be armed before starting the control loop. A timeout will occur in 10 seconds. The vehicle can be armed by running the following command in the flight controller simulation window (the first window where sim_vehicle was started):

```
arm throttle
```
