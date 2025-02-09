# Via-Point Based Stochastic Trajectory Optimization for Signal Temporal Logic
This repo is a [ROS](https://www.ros.org/) (1) implementation of the algorithm in [this paper](https://www.youtube.com/watch?v=dQw4w9WgXcQ) by Ilyes et al.

## Intallation 
Install the code using 
`git clone https://github.com/rolizzlefoshizzle/stlVpStoROS`

#### Dependencies 
This repo relies on a couple other dependencies

First, make sure to create your vitual environment if you're using one. Make sure to install 
- matplotlib
- jax cpu (using `pip install --upgrade "jax[cpu]"`)

**Important**: Be sure to set the path to the virtual environment in the launch file

Next install [STLRom](https://github.com/decyphir/STLRom)(within the stlVpSto directory) and follow the build instructions:

`git clone https://github.com/decyphir/STLRom` 

As of November 2023, the build instructions are

`cd STLRom/build`

`cmake ..`

`make`

Finally, install [Vp-Sto (the jax-dev branch)](https://github.com/JuJankowski/vp-sto/tree/jax-dev)(within the stlVpSto directory) and follow the build instructions:

`git clone https://github.com/JuJankowski/vp-sto/tree/jax-dev`

As of November 2023, the build instructions are

`cd vp-sto`

`pip install .`

## Usage
In one terminal, launch the system using  
`roslaunch ro-to-go FIXME.launch --screen`

Then, in the second terminal, publish the formula you'd like to execute:
`rostopic pub /userCommand std_msgs/String "data: '<the formula>.stl'"`

Formulas are located in the scripts/formulas directory. Specify which formula to plan for in the command above

The following parameters can be changed in the launch file:
- time resolution for evaluating STL robustness of MPC rollouts
- time delay between local planner calls (unused)
- time delay between global planner calls (unused)
- period of control input durations
- time resolution of truthing system
- variance of gaussian noise applied to predicate states
- initial robot position and velocity 
- initial predicate positions
