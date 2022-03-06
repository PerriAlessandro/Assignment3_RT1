__Research Track 1  -  Third Assignment__  
================================

This is the third assignment of the "Research Track I" course.
The aim of this project consists in creating a package in which the user will be free to use three different modalities for moving the robot that'll be better explained in the "Description" section:
- __autonomously reach a x,y coordinate inserted by the user__;
- __let the user drive with the keyboard__ ;
- __let the user drive the robot assisting them to avoid collisions__.

__How to run__
----------------------
### Installation
This project is based on the ROS Noetic enviroment, if you havent installed it yet, follow the instructions in this link [noetic/Installation/Ubuntu - ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu).
Once you have installed ROS, you will need to clone this repository into the src/ folder of your workspace:
```
git clone https://www.github.com/PerriAlessandro/final_assignment
```
You will also need to download __xterm__, the __slam_gmapping__ package and the ROS navigation stack:
```
sudo apt install xterm
git clone https://www.github.com/CarmineD8/slam_gmapping
sudo apt-get install ros-noetic-navigation
```
### Launch the simulator
To properly launch the project, you should run three different *.launch* files:
- __simulation_gmapping.launch__, to launch the simulation;
- __move_base.launch__, to manage the movement of the robot;
- __final_assignment.launch__, to launch the three nodes I implemented for the UI and the three modalities.

To avoid launching every time all those files, I created a .launch file to run them using only a command line, so you only need to run this:
```
roslaunch final_assignment run.launch
```
__Description__
----------------------
As said in the intro, the aim of the assignment was creating some ROS nodes to properly manage three different modalities that let the robot move inside the map. 
To do that, I created three Python scripts:

- __user_interface.py__, 
- __goal_reaching.py__
- __teleop.py__

### User Interface
The first one is the User Interface that lets the user switch between the modalities, including the 'idle' one (i.e. when no mode is active). The command is given by a user keyboard input and it is sent to the other nodes using ROS topics.
### Autonomous Driving
The second script, as the name may suggest, implements the 'Autonomous Driving modality'. The user will be asked to insert the 'x' and 'y' coordinates to which the robot will navigate. The whole task is accomplished by a __ROS action__. A 60 seconds timeout is set, so if the request cannot be accomplished the goal will be cancelled. The user can also cancel the goal before the time is over, it is sufficient to return to the 'idle' status by pressing '0' on the UI console.
### Manual Driving
The third script implements both the __Assisted__ and __Not Assisted Driving__.
The script is essentially a revisitation of __teleop_twist_keyboard__ because this one already lets the robot move using keyboard inputs, so the main part on which I worked was related to the Assisted Driving modality.
In a nutshell, this last mode makes a subscription to _/scan_ topic in order to check if a certain direction is free or if there is an obstacle (e.g. a wall). Note that the robot can 'see' through its lasers only within a  +-90 relative degrees range, so it won't be able to avoid an obstacle if it is moving backward.
The user can __quit__ both the modalities by pressing `p` from the __teleop console__, or alternatively by pressing another command from __UI console__.

__Pseudocodes__
----------------------
Here you can find the pseudocodes for the three scripts:

### user_interface.py
```
while true
  waiting for a command
  if command == 0
    switch to 'idle' modality
  elseif command == 1
    switch to 'autonomous driving' modality
  elseif command == 2
    switch to 'manual driving (not assisted)' modality
  elseif command == 3
    switch to 'manual driving (assisted)' modality
  elseif command == 4
    quit the simulation
  else
    the key pressed is wrong

```
### goal_reaching.py
```
while true
  if autonomous driving mode is selected
    if the goal has not been set yet
      ask the user to insert the coordinates
      set the action
      set the timer
    if time has expired
      quit mode
  else
    cancel the goal if it's still pending
```
### teleop.py
```
while true
  if not assisted manual driving mode is selected
    freely command the robot with keyboard
  elseif assisted manual driving mode is selected
    command the robot with keyboard but with filtered commands
  else
    quit the node if it's still running
```
__Possible Improvements__
----------------------






