Differential Drive Bot
===
aka Poor Man's Batmobile by <i>Nicole Baptist</i>

**Overview**
<p>This first package models a two-wheeled robot to traverse the terrains of Rviz and Gazebo. Its two deliverables are:<br>

1. To follow a rectangular path (please see follow_rect node)
2. To perform flips by switching directions quickly (please see flipper node)
<br>
This package focuses on simulations, as it depends heavily on software such as Rviz and Gazebo; it also emphasizes the importance in creating an accurate virtual model (with the proper parameters/physics), as such is necessary to spawn and interact with objects in Gazebo.</p>
---

*Follow Rect*
<p>This portion of the code pulls from 

`ddrive.urdf.xacro`, `ddrive.gazebo.xacro`, `ddrive.world`, and `ddrive.yaml`
spawns the differential drive robot in Gazebo to follow a rectangular path.<br>
(Please note that this trajectory only executes once.)<br>

*Flipper*
<p>This section of code also utilizes the above mentioned configuration codes. Rather than turning, this robot changes velocities quickly; this results in the robot flipping back and forth along a linear path.<br>

This package depends on the following files:

Python files (nodes):<br>
1. follow_rect (nodes) - pubs(cmdvel, path), srvs(pause, resume), brod(brodometry), pram(width, height, period, ~pub_freq)<br>
2. flipper (nodes) - a

Configuration files:<br>
1. ddrive.yaml (config) - contains parameters of robot (inc. chassis, wheel, and caster dimensions, weights<br>
2. ddrive.urdf.xacro (urdf) - robot model description<br>
3. ddrive.gazebo.xacro (urdf) - initializes gazebo plugins/mechanics for Gazebo<br>
4. hw3bot.rviz (config) - loads robot model into Rviz<br>
5. ddrive.world (worlds) - initializes Gazebo environment (inc. Jersey barriers, cardboard boxes, and Cessna plane<br>

Launch files:<br>
1. ddrive.launch<br>
2. ddrive_rviz.launch<br>
      a. note that the gazebo launch (ddrive.launch) must be used first in order to obtain the published odom in rviz.

Note: they also include CMakeLists.txt, package.xml,...the usual
</p>
<p></p>

---

**Instructions**
<p><!>After cloning repository from

[github](https://github.com/ME495-EmbeddedSystems/homework-3-nbaptist16)
, don't forget to catkin_make, source your workspace, all that jazz<!></p>
<p></p>
<p>The following libraries are required:</p>
<p>(rospy)<br>
geometry_msgs<br>
math<br>
nav_msgs<br>
rosservice<br>
std_srvs<br>
tf2_ros<br>
yaml<br>
 </p>

<p></p>
<p>Having sourced your workspace, the basic package can be launched using the following commands:</p>

*Follow_Rect (part1)*

`roslaunch diff_drive ddrive.laumch`

<p>Note that the default is to launch Gazebo in a paused state. In order to run the simulation, the user can press the play button on the bottom left of the Gazebo GUI window. This way, the simulation should be unobstructed and the whole trajectory viewable.</p>

<p> - - - </p>

*Flipper (part 2)*

`roslaunch diff_drive ddrive.launch drectangle:=False`

<p>The default for this package is to run the follow_rect node, so the user will need to disable the follow_rect node by appending the last argument to the launch command.</p>

<p> - - - </p>

*Rviz (part 0)*

`roslaunch diff_drive ddrive.launch drvize:=True`
`roslaunch diff_drive ddrive_rviz.launch mov:=True`

<p>To view the robot simultaneously moving in Rviz and Gazebo, these commands can be called in the order stated. (Trust me, do it this way or things get weird.)<br>
The user can also choose to view a stationary version of the robot (WITHOUT ddrive.launch running) by entering the following command:<br>

`roslaunch diff_drive ddrive_rviz.launch`

I know. It's confusing. But hey, it works!
</p>

---

**Options**

<p>(Modifiable settings)</p>

<p>There are several arguments used in the launch files to control the robot's response; the main changable arguments are:<br>

1. *paused* (default = true) - launches Gazebo in a paused state; can be changed by pressing the play button in the GUI.
2. *drectangle* (default = true) - runs follow_rect node; can be changed by adding 'drectangle:=False' when calling the launch file (will instead run the flipper node)<br>

Other parameters such as the world loaded, the model of the robot, and the initial spawn point of the robot can also be changed by manually editing the launch file (please see ddrive.launch comments for instruction.)
</p>

---

**Media**

<p></p>

PenguinBot 0.0.1: https://www.youtube.com/watch?v=SDr5WJGrRT8&feature=youtu.be

(Testing the flipper node takes an unexpected turn)

<p></p>

Diff_Drive: https://www.youtube.com/watch?v=t4UcbvjCN7k&feature=youtu.be<br>

Includes: Gazebo and Rviz trials (rectangle only)