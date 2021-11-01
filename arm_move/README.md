Arm Motion Planning
===
aka Spooky Automated Reacher-Grabber by <i>Nicole Baptist</i>

**Overview**
<p>This second package is based on the `/px100` robot. It is programmed to interact with its environment by:<br>

1. Avoiding set obstacles in Rviz and in real life
2. Grabbing an object, moving it, and placing it in a new location.
<br>

This package also focuses on simulations; however, rather than connecting Rviz and Gazebo, this portion of the assignment utilizes the MoveIt Motion Planning Framework.</p>

---

*Mover*
<p> The main code pulls from

`Step.srv` and `waypoints.yaml` to load and store waypoints for a calculated trajectory (follow service)<br>

The `arm_move` package depends on the following files:

Python files (nodes):<br>
1. mover (nodes) - pubs (move_group/display_planned_path), srvs(reset, step, follow), param(@yaml: x, y, z, xo, yo, zo, wo, gripd)
2. crashlanding.py (test) - tests whether the robot produces the correct error at collision

Configuration files:<br>
1. waypoints.yaml (config) - please see above
2. Step.srv (srv) - states the inputs and outputs of the step service

Launch files:<br>
1. arm.launch (launch) - the main launch file which launches the mover node
2. arm.test (launch) - the testing launch file which launches the mover node and crashlanding.py

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
<p>sys<br>
copy<br>
rospy<br>
moveit_commander<br>
moveit_msgs.msg<br>
geometry_msgs.msg<br>
math<br>
std_msgs.msg<br>
moveit_commander.conversions<br>
arm_move.srv<br>
std_srvs.srv<br>
moveit_msgs.msg<br>

unittest<br>
rostest<br>
</p>

<p></p>
<p>Having sourced your workspace, the basic package can be launched using the following commands:</p>

`roslaunch arm_move arm.launch`

<p>Note that while there aren't set arguments to change these options, the model of the robot, language used (cpp or py), environment (fake node or real world), etc. can all be changed via the launch file.</p>

<P>The services can then be called in a new terminal:<br>

`rosservice call /px100/follow`

For this service, simply edit the waypoints.yaml to contain your desired waypoints.

`rosservice call /px100/step <args>`

Where `<args>` represents the entered Step. HINT: after entering `rosservice launch /px100/step`, keep pressing `tab` to see how the input arguments should be entered.

DO NOT FORGET TO SOURCE THE WORKSPACE IN THE NEW TERMINAL!

Note: `/px100/reset` is automatically called at the beginning of every run.

<p> - - - </p>

<p>To run tests, simply call the following command from your source:<br>

`catkin_make run_tests`<br>

which will return to you either a SUCCESS (if the robot produces the correct -1 error message at collision) or a FAIL.</p>

---

**Options**

<p>(Modifiable settings)</p>

<p>As mentioned earlier, there are various parameters that are changeable. Please see arm.launch file for full list of arguments.<br>

The main change is that the user can choose whether to use the fake node (simulation only) or the real node (real world execution with simulation) depending on the resources available to them. The default is to use real; therefore, the simulation version is called as follows:<br>

`roslaunch arm_move arm.launch use_fake:=true  use_actual:=false`<br>

The real version can be launched using the default call without any additional argument setting. Please note that only one of the two (fake or real) can be used at once.<br>

The rest of the parameters can be changed in a similar fashion. The editable arguments are listed in the arm.launch file.
</p>

---

**Media**

<p></p>

Step service: ![](https://raw.githubusercontent.com/ME495-EmbeddedSystems/homework-3-nbaptist16/master/arm_move/images/stepsrv.png?token=ACEM63VX7TLVR62AW4KZCOC7VUQ4G)

(Example outputs of step service: first trial returns -1 (failed trajectory due to collision) and the second returns +1 (successfully planned trajectory), where the robot can be seen moved from the home position.)

<p></p>

Test one, two, three: ![Anything but that!](https://raw.githubusercontent.com/ME495-EmbeddedSystems/homework-3-nbaptist16/master/arm_move/images/testhax.png?token=ACEM63XST52WWKEMXIGTV6C7VUQ7C)

(Example output of a "successful" test. It's really just hacked since I quit and it didn't run any tests, but I figured you'll be able to see it fail naturally and might want to see the satisfying SUCCESS to restore your faith in humanity.)

<p></p>

Arm in action!: https://youtu.be/jfoHQk0TEr8

(I like to Moveit Movit. You like to Moveit Movit.)

TODO: Add background audio


---

**Acknowledgments**

<p>Huge thanks and shoutout to Mr. Chamzas and Ms. Smith for all of their support and guidance along the way.<br>
Thank you also to our team mom for reminding me that if I don't finish this, then "the sleepless nights were for nothing :)"</p>