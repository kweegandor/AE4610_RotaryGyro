# Quadcopter

## Introduction

### Objective

This experiment will introduce students to the basic principles for controlling the flight of a small quadcopter, the CrazyFlie 2.1. Weighting only 27 grams and having 9.2 cm of length and width, is a “nanoquad” which has rapidly become one of the preferred platforms for quadcopter research.

In Part A, students will adjust the gains of a PID controller to follow autonomously a waypoint maneuver. In Part B, the students will interact with the obstacle avoidance capabilities of the quadcopter.

### Equipment Required

* [ ] Crazyflie 2.1 quadcopter with flow deck and multi-ranger deck
* [ ] Crazyradio 2.4 GHz
* [ ] Crazyflie Python Client

### Background

We define the body frame of a quadrotor with the X, Y and Z axes shown below in Figure 8.1, with roll angle about the x-axis, pitch angle about the y-axis, and yaw angle about the z-axis. There are different ways of attaching a reference frame to a quadcopter, Figure 8.1 shows the convention which is consistent with Crazyflie quadrotor’s source code.&#x20;

![Figure 8.1. Quadrotor Body Frame Axes](<../.gitbook/assets/image (96).png>)

A quadrotor has four sources of thrust and torque with its four rotors. As seen below in Figure 8.2, adjacent rotors rotate in opposite directions. In Figure 8.2, if the output of motors 1 & 2 are increased and motors 3 & 4 are decreased, the quadrotor can maintain an equal amount of total thrust while creating a roll moment.  If the output of motors 1 & 4 are increased and motors 2 & 3 are decreased, the quadrotor can maintain an equal amount of total thrust while creating a pitch moment. If the output of motors 1 & 3 are increased and the output of motors 2 & 4 are decreased, the quadcopter will develop a yawing moment. This arrangement of the rotors enables full control of a quadrotor in 3D space.

![Figure 8.2 Quadrotor Propeller Direction. Note: Does NOT represent actual configuration.](<../.gitbook/assets/image (105).png>)

![Figure 8.3 Cone Shape of the Crazyflie Height Sensing.](<../.gitbook/assets/image (46).png>)

#### Flow Deck

The Flow Deck consists of a laser sensor that measures distance to the ground and a low-resolution camera called optical flow sensor which measures movement parallel to the ground. Together these sensors allow the CrazyFlie to interpret its movement in horizontal and vertical directions.&#x20;

Note that the laser distance sensor will return the minimum distance to any object within its detection range as depicted in Figure 8.3. During the quadcopter’s operation, it may mistake an object (like a box lying on the ground) as the actual ground, try to maintain a certain distance over the false ground, and therefore suddenly and unexpectedly accelerate upwards. In order to avoid that the flight area must be cleared out before the flight tests.&#x20;

#### Multi-ranger Deck

The Multi-ranger Deck uses five laser sensors to measure the distance in the front/back/left/right/up direction. This enables the CrazyFlie to detect proximity to objects up to four meters away. This allows CrazyFlie to avoid obstacles.

## Part A: Position PID Controller Tuning

Four PID controllers work together to facilitate position control of the CrazyFlie. The outermost position controller, the controller we will be tuning in this part of the lab, takes the target position as reference, and calculates a target linear velocity to be sent to the _**velocity controller**_. The velocity controller takes target linear velocity as reference, and calculates a target roll/pitch angle to be sent to the _**attitude controller**_, which uses the target roll/pitch angle as reference and calculate target roll/pitch rate (angular velocity). Finally, the innermost _**rate controller**_ takes the target roll/pitch rate as reference, and directly controls the motors to generate moment in X/Y axis to achieve desired angular rates. In this lab, we will vary the PID gains for the outmost position controller and treat the inner controllers as a black box. We shall use the following PID controller gains as a starting point.&#x20;

#### Table 2. Initial PID Gains

| Initial PID Gains | X-Axis | Y-Axis | Z-Axis |
| ----------------- | ------ | ------ | ------ |
| $$k_p$$           | 2.0    | 2.0    | 2.0    |
| $$k_d$$           | 0.2    | 0.2    | 0.2    |

Recall that the current altitude is measured with the laser distance sensor, which reports the distance to the closest obstacle within the cone shaped detection range (Figure 8.3). Therefore, if the CrazyFlie is too close to a wall or a surface, the height reading may not be accurate. Also, note that the area of the cone increases with altitude.

To see the effect the PID gains have on vehicle performance, we will command the quadrotor to follow a set of waypoints, each time with different PID gains.

We shall use lab8\_part1\_pid.py to control the CrazyFlie quadrotor. The program allows the user to set the PID gains for the position and velocity controllers.  Upon execution, the program instructs the quadrotor to take off, follow the selected set of waypoints, and land, while recording the quadrotor’s position throughout the experiment.&#x20;

There are two sections of the program you may want to change:

**1) Waypoints**

![](<../.gitbook/assets/image (101).png>)

The waypoints are given in reference to the world coordinate frame, which is aligned with the vehicle body frame when the program is executed. X is aligned with forward direction, Y with the leftward direction, and Z points upward.&#x20;

You may change target waypoints to your liking, but make sure:

* [ ] You use the same waypoints for each experiment
* [ ] The waypoints are safe and achievable
* [ ] The first waypoint has (x,y) coordinates of (0,0)

**2) PID Gains**

![](<../.gitbook/assets/image (68).png>)

This is the part of the code where you can change the PID gains.&#x20;

You should read the code to make sure you understand what to expect. After the program is finished, a log would be generated with the name lab8\_log.npy. Make sure you change the name of this file before running the next experiment, otherwise it would be overwritten.

To view the log, run lab8\_part1\_plot.py.   If you have changed the name of the log, edit the following line in lab8\_part1\_plot.py to read the correct log.

![](<../.gitbook/assets/image (56).png>)

You should see the following figures (3D plot appears as the first figure is closed):

![](<../.gitbook/assets/image (35).png>)

![Figure 8.4 Example plots generated by lab8\_part1\_plot.py](<../.gitbook/assets/image (84).png>)

Using these graphs, you will need to compare control performance with different gains in your lab report. Use the following gains for your experiments:

* Gain Set #1: Nominal Gain in Table 2
* Gain Set #2: Higher $$k_p$$ ****&#x20;
* Gain Set #3: Higher $$k_d$$&#x20;

#### Table 3 Gain Set #2

| Initial PID Gains | X-Axis | Y-Axis | Z-Axis |
| ----------------- | ------ | ------ | ------ |
| $$k_p$$           | 3.0    | 3.0    | 3.0    |
| $$k_d$$           | 0.2    | 0.2    | 0.2    |

#### Table 4 Gain Set #3

| Initial PID Gains | X-Axis | Y-Axis | Z-Axis |
| ----------------- | ------ | ------ | ------ |
| $$k_p$$           | 2.0    | 2.0    | 2.0    |
| $$k_d$$           | 0.5    | 0.5    | 0.5    |

### Procedure

1.  Change PID gains to gain set #1


2.  Run **python3 lab8\_part1\_plot.py**


3.  Change log name, view log


4.  Change PID gains to gain set #2


5.  Run **python3 lab8\_part1\_plot.py**


6.  Change log name, view log


7.  Change PID gains to gain set #3


8.  Run **python3 lab8\_part1\_plot.py**


9.  Change log name, view log


10. Compare three logs, discuss the difference, and explain how each gain affects the outcome

## Part B: Obstacle Avoidance Using a Distance Sensor

This part of the experiment will examine the obstacle avoidance for a small quadcopter. Obstacle avoidance is becoming increasingly important in small drone applications. This lab will use the quadrotor’s proximity sensor to detect objects in front of the drone. The concept can be extended to detect objects on all sides.&#x20;

The addition of obstacle avoidance capabilities is essential for indoor quadrotor flight. While the piloting of the quadrotor is the inner-loop control, the obstacle avoidance can be considered the outer-loop control, overriding the pilot’s inputs to prevent a collision. &#x20;

The proximity sensor must be used in conjunction with an appropriate obstacle avoidance algorithm that is relevant to the flight conditions and scenario. In the case of this lab, the quadrotor will be not be allowed to come within 40 cm of the object in front of it. A minimum distance of 40 cm should therefore always be maintained. The quadrotor will set a velocity in the direction opposite to the incoming object. As another example, an algorithm could also be created to maintain proximity to a nearby object, i.e., maintaining a distance of 40 cm away from a target. This type of algorithm would be useful in swarm or teammate following applications.

For this experiment, we will try to mimic static obstacles and incoming obstacles. To mimic static obstacles, we place an obstacle within the safety margin and see how the vehicle reacts to it. For an incoming obstacle, we move the obstacle closer to the vehicle and see how the vehicle reacts.

The file **lab8\_part2\_push.py.** contains the following two functions:

1\) `is_close`

![](<../.gitbook/assets/image (60).png>)

This function determines whether there is an obstacle within the given minimum distance.

2\) `main`

![](<../.gitbook/assets/image (42).png>)

Review carefully the code listed above, for discussion in the final report. To land the CrazyFlie, slowly put your hand on top of the vehicle.

### Procedure

1.  Place the CrazyFlie in the center of the floor mat and away from any other objects.


2.  Open **lab8\_part2\_push.py**


3.  Make sure the MIN\_DISTANCE is set to 0.4 meters.


4.  Make sure the VELOCITY is set to 0.5 m/s.


5.  Run  **lab8\_part2\_push.py:**

    **python3 lab8\_part2\_push.py**


6.  The CrazyFlie should reach a steady hover before interfering. Place your hand or a flat object in front of the push sensor and observe as the CrazyFlie maintains a minimum distance of 0.2 m from the incoming object. Make observations regarding the return to steady state hover.&#x20;


7.  Try to imitate a static obstacle with your hand or object, commanding the CrazyFlie to maintain the minimum distance.&#x20;


8.  Save the position data.


9.  Try to imitate an incoming obstacle with an object moving slower than 0.5m/s.


10. Save the position data.


11. Try to imitate an incoming obstacle with an object moving faster than 0.5m/s. Do not hit the vehicle!


12. Save the position data.


13. Close everything. **DO NOT SAVE THE CHANGES.**

## Analysis & Lab Report

### Part A

*   Plot all data sets and discuss the differences. You can directly import csv file into Matlab for plotting. Your plots should look like example plots shown in Figure 8.4.


*   Discuss and explain your observations of the hover performance of the Crazyflie flight very close to the ground.


* Discuss the impact of increasing the proportional and derivative gains on the waypoint controller performance.&#x20;

### Part B

*   Plot all datasets and include a brief discussion on your experimental observations and the plotted results.


*   Discuss the observed vehicle response to stationary versus moving obstacles.


* Discuss any important changes to the obstacle avoidance algorithm in **lab8\_part2\_push.py** you would suggest for it to be useful for flying through a field of stationary and moving obstacles.

