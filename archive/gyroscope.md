---
description: >-
  This manual documents the modelling of a gyroscope for which an angular
  position controller using gimbal torque is developed. Finally, the controller
  is implemented on the hardware and evaluated.
---

# Gyroscope

## Objective

The purpose of this experiment is to design a controller that maintains the direction of a gyroscope under base excitation. The controller can also be used to rotate the gyro platform to a desired orientation using the gyroscopic principle.&#x20;

![Image credit: NASA](<../.gitbook/assets/image (77).png>)

## Equipment Required

*   [ ] Gyroscope (Electromechanical plant)


*   [ ] Input/Output Electronic ECP Model 750 box


*   [ ] DSP based Controller/Data Acquisition Board which is already installed in a PC


* [ ] ECP software

## Part A: Modeling

### The Gyroscope Model

This experiment will be performed using the Model 750 Control Gyroscope.  The system, shown in Fig. 3.1, consists of an electromechanical plant and a full complement of control hardware and software.  The user interface to the system is via an easy-to-use PC-based environment that supports a broad range of controller specification, trajectory generation, data acquisition and plotting features.  A picture of the setup including the DSP card and input/output electronics is shown in Fig. 3.2.

![Figure 3.1: The Model 750 Control Moment Gyroscope](<../.gitbook/assets/image (24).png>)

### Description of the Gyroscope

The gyroscope consists of a high inertia brass rotor suspended in an assembly with four angular degrees of freedom, as seen in Fig. 3.3.  The rotor spin torque is provided by a rare Earth magnet type DC motor (Motor #1), whose angular position is measured by an optical encoder (Encoder #1) with a resolution of 2,000 counts per revolution.  The motor drives the rotor through a 10:3 gear reduction ratio, which amplifies both the torque and encoder resolution by this factor.  The first transverse gimbal assembly (body C) is driven by another rare Earth magnet motor (Motor #2) to effect motion about Axis #2.  The motor drives a 6.1:1 capstan to amplify the torque between the adjoining bodies C and B.  A 1000 line encoder (Encoder #2) with 4x interpolation is mounted on the motor to provide feedback of the relative position between bodies C and B with resolution of 24,400 counts per revolution.

![Figure 3.2:  The Gyroscope Control System Setup](<../.gitbook/assets/image (64).png>)

The subsequent gimbal assembly, body B, rotates with respect to body A about Axis #3.  There is no active torque applied about this axis.  A brake, which is actuated via a toggle switch on the Controller box, may be used to lock the relative position between bodies A and B and hence reduce the system's degrees of freedom.  The relative angle between A and B is measured by Encoder #3 with a resolution of 16,000 counts per revolution.  Finally, body A rotates without actively applied torque relative to the base frame (inertial ground) along Axis #4.  The Axis #4 brake is controlled similar to the Axis #3 brake and the relative angle between body A and the base frame is measured by an optical encoder (Encoder #4) with a resolution of 16,000 counts per revolution.

![Figure 3.3: Control Moment Gyroscope Apparatus](<../.gitbook/assets/image (52).png>)

Inertial switches or “g-switches” are installed on bodies A, B, and C to sense any over-speed condition in the gimbal assemblies.  The switches are set to actuate at 2.1 g’s.  For Axis #2, limit switches and mechanical stops are provided at the safe limit of travel.  When any of these normally closed switches sense a high angular rate condition, they open and thereby cause a relay to turn off power to the controller box.  When this power is lost, the fail-safe brakes (power-on-to-release type) at Axes #3 and #4 engage.  Also, upon loss of power, the windings of Motor #1 and #2 have shorted, thereby causing electromechanical damping.  Thus, all axes are actively slowed and stopped whenever an over-speed or over-travel condition is detected.

Metal slip rings are included at each gimbal axis to allow continuous angular motion.  These low noise, low friction, slip rings pass all electrical signals including those of the motors, encoder, g-switches, limit switches, and brakes to the control box.

### Mathematical Model of the Gyroscope

In the configuration used in the experiment, gimbal Axis #3 is locked ($$\omega_3$$ = 0), so that bodies A and B become one and the same (see Fig. 3.4).  The resulting plant is useful for demonstrations of gyroscopic torque action where the position and rate, $$q_4$$ and $$\omega_4$$, may be controlled by rotating gimbal #2 while the rotor is spinning.  Figure 3.5 shows the coordinate system used for this model.

![Figure 3.4: Gimbal # 3 Locked, All Others Free](<../.gitbook/assets/image (38).png>)

### Basic Equations

Figure 3.5 shows the definitions of the coordinate axes used for the model development. Let $$I_x, J_x, K_x$$    (x = A, B, C and D) denote the scalar moments of inertia respectively in the bodies A, B, C, and D. Referring to Fig. 3.5, the inertia matrices $$I^A, I^B, I^C, I^D$$  are given as follows:

$$
I^A = \begin{bmatrix}
I_A &0 &0 \\
0 &J_A &0 \\
0 &0 &K_A
\end{bmatrix}, I^B = \begin{bmatrix}
I_B &0 &0 \\
0 &J_B &0 \\
0 &0 &K_B
\end{bmatrix}, I^C = \begin{bmatrix}
I_C &0 &0 \\
0 &J_C &0 \\
0 &0 &K_C
\end{bmatrix}, I^D = \begin{bmatrix}
I_D &0 &0 \\
0 &J_D &0 \\
0 &0 &K_D
\end{bmatrix}
$$

The basic gyroscope equation can be written as:

$$
\vec{T} = \frac{d}{dt}\vec{H} = \dot{\vec{H}} + \vec{\omega}_F  \times \vec{H} \tag{3.1}
$$

where $$\vec{T}$$ is the vector of applied torque. Referring to Fig. 3.5, the components of applied torque can be written as

$$
\vec{T} = \begin{bmatrix} 
T_2 \\
T_1 \\
0 
\end{bmatrix} \tag{3.2}
$$

where $$T_1$$ and $$T_2$$ are the applied torques about Axes #1 and #2, respectively (see Fig. 3.5).

![Figure 3.5: Coordinate Frame Definitions](<../.gitbook/assets/image (81).png>)

The components of the angular momentum vector $$\vec{H}$$ and the angular velocity vector $$\vec{\omega}_F$$ can be written as

$$
\vec{H} = \begin{bmatrix} 
(I_D + I_C) \omega_2 \\
J_D \omega_1 \\
(I_D + K_A + K_B + K_C) \omega_4 
\end{bmatrix} \tag{3.3}
$$

$$
\vec{\omega}_F = \begin{bmatrix} 
\omega_2 \\
0 \\
\omega_4 
\end{bmatrix} \tag{3.4}
$$

Substituting equations 3.2, 3.3 and 3.4 into Eq. 3.1 results in the following set of nonlinear equations:

$$
T_1 = J_D\dot{\omega}_1 + (I_C + I_D)\omega_2\omega_4 \space \boldsymbol{-} \space (I_D + K_A + K_B + K_C)\omega_2\omega_4 \\
T_2 = (I_C + I_D)\dot{\omega}_2 \space \boldsymbol{-} \space J_D\omega_1\omega_4 \\
0 =  (I_D + K_A + K_B + K_C)\dot{\omega}_4 + J_D\omega_1\omega_2 \tag{3.5}
$$

### Linearized Model

In this section, we proceed to linearize the nonlinear gyroscopic equations (Eq. 3.5) by imposing certain assumptions.

#### Assumptions

1.  The angle of rotation of the rotor disk D about the gimbal axis 2 is small.


2. $$\displaystyle{\frac{\omega_2}{\Omega}, \frac{\omega_4}{\Omega}}$$ are small, where $$\Omega =\omega_1$$ is the spin speed of the rotor disk D (rotational speed of the rotor) which is given as 400 RPM or 41.89 rad/sec.

Upon linearization ($$\omega_2\omega_4 \approx 0, \omega_1\omega_4 \approx \Omega\omega_4, \omega_1\omega_2 \approx \Omega \omega_2$$), we further obtain the following linearized equations:&#x20;

$$
T_1 \space \boldsymbol{-} \space J_D\dot{\omega}_1 = 0 \tag{3.6}
$$

$$
T_2 + J_D\Omega\omega_4 \space \boldsymbol{-} \space (I_C + I_D)\dot{\omega}_2 = 0 \tag{3.7}
$$

$$
J_D\Omega\omega_2 + (I_D + K_A + K_B + K_C)\dot{\omega}_4 = 0 \tag{3.8}
$$

In these equations, the rotor spin dynamics (Eq. 3.6) are decoupled from those of the second and fourth gimbals (Eqs. 3.7 and 3.8).  Since the rotor speed may be independently controlled, the salient dynamics become those involving motion at the gimbal locations. The gimbal angular positions $$q_2$$ and $$q_4$$ are related to $$\omega_2$$ and $$\omega_4$$ as

$$
\dot{q_2} = \omega_2 \space \space \text{and} \space \space \dot{q_4} = \omega_4 \tag{3.9}
$$

Taking the Laplace transform of Eqs. 3.7, 3.8 and 3.9 and eliminating $$\omega_2$$ and $$\omega_4$$ from the resulting equations results in the following transfer functions for $$q_4(s)/T_2(s)$$and $$q_2(s)/T_2(s)$$:

$$
G_4(s) = \frac{q_4(s)}{T_2(s)} = \frac{ \boldsymbol{-}J_D \Omega}{(I_C + I_D)(I_D + K_A + K_B+K_C)s^3 +\Omega ^2 J^2_D s} \tag{3.10}
$$

$$
G_2(s) = \frac{q_2(s)}{T_2(s)} = \frac{(I_D + K_A + K_B + K_C)s}{(I_C + I_D)(I_D+K_A + K_B + K_C)s^3 + \Omega^2J^2_D s} \tag{3.11}
$$

## Part B: Controller Design

### Control System Overview

The objective is to design a controller for regulation and control of the angular position $$q_4$$ using the gimbal torque $$T_2$$ . This mimics the following problem: Given a spacecraft with a momentum gyro, we would like to regulate and control the spacecraft attitude ($$q_4$$) using the gimbal torque ($$T_2$$) of the gyroscope. In this lab, the position of Axis #4, $$q_4$$ , is controlled by torquing the Axis #2 motor.  This is accomplished here with the use of a technique known as successive loop closure.  First, a rate feedback loop around $$\omega_2$$ is closed to damp the nutation mode of the gyroscope.  Subsequently, an outer loop is closed to control $$q_4$$.  The block diagram for this process can be seen in Figure 3.6.

![Figure 3.6: Successive Loop Closure Control Scheme](<../.gitbook/assets/image (57).png>)

The equations defining the numerators and denominators in the block diagram are as follows.

$$
D(s) = (I_C + I_D)(I_D + K_A + K_B + K_C)s^3 + \Omega^2J_D^2s \tag{3.12}
$$

$$
N_2 = k_{e_2}k_{u_2}(I_D + K_A + K_B + K_C)s \tag{3.13}
$$

$$
N_4 = \boldsymbol{-} k_{e4}k_{u2}\Omega J_D \tag{3.14}
$$

Note that the $$q_2$$ and $$q_4$$ shown in Fig. 3.6 represent measurements using encoders at Axes #2 and #4, respectively. The values $$k_{e2}$$ and $$k_{e4}$$ represent the encoder gains for Encoders #2 and #4, respectively.  Finally, $$k_{u2}$$ is a control effort gain and $$\Omega$$ is the rotational speed of the rotor in rad/sec (given above).   The numerical values of various parameters are given in Tables 1 through 3 in the next page.

### System Parameters

#### **Table 1: Gyroscope Inertia Values**

| Body | Inertia Element | Value (kg - m^2) |
| ---- | --------------- | ---------------- |
| A    | $$K_A$$         | 0.067            |
| B    | $$I_B$$         | 0.012            |
|      | $$J_B$$         | 0.012            |
|      | $$K_B$$         | 0.03             |
| C    | $$I_C$$         | 0.0092           |
|      | $$J_C$$         | 0.023            |
|      | $$K_C$$         | 0.022            |
| D    | $$I_D$$         | 0.015            |
|      | $$J_D$$         | 0.027            |

#### **Table 2: Encoder Gains**

| **Axis i** | Output/Rev $$\bar{k}$$ (counts/rev) | Gain $$\bar{k}_{ei} = \bar{k}/(2\pi)$$ (counts/rad) |
| ---------- | ----------------------------------- | --------------------------------------------------- |
| 1          | 6667                                | 1061                                                |
| 2          | 24400                               | 3883                                                |
| 3          | 16000                               | 2547                                                |
| 4          | 16000                               | 2547                                                |

#### **Table 3: Control Effort Gains**

| **Gain**    | Value (N/count) |
| ----------- | --------------- |
| $$k_{u1}$$  | 1.28 E-05       |
| $$k_{u2}$$  | 9.07 E-05       |

{% hint style="info" %}
$$k_{ei} = \bar{k}_{ei} * 32$$ where the number 32 is called the **firmware gain.** The controller firmware multiplies the encoder and commanded position signals internally by 32. This is done for increasing the numerical precision. This multiplication is not performed on the plotted data. The constants used in the transfer functions $$N_2, N_4$$ are $$k_{ei}$$ and **not** $$\bar{k}_{ei}$$ .
{% endhint %}

### Specifications

1.  All closed-loop poles must be in the left half of the complex plane


2.  The peak time in response to a unit step command input must be less than 0.2 sec.&#x20;


3.  The 2% settling time to a unit step command input must be less than 0.5 sec.&#x20;


4. For the outer loop, $$k_p$$ must be less than 6.0 and $$k_d$$ must be less than 0.4.

### Procedure

1.  Select the inner loop derivative gain $$k_v$$ to be equal to 0.08.


2.  Open a new m-file and compute the transfer function $$q_4(s)/r_i(s)$$ using equations (3.12) through (3.14) and Fig. 3.6. Save your m-file.


3.  Use **controlSystemDesigner** or **rltool** in MATLAB (depending on the version) and select appropriate values for $$k_p$$ and $$k_d$$ to meet the given set of specifications. If needed, readjust the $$k_v$$ value. You need to meet the requirements in **controlSystemDesigner/rltool.** However, you may have to change them later in SIMULINK to ensure they work. Save the **controlSystemDesigner/rltool** figures and your gains. Make sure to use the consistent controller architecture in the **controlSystemDesigner/rltool**.


4.  Develop a SIMULINK diagram of the block diagram shown in Fig. 3.6 and save it for use in part C of the experiment. Use two separate transfer functions as shown in Fig 3.6.&#x20;


5.  Obtain the SIMULINK response to a unit step command using the set of gains you have selected. Assess the SIMULINK step response using `stepinfo()` command. If the response does not meet the design requirements, then modify the gains until it does. Save the response, and make sure it proves you meet the requirements.


6.  Individually change each of the three gains {$$k_p, k_d$$,$$k_v$$}  in three separate runs so that only one gain is ever altered at a time.  Reduce $$k_p$$ and $$k_d$$ each by 50% and reduce $$k_v$$ by 25%.  Obtain simulation response to the same unit step command used in the previous step and compare it with the simulation response for the nominal case by graphing them on the same plot. Observe the effect of $$k_p, k_d$$, and $$k_v$$ on the response behavior of the closed-loop system.


7. The TAs will need to see: Gains, **controlSystemDesigner/rltool** plots, SIMULINK diagram, SIMULINK response to a step command (proving peak and settling time requirements are met), and the 3 modified responses.

{% hint style="warning" %}
You must have your work checked out by one of the TAs before leaving the lab to get credit for your work.
{% endhint %}

## Part C: Controller Implementation & Evaluation

{% hint style="danger" %}
### **Safety Briefing**

**Safety Note 1**: In the event of an emergency, control effort should be immediately discontinued by pressing the red “OFF” button on the front of the control box.

**Safety Note 2**: Stay clear of and do not touch any part of the mechanism while the rotor is moving, while a trajectory is being executed, or before the active controller has been safety checked.

**Safety Note 3:** The rotor should never be operated at speeds above 825 RPM.  The user should take precautions to assure that this limit is not exceeded. If this accidentally happens, hit "Abort Control" in the ECP software and re-initiate rotor.

**Safety Note 4:** Never leave the system unattended while the ECP control box is powered on.
{% endhint %}

### Experimental Setup

These steps should be performed primarily by the TA, but the students should follow along to gain a deeper understanding of how the experiment works.

#### Functionality Test

This test ensures if everything is sound mechanically, electrically, and in the software.&#x20;

1. **Hardware checks**
   1. Ensure the rotor cover panel is secure.
   2. Switch on the ECP control box and disengage Axis 3 and Axis 4 brakes.
   3. Ensure all axes can turn freely with no significant friction felt by hand.
   4. Ensure the two Axis 2 limit switches function correctly (**with rotor not turning**) by checking auto-shutoff occurs when manually pressing each of them. Switch the ECP control box back on.
   5. Ensure Axis 3 and Axis 4 inertial switches function correctly (**with rotor not turning**) by aggressively turning the appropriate axis by hand to initiate auto-shutoff. Switch the ECP control box back on.
2. **Software checks**
   1. Launch _E2Usr32 - Model750_, the ECP CMG control software, a shortcut for which can be found on the Desktop. Alternatively, the executable can be found at _C:\Program Files (x86)\ECP Systems\_MV\mv_
   2. If the program doesn't launch, make sure it is being executed in Windows XP SP3 compatibility mode
   3. Go to `Setup > Communications > PMAC 00 - PCIO - Plug and play > Test` and check that the dialog confirms successful PMAC connection. If it does not, the PCI card is likely not seated in the PC correctly. Have the TA try wiggling the ribbon cable in the back of the PC to get a good connection and re-try communication check. If nothing works, have a TA contact the Lab Manager.
   4. If Apparatus is not listed as _"C.M.G. Model 750"_ go to `Utility > Download Controller Personality File` and load _"gyro20.pmc"_ from _C:/Program Files (x86)/ECP Systems\_MV/mv/._
   5. Load Configuration file _"default.cfg"_ from _C:\Program Files (x86)\ECP Systems\_MV\mv._
   6. Go to `Utility > Reset Controller` and check the following values on the main screen have been set:
      1. Encoders 1/2/3/4 Pos: 0 COUNTS
      2. Control Loop Status: OPEN
      3. Motor 1 Status: OK
      4. Motor 2 Status: OK
      5. Servo Time Limit: OK
   7. Ensure the 4 encoders are reading correctly by manually turning each axis and watching the corresponding encoder counts value change on the main screen (you may want to zero the encoders prior to this step to see it more clearly). Note that Encoder 1 is the rotor, which is hard to move by hand since it is enclosed, so aggressively rotate Axis 3 to let the rotor inertia create some encoder position change.&#x20;

#### Orientation setting

1. With the ECP control box powered **on,** all brakes **off**, and flywheel **not turning,** roughly orient the gyroscope according to Figure 3.7 (as viewed from the front such that labels are the correct way up).
2. Apply all brakes and turn off the ECP control box.
3. Ensure Axis 3 is parallel with the platform base by using a spirit level or phone app, making small adjustments by hand (with the brakes on).
4. Ensure Axis 2 is perpendicular to the platform base and Axis 3 by using the spirit level app of a phone app, making small adjustments by hand (with the brakes on).
5. It is not critical where Axis 4 is located since the controller will be doing a relative angle maneuver.

![Figure 3.7: Initial gyroscope orientation as viewed from the front (labels the correct way up) ](<../.gitbook/assets/image (11).png>)

### &#xD;Controller Setup

#### **Configure Data Logging**

1. Go to the `Data` menu and click on _Setup Data Acquisition_.
2. Be sure that the selected variables to log match those below. When finished, hit OK to exit this menu.

![](<../.gitbook/assets/image (22).png>)

#### Initialize Rotor

{% hint style="danger" %}
_After these steps, the rotor will be turning, causing a potential snag hazard. Ensure a safe distance between the gyroscope and all personnel. Never touch any part of the gyroscope._

**The motor must never exceed 800 RPM!**
{% endhint %}

1. Switch on the ECP control box.
2. Brake all axes so that the accelerating rotor does not cause the axes to move from the initialized positions:
   1. Set Axis #3 and Axis #4 on the ECP control box to ON.
   2. Set Axis #2 V-Brake on the ECP software main screen to ON.
3. Click on the `Command` menu and then on `Initialize Rotor Speed.`
4. **Being careful not to make a typo**, input 400 RPM and click OK. If you do accidentally input a speed higher than intended, click `Abort Control` on the main screen, go back to the `Initialize Rotor Speed` input and input the correct RPM.
5. Once the rotor has reached the desired RPM, release Axis 2 and Axis 4 brakes. Axis 3 should remain braked throughout the entirety of the lab.
6. Observe gyroscopic precession and nutation in open-loop using a ruler or similar object to perturb the system about Axis #4:
   1. Apply a constant torque to Axis #4 by pushing on the outer part of the frame. You should notice that the gyro rotates about Axis #2 at a constant angular _rate_. Contrast this with a constant angular _acceleration_ that would have occurred if the rotor were not turning. This is known as _**gyroscopic precession**._
   2. Apply a momentary torque to Axis #4 by sharply jabbing the outer part of the frame. You should notice that the gyro slightly "wobbles" about Axis #2. This is known as _**gyroscopic nutation**._ In the absence of damping from friction in the bearings and air resistance due to the angular motion, this wobble would last indefinitely.&#x20;
   3. Return Axis #2 to its vertical position by torquing Axis #4.

#### Configure Control Algorithm

1. Go to the `Setup` menu and clicking on `Control Algorithm`. &#x20;
2. Click the `Load From Disk` button and select the default controller file, _GyroControl\_Gimbal4\_PID.alg_. The original file can be found at _C:\AE4610-ControlsLab\Gyro\_Controller_ but, since this version is read-only, it should be loaded from a locally saved copy on the TA's login profile.
3. Be sure that the sample period, $$T_s$$, is **0.00884** seconds. $$T_s$$ is located both on the control algorithm screen and in the program itself, so be sure that both values are correct.&#x20;
4. Set the $$k_v, k_p$$ and $$k_d$$ gains to the values from your controller design in Part B.
   1. **Check the values with a TA before entering them**.
   2. **Do not input magnitude of** $$\bm{k_p > 6.0}$$ **nor** $$\bm{k_d > 0.4}$$**.**
   3. Change values by clicking the **Edit Algorithm** button, editing the relevant section of code, and exiting the edit algorithm section by selecting **Save Changes and Quit** under the **File** menu.&#x20;
   4. Retain the sign of each gain, only changing the magnitude from your calculations in Part B;  $$k_v$$ and $$k_d$$should be positive, whereas $$k_p$$ should be negative.
5. Enable the algorithm by clicking **Implement Algorithm**.
6. Click **OK**, and check that the Control Loop Status now reads **CLOSED** on the main screen.

### Controller Evaluation

With the rotor initialized and the closed-loop controller active, you will now evaluate your calculated gains in three different trajectories.

#### Trajectory 1: Small Step Input

1. Select the `Command menu` and then `Trajectory 1`.
2. Select the _Step Input_ and then click the `Setup` button. Enter the following parameters for this case:
   1. Step size = 200 counts.
   2. Dwell time = 1000 ms.
   3. 1 repetition.&#x20;
3. Hit _OK_, then _OK_ again to leave the setup trajectory menus.
4. Ensure the rotor is oriented vertically using the wooden ruler.
5. Ensure all people are at a safe distance from the gyroscope.
6. Go to the `Command` menu and click `Execute`.  Make sure that both _Normal Data Sampling_ and _Execute Trajectory 1 Only_ are checked.
7. Click the `Run` button. The input trajectory will be run on the gyroscope now. When the box on the screen says _Upload Complete_, click on the `OK` button.
8. Plot and review the data:
   1. Go to the `Plotting` menu and select `Setup Plot`. Set _Command Position 1_ and _Encoder 4 Position_ on the left axis and _Control Effort 2_ on the right axis. Click `OK` when finished to view the plot.
   2. If desired, zooming the plot can be accomplished by going to the `Plotting` menu and selecting _Axis Scaling_.
   3. Discuss the results, particularly the quality of your controller's response and how it may be improved, with your TAs. Given enough time at the end of the lab, you may return to this trajectory and see how gain changes may improve the controller's response.
9. Save the data by clicking on `Data > Export Raw Data`.

#### Trajectory 2: Large Step Input

1. Repeat steps 1-8 from Trajectory 1, but this time using the following trajectory settings:
   1. Step size = 1000 counts.
   2. Dwell time = 1000 ms.
   3. 1 repetition.
2. Save the data by clicking on `Data > Export Raw Data`.

#### Trajectory 3: Ramp Input

The step response is useful for system characterization but is seldom used for an actual in-service trajectory because it is excessively harsh (high _acceleration_ and _jerk_ ("rate of change of acceleration")). A more common trajectory used for tracking applications is a _ramp_.

1. Repeat steps 1-8 from Trajectory 1, this time using the following trajectory settings:
   1. Ramp input (Unidirectional Moves **not checked).**
   2. Distance = 6000 counts.
   3. Velocity = 2000 counts/sec.
   4. Dwell Time = 1000 ms.
   5. 2 repetitions.
2. Save the data by clicking on `Data > Export Raw Data`.
3. In addition to the previous plots, plot Control Effort 2 and Encoder 2 Velocity data. (This should be done in separate plots since the scaling of these two variables is greatly different.) Note the relatively close tracking and rapid accelerations at each end of the constant velocity sections. This would not be possible for the small actuator of Axis #2 acting on the massive assembly in a conventional fashion. By using gyroscopic control actuation and the associated transfer of momentum stored in the rotor, the high authority control is made possible.

#### Trajectory 1 Re-Run

_**Only if time permits at the end of your session**,_ go back to Trajectory 1 and see if you can improve the controller's response by manually adjusting the gains using your intuition and knowledge of individual gain effects. This task is intended only to allow you an opportunity to get a real-life feel for the effect of gain changes, it will not be graded.

## Analysis & Lab Report

*   Include an overview of the controller design from Part B.


* Using the SIMULINK block diagram you have developed of Fig. 3.6 in Part B, simulate the system response for the same step and ramp commands you have used in Part C. Compare your simulated responses with the corresponding experimental results. What is the rise time and % overshoot in the experiment and simulation responses to the step command used?

