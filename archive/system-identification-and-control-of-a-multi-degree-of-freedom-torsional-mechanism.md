---
description: >-
  This manual documents the modeling of a 2-DOF torsional pendulum, followed by
  the design of a collocated PD controller, and finally the controller
  implementation and evaluation on the hardware setup.
---

# Torsional Pendulum

## Objective

The objectives of this laboratory experiment are as follows:

1.  Identify the parameters of a multi-degree-of-freedom system.


2.  Demonstrate some key concepts associated with proportional plus derivative control for a two degree-of-freedom torsional mechanism.


3. Implement a PD controller on a 2-disk (2 DOF) system where the feedback is based on the angular displacement $$\theta_1$$ of the lower disk. Such a scheme is referred to as **collocated** since the sensor output is directly coupled to the actuator input.

Implementing a suitable controller on a 2-disk (2 DOF) system, where the feedback is based on the angular displacement $$\theta_3$$ of the upper disk is referred to as **non-collocated** since the sensor output and the actuator input are at different locations. The addition of the spring and the second inertia to the rigid body single DOF increases the plant order by two and adds an oscillatory mode to the plant dynamics. This may be thought of, in a sense, as a dynamic disturbance to the rigid body plant (single DOF).

![Image source: https://www.phywe.com/en/torsional-vibrations-and-torsion-modulus.html](<../.gitbook/assets/image (6).png>)

## Equipment Required

*   [ ] Torsional mechanism (electromechanical plant Model 205a)


*   [ ] Input / Output Electronic ECP Model 250 box


*   [ ] DSP based Controller/Data Acquisition Board which is already installed in a PC


* [ ] MATLAB, SIMULINK and ECP software

## Part A: Modeling

This part of the experiment involves the first phase – system identification and modeling of the torsional mechanism. This is performed by representing the mechanism as a combination of spring-mass-damper systems and determining the model parameters from the system response. The approach will be to indirectly measure the inertia, spring, and damping parameters by making measurements of the system response while set up in a pair of classical spring-mass configurations.

### Electromechanical Plant

A schematic of the plant interconnection with the control computer is shown in Fig. 4.1-(b). The plant is shown in Fig. 4.1-(a). It consists of three disks supported by a torsionally flexible shaft which is suspended vertically on anti-friction ball bearings. The shaft is driven by a brushless servo motor connected via a rigid belt and pulley system with a 3:1 speed reduction ratio. An encoder located on the motor is used for commutation. This is the process by which the current is distributed to the motor coils. In order to commutate the motor, a sensor connected to the motor shaft is used to feedback the instantaneous rotor position. The encoder on the base of the shaft measures the angular displacement (in counts) which is converted to an angle $$\theta_1$$ of the lower disk $$J_1$$ . The second disk $$J_2$$ is connected to its encoder, which measures $$\theta_2$$, by a belt/pulley with a 1:1 pulley ratio and similarly the third disk $$J_3$$ is connected to its encoder (which measures $$\theta_3$$) by a rigid belt/pulley with a 1:1 pulley ratio.

![Figure 4.1. Experimental control system](<../.gitbook/assets/image (60).png>)

The plant may be placed in a variety of free and clamped configurations with 1, 2, and 3 degrees of freedom. For 1 and 2 DOF plants, the torsional spring constant $$k_1$$ may be halved by the choice of disk location. Changing configuration often requires removing or replacing inertia disks. Although these operations are straightforward, it is recommended that they are performed with care. The user may change inertia values by changing the number of masses and/or their location on a given disk.

In the following experimental procedure, we will identify the plant parameters $$J_1,\space J_3,\space k_1,\space c_1$$ and $$c_3$$, where $$J$$ is the mass moment of inertia, $$k$$ is the torsional spring constant, and $$c$$ is the damping coefficient.

### System Identification

Consider a one-DOF underdamped system model given by the second-order scalar differential equation

$$
J \ddot{\theta} + c\dot{\theta} +k\theta = 0 \tag{4.1}
$$

We will use the **logarithmic decrement** to estimate the damping ratio of the system. The solution of (4.1) is given by

$$
\theta(t) = \Theta e^{-\zeta \omega_n t}\cos(\omega_d t + \phi)
$$

where

$$
2\zeta \omega_n = \frac{c}{J}, \qquad \omega_n = \sqrt{\frac{k}{J}}, \qquad \omega_d = \omega_n \sqrt{1-\zeta^2} \tag{4.2}
$$

The value of $$\theta$$ during two complete successive cycles is given by

$$
\theta(t_1) = \Theta e^{-\zeta \omega_nt_1}\cos(\omega_d t_1 + \phi)
$$

$$
\theta(t_2) = \Theta e^{-\zeta \omega_n t_2}\cos(\omega_d t_2 + \phi)
$$

where

$$
t_2 = T_d + t_1, \space \text{with} \space T_d = \frac{2\pi}{\omega_d}
$$

![Figure 4.2. Calculation of the logarithmic decrement](<../.gitbook/assets/image (66).png>)

Since

$$
\cos(\omega_dt_2 + \phi) = \cos(2\pi + \omega_dt_1 + \phi) = \cos(\omega_dt_1 + \phi)
$$

it follows that

$$
\frac{\theta(t_1)}{\theta(t_2)} = \frac{e^{-\zeta \omega_n t_1}}{e^{-\zeta \omega_n (t_1+T_d)}} = e^{\zeta \omega_n T_d} \tag{4.3}
$$

Let the logarithmic decrement $$\delta$$ be given by

$$
\delta = \ln\frac{\theta(t_1)}{\theta(t_2)}
$$

It follows then easily from (4.3) and (4.2) that

$$
\delta = \zeta\omega_nT_d = \zeta\omega_n\frac{2\pi}{\omega_n \sqrt{1-\zeta^2}}
$$

or that

$$
\delta = \frac{2\pi\zeta}{\sqrt{1-\zeta^2}} = \left(\frac{2\pi}{\omega_d}\right) \left(\frac{c}{J}\right) \tag{4.4}
$$

Finally, if $$\delta$$ is known, an estimate for $$\zeta$$ is given by

$$
\zeta = \frac{\delta}{\sqrt{(2\pi)^2 + \delta^2}}
$$

The previous procedure can be repeated also for the case when measurements are taken over non-successive cycles. To this end, notice that (see Fig. 4.2) $$t_3 = t_1 + 2T_d$$ and more generally, that

$$
t_m = t_1 +mT_d
$$

Therefore

$$
\frac{\theta(t_1)}{\theta(t_{m+1})} = \frac{\theta(t_1)}{\theta(t_2)}\frac{\theta(t_2)}{\theta(t_3)} \cdots \cdots \frac{\theta(t_m)}{\theta(t_{m+1})}
$$

where

$$
\frac{\theta(t_i)}{\theta(t_{i+1})} = e^{\zeta\omega_nT_d}, \quad i = 1, \ldots, m
$$

The previous two equations give that

$$
\frac{\theta(t_1)}{\theta(t_{m+1})} = (e^{\zeta\omega_nT_d})^m = e^{m\zeta\omega_nT_d}
$$

or that

$$
\ln\left(\frac{\theta(t_1)}{\theta(t_2)}\right) = m \zeta \omega_n T_d = m \delta
$$

Finally,&#x20;

$$
\delta = \frac{1}{m} \ln\left(\frac{\theta(t_1)}{\theta(t_{m+1})}\right)
$$

### Loaded vs. Unloaded Disk

Assume that the natural frequency of the loaded disk is $$\omega_{n_\ell}$$ and the corresponding damping ratio is $$\zeta_\ell$$ . Therefore,

$$
\omega_{n_\ell} = \frac{\omega_{d_\ell}}{\sqrt{1-\zeta_\ell^2}}, \qquad \zeta_\ell = \frac{\delta_\ell}{\sqrt{(2\pi)^2 + \delta_\ell^2}} \tag{4.5}
$$

where $$\omega_{d_\ell}$$ is the damped natural frequency for the loaded disk case (denoted by subscript ‘$$\ell$$'). Similarly, for the unloaded disk case (subscript '$$u$$') one has

$$
\omega_{n_u} = \frac{\omega_{d_u}}{\sqrt{1-\zeta_u^2}}, \qquad \zeta_u = \frac{\delta_u}{\sqrt{(2\pi)^2 + \delta_u^2}} \tag{4.6}
$$

Recall that

$$
\omega_{n_l}^2 = \frac{k}{J_m+J}, \qquad \omega_{n_u}^2 = \frac{k}{J}
$$

for the loaded and unloaded cases, respectively. Solving for $$k$$ and equating the resulting expressions yields &#x20;

$$
\omega_{n_l}^2 (J_m + J) = \omega_{n_u}^2J
$$

or that

$$
J = \frac{J_m\omega_{n_l}^2}{\omega_{n_u}^2 - \omega_{n_l}^2} \tag{4.7}
$$

From (4.7) and (4.2), the damping coefficient can be computed from

$$
c = 2\zeta_uJ\omega_{n_u} \tag{4.8}
$$

### Experimental Procedures

1.  For model 205a, clamp the center disk using the 1/4” bolt, square nut, and clamp spacer as shown in Fig. 4.3. Only light torqueing on the bolt is necessary.


2.  Secure four 500g masses on the upper and lower disks. Verify that the masses are secured properly and that centerline of each mass is at **** a distance of 9.0 cm from the shaft centerline.

    &#x20;                                           <img src="../.gitbook/assets/image (39).png" alt="" data-size="original">&#x20;

    &#x20;                                      **Figure 4.3. Electromechanical plant model 205a**


3.  Enter the program by clicking on the shortcut to 3D Torsion on the desktop and turn on the torsional mechanism.


4.  Enter **File** menu, choose **Load Setting** and select the file **C:\Program Files (x86)\ECP Systems\cn\default.cfg**.


5.  Choose the correct personality file by going to **Utility** menu and clicking on **Download Controller Personality File**. Download the file **C:\Program Files (x86)\ECPSystems\cn\M205di6.pmc**. This will implement ‘CLOSED’ Loop by default. Click on **Abort Control** at the lower right of the screen to ‘OPEN’ the loop.


6.  Enter the **Control Algorithm** box via the **Setup** menu and set `Ts = 0.00442` second, then OK.  Enter the **Command** menu, go to **Trajectory** and select **Step, Setup**. Select **Open Loop Step** and input a step size of 0 (zero), a duration of 4000 ms and 1 repetition. Exit to the Background Screen by consecutively selecting **OK**. This puts the controller board in a mode for acquiring 8 sec of data on command but without driving the actuator.


7.  Under **Data** menu, go to **Setup Data Acquisition** and select Encoder #1 and Encoder #3 as data to acquire and specify data sampling every 2 (two) servo cycles, i.e. every 2 Ts's. Select **OK** to exit. Select **Zero Position** from the **Utility** menu to zero the encoder positions.


8.  From the **Command** menu, select **Execute**. Prepare to manually displace the upper disk by approximately 20 deg. Exercise caution in displacing the inertia disk; displacements beyond 40 deg may damage and possibly break the flexible drive shaft. (Displacements beyond 25 deg will trip a software limit which disables the controller indicated by "Limit Exceeded" in the Controller Status box in the Background Screen. To reset, simply reselect Execute from the Command menu.) With the upper disk displaced approximately 20 deg (≤ 1000 encoder counts as read on the Background Screen display) in either direction, select **Run** from the **Execute** box and release the disk approximately 1 second later. The disk will oscillate and slowly attenuate while encoder data is collected to record this response.  Select **OK** after data is uploaded.


9.  Under the **Plotting** menu, select **Setup Plot** and choose Encoder #3 position, and then select **Plot Data** from the **Plotting** menu.  You will see the upper disk time response.


10. Save the data by clicking on the **Data** menu and going to **Export Raw Data**.


11. Remove the four masses from the third (upper) disk and repeat steps 8 through 10 for the unloaded disk.  If necessary, repeat step 6 to reduce the execution (data sampling) duration.


12. Repeat steps 8 through 11 for the lower disk, disk # 1. For the lower disk experiment, in step 9, you will need to remove Encoder #3 position and add Encoder #1 position to the plot set-up.


13. Remember to switch off the system when you are done with your experiments.

### Analysis

1.  Determine the damped natural frequency (in rad/sec) of the upper disk from the plot obtained for the upper disk when it is loaded with the 4 masses by choosing several consecutive cycles (for example, 5 to 10) in the amplitude range between 100 and 1000 counts (much smaller amplitude responses become dominated by nonlinear friction effects and do not reflect the salient system dynamics). This damped frequency for the loaded upper disk, $$\omega_{d_{U\ell}}$$, is related to the natural frequency for the loaded upper disk, $$\omega_{n_{U\ell}}$$, according to Eqn. (4.6), where the subscript ‘$$U\ell$$’ denotes the upper loaded disk.


2.  Measure the reduction from the initial cycle amplitude $$X_0$$ to the last cycle amplitude $$X_n$$ for the _n_ cycles measured above. Using relationships associated with the logarithmic decrement and damping ratio, find the damping ratio $$\zeta_{Ul}$$ and the natural frequency $$\omega_{n_{U\ell}}$$.


3.  Determine the damped frequency $$\omega_{d_{Uu}}$$for the unloaded upper disk from the corresponding unloaded upper disk response of the Experimental Procedure. Repeat steps 1 and 2 of the Analysis to calculate the damping ratio $$\zeta_{Uu}$$ and the natural frequency $$\omega_{n_{Uu}}$$of the unloaded upper disk, where the ‘$$Uu$$’ subscript denotes the upper unloaded disk.


4.  Obtain $$\omega_{n_{L\ell}}, \space \omega_{n_{Lu}}, \space \zeta_{L\ell}$$ and $$\zeta_{Lu}$$ from the plots of the lower disk response for the loaded and unloaded cases respectively, where the '$$L\ell$$' and '$$Lu$$' subscripts denote loaded and unloaded disk cases respectively for the lower disk. How do these damping ratios compare with that for the upper disk?


5.  Use the following information pertaining to each mass piece to calculate the portion of each disk’s inertia attributable to the masses for the ‘$$U\ell$$’ (upper disk - loaded) and '$$L\ell$$' (lower disk - loaded) cases.

    * Mass (including bolt and nut): 500g (± 5g)
    * Diameter: 5.00 cm (± 0.02 cm)

    Calling the **total** inertia from these masses about the shaft centerline as $$J_m$$, use the following relationships to solve for the unloaded upper disk inertia $$J_U$$, and upper torsional shaft spring $$k_U$$.\


    &#x20;                                               $$\omega_{n_{Ul}}^2= \displaystyle\frac{k_U}{J_m + J_U} \quad \text{ and } \quad \omega_{n_{Uu}}^2 = \frac{k_U}{J_U}$$&#x20;



    Using $$\zeta_{Uu}$$ and $$\omega_{n_{Uu}}$$ , find the damping coefficient $$c_U$$ with Eqn. 4.8.


6.  Repeat the procedure to find the lower unloaded disk inertia ($$J_L$$), the spring constant of the lower torsional shaft ($$k_L$$) and the damping of the lower disk ($$c_L$$). (Take the inertia contribution of the motor, belt, and pulleys to be $$J_{\text{motor-belt-pulleys}}$$ = 0.0005 kg.m$$^2$$).


7. Compare the damping ratios of the loaded and unloaded cases of upper and lower disks and report your observations.

## Part B: Controller Design

This part of the experiment involves the second phase – the development of PD collocated control on the torsional mechanism. This is done by using the parameters determined from Part A to model the system and determining proportional and derivative gains for the controller using MATLAB based on the given design specifications.

The configuration of the torsional mechanism that will be used for controller design and evaluation is shown in Fig. 4.4.&#x20;

![Figure 4.4. Configuration of the torsional mechanism used for controller design and evaluation](<../.gitbook/assets/image (74).png>)

In this configuration, the center disk is removed and two masses are added to both top and bottom disks.&#x20;

### Theoretical Background

![Figure 4.5. Free-free two DOF torsional plant](<../.gitbook/assets/image (35).png>)

![Figure 4.6. Free body diagram of the lower disk (left) and upper disk (right)](<../.gitbook/assets/image (106).png>)

### Equations of Motion

This section provides time and Laplace domain expressions which are useful for linear control implementation and are used in the experiments described later. The most general form of the two degrees of freedom torsional system is shown in Fig. 4.5, where friction is idealized as being viscous.

It is important to note that the setup includes two small masses on each disk and the controller design will be done for this case. Hence, $$J_1$$ includes the mass moment of inertia of the lower disk ($$J_L$$), the mass moment of inertia contribution from **two** masses ($$J_m$$) and the mass moment of inertia of the motor-belt-pulley system ($$J_{\text{mbp}}$$), i.e., $$J_1 = J_L + J_m+J_{\text{mbp}}$$. Similarly, $$J_3$$ includes the mass moment of inertia of the upper disk ($$J_U$$) and the mass moment of inertia contribution from **two** masses ($$J_m$$), i.e., $$J_3 = J_U+J_m$$. Additionally, $$c_1$$ is taken as $$c_{Lu}$$ while $$c_3$$ is taken as $$c_{Uu}$$.

{% hint style="info" %}
In Part A, $$J_m$$ represented the total mass moment of inertia contribution from four masses (added to each disk). In Part B and Part C, there is a contribution of mass moment of inertia from only two masses, so in this case, $$J_m$$ (as calculated in Part A) should be suitably modified and will be lower than earlier.
{% endhint %}

Using the free body diagram of the 2 DOF free-free case, shown in Fig. 4.6, and summing torques acting on $$J_1$$ , one obtains&#x20;

$$
J_1\ddot{\theta}_1 +c_1\dot{\theta}_1+k_1\theta_1-k_1\theta_3=T(t) \tag{4.9}
$$

where

* $$T(t) = k_{VT}V$$&#x20;
* $$V$$ = applied voltage
* $$k_{VT}$$, gain that converts volts to torque in N-m, is calculated as follows (see Fig. 4.7):

![Figure 4.7. Computing the gain k\_VT](<../.gitbook/assets/image (45).png>)

$$
k_{VT} = k_a \cdot k_t \cdot k_p \tag{4.10}
$$

where

* $$k_a$$, the Servo Amp gain = 2 amp/V&#x20;
* $$k_t$$, the Servo Motor Torque constant = 0.1 N-m/amp&#x20;
*   $$k_p$$, the Drive Pulley ratio = 3 (N-m @ disk/N-m @ Motor)



    Thus, substituting these values in (4.10), we obtain $$k_{VT}$$ = 0.6 N-m/V. Similarly, summing torques acting on $$J_3$$&#x20;

$$
J_3\ddot{\theta}_3+c_3\dot{\theta}_3+k_1\theta_3-k_1\theta_1=0 \tag{4.11}
$$

These may be expressed in a state-space realization as

$$
\dot{x} = Ax + BT(t)
$$

where

$$
x = \begin{bmatrix}
\theta_1 \\
\theta_3 \\
\dot{\theta}_1 \\
\dot{\theta}_3
\end{bmatrix}, \qquad A = \begin{bmatrix}
0 &0 &1 &0 \\
0 &0 &0 &1 \\
-\frac{k_1}{J_1} &\frac{k_1}{J_1} &-\frac{c_1}{J_1} &0\\
\frac{k_1}{J_3} &-\frac{k_1}{J_3} &0 &-\frac{c_3}{J_3}
\end{bmatrix}, \qquad B = \begin{bmatrix}
0 \\
0 \\
\frac{1}{J_1} \\
0
\end{bmatrix} \tag{4.12}
$$

Taking the Laplace transfer of the above equations and assuming zero initial conditions, we may solve for the transfer functions

$$
\frac{\theta_1(s)}{T(s)} = \frac{(J_3s^2+c_3s+k_1)}{D(s)} \tag{4.13}
$$

$$
\frac{\theta_3(s)}{T(s)} = \frac{k_1}{D(s)} \tag{4.14}
$$

where

$$
D(s) = J_1J_3s^4 + (c_1J_3+c_3J_1)s^3 + (J_1k_1+J_3k_1+c_1c_3)s^2 + (c_1k_1+c_3k_1)s \tag{4.15}
$$

In Figs. 4.5 and 4.6, $$k_1$$ is the resultant of the stiffness of the lower ($$k_L$$) and the upper ($$k_U$$) shaft in series. The values for the stiffness of the lower and the upper shaft are the stiffness parameters that you identified in Part A. Then, knowing that these shafts are in series, use the formula

$$
k_1 = \frac{k_Lk_U}{k_L+k_U}
$$

to obtain the total stiffness. The block diagram of the closed-loop system with the PD controller is shown in Fig. 4.8. The gain $$k_{\text{hw}}$$ is the hardware gain and is given by

$$
k_{\text{hw}} = k_c \cdot k_a \cdot k_t \cdot k_p \cdot k_e \cdot k_s
$$

where

* $$k_c$$ , the DA**C** gain = 10V / 32,768 DAC counts
* $$k_a$$, the Servo **A**mp gain = approx. 2 (amp/V)
* $$k_t$$ , the Servo Motor **T**orque constant = approx. 0.1 (N-m/amp)
* $$k_p$$ , the Drive **P**ulley ratio = 3 (N-m @ Disc / N-m @ Motor)
* $$k_e$$ , the **E**ncoder gain = 16,000 pulses / 2π radians
* $$k_s$$ , the Controller **S**oftware gain = 32 (controller counts / encoder or ref input counts)

The gains of the PD controller $$K_p$$ and $$K_d$$ are the free parameters that must be chosen to achieve specific performance objectives (rise time, overshoot, etc.). The plant transfer function

$$
G_p(s) = \frac{n(s)}{d(s)} = \frac{\theta_1(s)}{T(s)}
$$

where $$\theta_1$$ is the lower disk displacement and $$T$$ is the input torque at the motor.

In this experiment, we consider PD control of a 2-disk system where the controlled output, $$\theta_1$$, is of the lower disk. Such a scheme is referred to as collocated since the sensor output is rigidly coupled to the actuator input.

The addition of the spring and second inertia increases the plant order by two and adds an oscillatory mode to the plant dynamics. This may be thought of, in a sense, as a dynamic disturbance to the rigid body plant. The collocated PD control implemented here is the approach most commonly used in industry. It may be practically employed when there is flexibility between the actuator and some inertia, and the location of objective control being near the actuator. If the location of objective control is at the distant inertia, however, this method has its limitations.

The approach in this experiment will be to design the controller by interactively changing the PD gains and observing their effect on the physical system.

![Figure 4.8. Closed-loop block diagram of the torsional plant with a PD controller - collocated control](<../.gitbook/assets/image (4).png>)

### Specifications & Procedure

#### Specifications

1.  All closed-loop poles must be in the left half of the complex plane.


2.  The rise time to a unit step command input must be less than 0.4 sec (for $$\theta_1$$).


3.  The overshoot in response (for $$\theta_1$$) to a unit step command input must be less than 10% without excessive oscillation.


4. $$K_p$$ must be less than 1.0 and $$K_d$$ must be greater than 0.02 but less than 0.1.

#### Procedure

1.  Open a new m-file and compute the transfer function $$\theta_1(s)/T(s)$$ using Eqn. 4.13. Save your m-file.


2.  Use **controlSystemDesigner**/**rltool** in MATLAB and select appropriate values for $$K_p$$ and $$K_d$$ to meet the given set of specifications. Start with an initial guess for the gains and adjust. You need to meet the requirements in **controlSystemDesigner**/**rltool**. Save the **controlSystemDesigner**/**rltool** figures and your gains. Make sure to use the consistent controller architecture in the **controlSystemDesigner**/**rltool**. This will be known as high gain controller.


3.  Compute the transfer function $$\theta_3(s)/\theta_1(s)$$ using Eqns. 4.13 and 4.14.


4.  Develop a SIMULINK diagram using the block diagram shown in Fig. 4.8 as reference and save it.


5.  Obtain the SIMULINK response of $$\theta_1$$ to a unit step command using the set of high gains (from Step 2) you have selected. Save the response, and make sure it proves you meet the requirements.


6.  Using the gains from the high gain controller as starting points in SIMULINK, iteratively reduce gains, until you obtain a well-behaved step response of $$\theta_3$$ with ≤ 10% overshoot (without excessive oscillation) and as fast a rise time as possible. Save these gains as a different set, which will correspond to a low gain controller and will be tested in Part C.


7.  Obtain the SIMULINK response of $$\theta_1$$to a unit step command using the set of low gains (from Step 6) and compare it with the response from Step 5. Are the design specifications still satisfied?


8. The TAs will need to see: Transfer functions, Gains, controlSystemDesigner/rltool plots, Simulink diagram, and Simulink response to a step command (proving overshoot and rise time requirements are met).

{% hint style="info" %}
You must have your work checked out by one of the TAs before leaving the lab to get credit for your work.
{% endhint %}

## Part C: Controller Implementation & Evaluation

{% hint style="danger" %}
### Safety Briefing

**Safety Note 1:** The system's safety functions must be verified before each operational session. The system must be checked visually to verify that the disks, masses and connecting shaft all appear to be undamaged and securely fastened.

**Safety Note 2:** In the event of an emergency, control effort should be immediately discontinued by pressing the red "OFF" button on the front of the control box.

**Safety Note 3:** Stay clear of and do not touch any part of the mechanism while it is moving, while a trajectory is being executed or before the active controller has been safety checked.

**Safety Note 4:** Verify that the masses and inertia disks are secured as per he instructions prior to powering up the Control Box.

**Safety Note 5:** Never leave the system unattended while the Control Box is powered on.
{% endhint %}

### Implementation

1.  Set up the system with two masses on the upper and lower disk as shown in Fig. 4.9. Ensure that the two masses are located along the hub split line of the disks. Observe that the middle disk has been removed.


2.  Enter the program by clicking on the shortcut to 3D Torsion on the desktop and turn on the torsional mechanism.

    &#x20;                                            <img src="../.gitbook/assets/image (18).png" alt="" data-size="original">&#x20;

    &#x20;     **Figure 4.9. Electromechanical plant model 205a configured for closed loop experiments**
3.  Enter **File** menu, choose **Load Setting** and select the file **C:\Program Files (x86)\ECP Systems\cn\default.cfg**.


4.  Choose the correct personality file by going to **Utility** menu and clicking on **Download Controller Personality File**. Download the file **C:\Program Files (x86)\ECPSystems\cn\M205di6.pmc**.


5.  Enter the **Control Algorithm** box under **Setup** and ensure that the sample period, **Ts**, is **0.00442 second** and select **Continuous Time Control**. Select **Pl + Velocity Feedback** (this is the return path derivative form) and **Setup Algorithm**. Enter the high gain values Kp and Kd determined earlier (Ki = 0) and select OK. Check the values with a TA before entering them.  **Do not input magnitude of Kp > 1.0 nor Kd > 0.2 and Kd < 0.02.** Ts is located both on the control algorithm screen and in the program itself, so be sure that both values are correct. The algorithm can now be implemented by selecting **Implement Algorithm,** then OK.


6. First, displace the lower disk with a light, non-sharp object (e.g., a plastic ruler) to verify stability prior to touching the plant. Similarly, displace the upper disk and observe the response. Note the difference in their stiffness.

### Evaluation

1.  Go to the **Data** menu and click on **Setup Data Acquisition**. Check that data is gathered every 5 servo cycles. Be sure that the Commanded Position, Encoder 1 Position and Encoder 3 Position are all located in the **Selected Items** box.  If they are not, add them to that list by selecting them in the **Possible Choices** box and then clicking on the **Add Item** button. When finished, hit **OK** to exit this menu.


2.  Prepare the input for the system by selecting the **Command** menu and then **Trajectory**. **Uncheck** the box Unidirectional Moves. Select the **Step -> Setup -> Closed loop Step Input**.  Enter the following parameters for this case: step size, 1000 counts; dwell time, 5000 ms; 1 repetition. When finished, hit **OK**, then **OK** again to leave the setup trajectory menus.


3.  Go to the **Utility** menu and click **Zero Position**. Go to the **Command** menu and click **Execute**. Then click the **Run** button.  The input trajectory will be run on the torsional mechanism now.  When the box on the screen says **Upload Complete**, click on the **OK** button.


4.  After data collection has finished, go to the **Plotting** menu and select **Setup Plot**.  Set Command Position and Encoder 1 Position on the left axis.  Click on the **Plot Data** button when finished.  This will generate a plot of the data from the executed trajectory.  If desired, zooming the plot can be accomplished by going to the **Plotting** menu and selecting **Axis Scaling**. Similarly, obtain the plot of Encoder 3 position.


5.  Save the data by clicking on the **Data** menu and going to **Export Raw Data**.


6.  Repeat Steps 5-11 using the low gain controller values determined in Part B. How does the physical stiffness of the setup compare with the high gain controller?


7. Remember to SWITCH OFF the system when you are done with your experiments.

### Analysis

1.  Compare the steady-state error values of the high gain controller as well as low gain controller response of $$\theta_1$$ obtained in the experiment and explain any difference.


2.  Simulate the system response for the same step command you have used in the lab experiment. Compare your simulated responses with the corresponding experimental results for both $$\theta_1$$ and $$\theta_3$$ . What is the rise time and % overshoot in the experiment and simulation responses to the step command used (only for $$\theta_1$$)? Explain the cause for any difference.


3. Using controlSystemDesigner/rltool, determine the gain margin (GM) and phase margin (PM) of the closed-loop system of the controller for the high gain controller as well as the low gain controller.&#x20;

