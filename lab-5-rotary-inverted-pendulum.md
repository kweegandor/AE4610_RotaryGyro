# Lab 5: Rotary Inverted Pendulum

## Objective

The objectives of this laboratory experiment are as follows:

1. Linearize nonlinear equations of motion.
2. Obtain the linear state-space representation of the rotary pendulum plant.
3. Design a state-feedback control system that balances the pendulum in its upright Pole Placement.
4. Simulate the closed-loop system to ensure the specifications are met.
5. Use energy-based control schemed to develop swing up pendulum control.
6. &#x20;Implement controllers on the Quanser Rotary Pendulum plant and evaluate the performance.

## Equipment

* [ ] Rotary pendulum (Quanser Rotary inverted pendulum module)
* [ ] Rotary servo base (Quanser SRV02)
* [ ] Power amplifier (Quanser VoltPaq-X2)
* [ ] DAQ (Quanser Q2-USB)
* [ ] MATLAB and Simulink

## Modeling

This experiment involves modeling of the rotary inverted pendulum and calibration of the Simulink model so that the system follows the convention described in this lab manual.

### Rotary Inverted Pendulum Model

![Figure 1 Rotary servo base with Rotary Pendulum module](.gitbook/assets/rotary\_setup.png)

#### _Table 1 Rotary Pendulum components (Figure 1)_

| ID# | Component                  |
| :-: | -------------------------- |
|  1  | Rotary Servo               |
|  2  | Thumbscrews                |
|  3  | Rotary Arm                 |
|  4  | Shaft Housing              |
|  5  | Shaft                      |
|  6  | Pendulum T-Fitting         |
|  7  | Pendulum Link              |
|  8  | Pendulum Encoder Connector |
|  9  | Pendulum Encoder           |

#### Table 2 Main Parameters associated with the Rotary Pendulum module

| Symbol          | Description                                                       | Value        | Unit                    |
| --------------- | ----------------------------------------------------------------- | ------------ | ----------------------- |
| $$m_p$$         | Mass of pendulum                                                  | 0.127        | $$kg$$​                 |
| $$L_p$$​        | Total length of pendulum                                          | 0.337        | $$m$$                   |
| $$l_p$$​        | Distance from pivot to center of mass                             | 0.156        | $$m$$                   |
| $$J_{p,cm}$$​   | Pendulum moment of intertia about center of mass                  | 0.0012       | $$kg \cdot m^2$$        |
| $$B_p$$         | Pendulum viscous damping coeffi­cient as seen at the pivot axis   | 0.0024       | $$N\cdot m\cdot s/rad$$ |
| $$m_{arm}$$​    | Mass of rotary arm with two thumb screws                          | 0.257        | $$kg$$​                 |
| $$L_r$$         | Rotary arm length from pivot to tip                               | 0.216        | $$m$$​                  |
| $$l_{arm}$$​    | Rotary arm length from pivot to center of mass                    | 0.0619       | $$m$$                   |
| $$J_{arm,cm}$$​ | Rotary arm moment of inertia about its center of mass             | 9.98 x 10^-4 | $$kg \cdot m^2$$​       |
| $$B_r$$​        | Rotary arm viscous damping coeffi­cient as seen at the pivot axis | 0.0024       | $$N\cdot m\cdot s/rad$$ |
| $$J_{arm}$$​    | Rotary arm moment of inertia about pivot                          | 0.0020       | $$kg\cdot m^2$$         |
| $$K_{enc}$$​    | Pendulum encoder resolution                                       | 4096         | $$counts/rev$$​         |

### Model Convention

The rotary inverted pendulum model is shown in Figure 2. The rotary arm pivot is attached to the Rotary Servo system and is actuated. The arm has a length of $$L_r$$, a moment of inertia of $$J_r$$, and its angle, $$\theta$$, increases positively when it rotates counterclockwise (CCW). The servo (and thus the arm) should turn in the CCW direction when the control voltage is positive, i.e. $$V_m$$ > 0.

The pendulum link is connected to the end of the rotary arm. It has a total length of $$L_p$$ and it center of mass is $$\frac{L_p}{2}$$ . The moment of inertia about its center of mass is $$J_p$$. The inverted pendulum angle, $$\alpha$$, is zero when it is perfectly upright in the vertical position and increases positively when rotated CCW.

![Figure 2 Rotary inverted pendulum conventions](.gitbook/assets/rotary\_convention.png)

### Nonlinear Equations of Motion

Instead of using classical mechanics, the Lagrange method is used to find the equations of motion of the system. This systematic method is often used for more complicated systems such as robot manipulators with multiple joints.&#x20;

More specifically, the equations that describe the motions of the rotary arm and the pendulum with respect to the servo motor voltage, i.e. the dynamics, will be obtained using the Euler-Lagrange equation:

$$
\frac{\partial^2L}{\partial t\partial \dot{q_i}} - \frac{\partial L}{\partial q_i} = Q_i
$$

The variables qi are called generalized coordinates. For this system let

$$
q(t)^T =\begin{bmatrix}
\theta(t) & \alpha(t)
\end{bmatrix}
$$

(2.1)

where, as shown in Figure 2, $$\theta(t)$$ is the rotary arm angle and $$\alpha(t)$$ is the inverted pendulum angle. The corresponding velocities are

$$
\dot{q}(t)^T =\begin{bmatrix}
\frac{d\theta(t)}{dt} & \frac{d\alpha(t)}{dt}
\end{bmatrix}
$$

With the generalized coordinates defined, the Euler**-**Lagrange equations for the rotary pendulum system are

$$
\frac{\partial^2L}{\partial t\partial \dot{\theta}} - \frac{\partial L}{\partial \theta} = Q_1
$$

$$
\frac{\partial^2L}{\partial t\partial \dot{\alpha}} - \frac{\partial L}{\partial \alpha} = Q_2
$$

The Lagrangian of the system is described

$$
L = T - V
$$

where T is the total kinetic energy of the system and V is the total potential energy of the system. Thus the Lagrangian is the difference between a system’s kinetic and potential energies.&#x20;

The generalized forces $$Q_i$$ are used to describe the nonconservative forces (e.g. friction) applied to a system with respect to the generalized coordinates. In this case, the generalized force acting on the rotary arm is

$$
Q_1 = \tau - B_r\dot{\theta}
$$

and acting on the pendulum is

$$
Q_2 = -B_p\dot{\alpha}
$$

​See [Table A](lab-5-rotary-inverted-pendulum.md#table-a-main-rotary-servo-base-unit-specifications) in Appendix for a description of the corresponding Rotary Servo parameters (e.g. such as the back-emf constant, $$k_m$$). Our **control variable** is **the input servo motor voltage**, $$V_m$$. Opposing the applied torque is the viscous friction torque, or viscous damping, corresponding to the term $$B_r$$. Since the pendulum is not actuated, the only force acting on the link is the damping. The viscous damping coefficient of the pendulum is denoted by $$B_p$$.&#x20;

The Euler-Lagrange equations is a systematic method of finding the equations of motion, i.e. EOMs, of a system. Once the kinetic and potential energy are obtained and the Lagrangian is found, then the task is to compute various derivatives to get the EOMs. After going through this process, the nonlinear equations of motion for the Rotary Pendulum are:

$$
\left (m_pL_r^2 + \frac{1}{4}m_pL_p^2 - \frac{1}{4}m_pL_p^2 cos(\alpha)^2+J_r \right )\ddot{\theta} - \left (\frac{1}{2}m_pL_pL_rcos(\alpha )\right)\ddot{\alpha}
$$

$$
+ \left(\frac{1}{2}m_pL_p^2sin(\alpha)cos(\alpha) \right)\dot{\theta}\dot{\alpha} + \left( \frac{1}{2}m_pL_pL_rsin(\alpha)\right)\dot{\alpha}^2 = \tau - B_r\dot{\theta}
$$

​(2.2)

$$
-\frac{1}{2}m_pL_pL_rcos(\alpha)\ddot{\theta} + \left(J_p + \frac{1}{4}m_pL_p^2 \right) \ddot{\alpha} - \frac{1}{4} m_pL_p^2cos(\alpha)sin(\alpha)\dot{\theta}^2
$$

$$
-\frac{1}{2} m_pL_pgsin(\alpha) = -B_p\dot{\alpha}
$$

(2.3)

The torque applied at the base of the rotary arm (i.e. at the load gear) is generated by the servo motor as described by the equation (2.4). Refer [Table A](lab-5-rotary-inverted-pendulum.md#table-a-main-rotary-servo-base-unit-specifications) in appendix for Rotary Servo parameters.

$$
\tau = \frac{\eta_gK_g\eta_mk_t(V_m - K_gk_m\dot{\theta})}{R_m}
$$

(2.4)

### Linearization

Here is an example of how to linearize a two variable nonlinear function called f(z). Variable z is defined

$$
z^T = \begin {bmatrix}  z_1 & z_2\end {bmatrix}
$$

and $$f(z)$$ is to be linearized about the operating point

$$
z_0^T = \begin {bmatrix}  a & b\end {bmatrix}
$$

The linearized function is

$$
f_{lin} = f(z_0) + \frac{\partial f(z)}{\partial z_1}\Bigr|_{z = z_0} (z_1 - z_0)+\frac{\partial f(z)}{\partial z_2}\Bigr|_{z=z_0}(z_2 - b)
$$

After linearization of equation 2.2 and 2.3 we get:

$$
(m_pL_r^2 +J_r)\ddot{\theta} - \frac{1}{2}m_p L_pL_r\ddot{\alpha} = \tau -B_r\theta
$$

​(2.4)

$$
-\frac{1}{2}m_pL_pL_r\ddot{\theta}+\left(J_p + \frac{1}{4}m_pL_p^2 \right)\ddot{\alpha} - \frac{1}{2}m_pL_pg\alpha = -B_r\dot{\alpha}
$$

(2.5)



### Linear State-space model

The linear state-space equations are

$$
\dot{x} = Ax + Bu
$$

​and

$$
y = Cx + Du
$$

​where x is the state, u is the control input, A, B, C, and D are state-space matrices. For the rotary pendulum system, the state and output are defined

$$
x^T = \begin {bmatrix} \theta & \alpha & \dot{\theta} & \dot{\alpha} \end {bmatrix}
$$

and​

$$
y^T = \begin {bmatrix} x_1 & x_2\end {bmatrix}
$$

In the output equation, only the position of the servo and link angles are being measured. Based on this, the C and D matrices in the output equation are

$$
C = \begin {bmatrix} 1& 0&0&0\\ 0&1&0&0 \end {bmatrix}
$$

$$
D = \begin {bmatrix} 0 \\ 0\end {bmatrix}
$$

Finally, A and B matrices in the $$\dot{x} = Ax + Bu$$​ equation are

$$
A = \frac{1}{J_T} \begin {bmatrix}  0& 0& 1 & 0 \\ 0& 0 & 0 &1 \\ 0 & \frac{1}{4}m_p^2L_p^2L_rg & -(J_p + \frac{1}{4}m_pL_p^2)B_r & -\frac{1}{2}m_pL_pL_rB_p \\ 0 & \frac{1}{2}m_pL_pg(J_r+m_pL_r^2) & -\frac{1}{2}m_pL_pL_rB_r & -(J_r+ m_pL_r^2)B_p \end {bmatrix}
$$

and

$$
B = \frac{1}{J_T} \begin {bmatrix} 0 \\ 0 \\ J_p + \frac{1}{4}m_pL_p^2 \\ \frac{1}{2}m_pL_pL_r \end {bmatrix}
$$

​where,&#x20;

$$
J_T = J_pm_pL_r^2 + J_rJ_p +\frac{1}{4}J_rm_pL_p^2
$$



### ​Pre-lab

1. **(not for summer)** Linearize equation 2.2 and 2.3. Show your work in the report.
2. **(not for summer)** Fit the two linear equations of motions shown in the lab manual into the matrix form shown in&#x20;

{% hint style="info" %}
To start with linearization, variable z for our system can be defined as:

$$z^T = \begin {bmatrix}  \theta & \alpha & \dot{\theta} & \dot{\alpha} & \ddot{\theta} & \ddot{\alpha}   \end {bmatrix}$$
{% endhint %}

### Model Analysis

1. Open _ROTPEN\_ABCD\_eqns\_student.m_ script and complete A, B, C and D state space representation matrix.
2. Run the _setup\_rotpen\_student.m_ script till Load model section. Print out state space matrix and verify values with TA.
3. Find the open-loop poles of the system.

#### Calibration

1. Open q\_rotpen\_mdl\_student Simulink diagram, go to hardware tab and click Build to build the QUARC controller.
2. Turn on the power amplifier.
3. Connect and run.
4. Rotate the arm and the pendulum in the counterclockwise direction and examine the direction of their response. Does the direction of these measurements agree with the modeling conventions given in [Model Convention](lab-5-rotary-inverted-pendulum.md#model-convention)? Explain why or why not
5. Go to the Rotary Pendulum Interface subsystem block.
6. The Source block called u (V) in q\_rotpen\_mdl\_student Simulink diagram is the control input (step input). When you set u (V) to 1 V, the rotary arm must move according to the model conventions that were defined in [Model Convention](lab-5-rotary-inverted-pendulum.md#model-convention). The Direction Convention Gain block is currently set to 0. Change this value such that the model conventions are adhered to. Plot the rotary arm response and the motor voltage in a Matlab figure when 1 V is applied.
7. Save the data.

{% hint style="success" %}
Note: When the controller stops, the last 10 seconds of data is automatically saved in the Matlab workspace to the variables data\_theta and data\_Vm. The time is stored in data\_alpha(:,1) vector, the pendulum angle is stored the data\_alpha(:,2) vector, and the control input is in the data\_Vm(:,2) structure.
{% endhint %}

### Lab Report

1. Include state space matrix.
2. Include open-loop poles of the system.

## Balance Control

{% hint style="danger" %}
For summer this section is skipped
{% endhint %}

Control gain is provided below:

$$
K = \begin {bmatrix}-5.26&28.16&-2.76&3.22 \end {bmatrix}
$$

## Swing up control

In this section a nonlinear, energy-based control scheme is developed to swing the pendulum up from its hanging, downward position. The swing-up control described herein is based on the strategy outlined in \[3]. Once upright, the control developed to balance the pendulum in the upright vertical position can be used.

### Pendulum Dynamic

The dynamics of the pendulum can be redefined in terms of pivot acceleration as

$$
J_p\ddot{\alpha} + \frac{1}{2}m_pgL_psin(\alpha) = \frac{1}{2} m_pL_p u cos(\alpha)
$$

​(4.1)

The pivot acceleration, u, is the linear acceleration of the pendulum link base. The acceleration is proportional to the torque of the rotary arm and is expressed as

$$
\tau = m_rL_ru
$$

(4.2)

### ​Energy Control

If the arm angle is kept constant and the pendulum is given an initial position it would swing with constant amplitude. Because of friction there will be damping in the oscillation. The purpose of energy control is to control the pendulum in such a way that the friction is constant.&#x20;

The potential and kinetic energy of the pendulum is

$$
E_p = \frac{1}{2}m_pgL_p(1-cos(\alpha))
$$

​(4.3)

and

$$
E_k = \frac{1}{2}J_p\dot{\alpha}^2
$$

In the potential energy calculation, we assume the center of mass to be in the center of the link, i.e. $$\frac{L_p}{2}$$ . Adding the kinetic and potential energy together give us the total pendulum energy​

$$
E = \frac{1}{2}J_p\dot{\alpha}^2 + \frac{1}{2}m_pgL_p(1-cos(\alpha))
$$

​(4.4)

Taking its time derivative we get

$$
\dot{E} = \dot{\alpha}\left(J_p\ddot{\alpha} + \frac{1}{2}m_pgL_psin(\alpha) \right)
$$

(4.5)

​To introduce the pivot acceleration u and eventually, our control variable, solve for sin α in Equation 4.1 to obtain

$$
sin(\alpha) = \frac{1}{m_pgL_p}(-2J_p\ddot{\alpha} + m_pL_pucos(\alpha))
$$

​Substitute this into $$\dot{E}$$, found in Equation 4.5, to get

$$
\dot{E} = \frac{1}{2}m_pL_pu\dot{\alpha}cos(\alpha)
$$

One strategy that will swing the pendulum to a desired reference energy Er is the proportional control

$$
u = (E - E_r)\dot{\alpha}cos(\alpha)
$$

By setting the reference energy to the pendulum potential energy, i.e. Er = Ep, the control will swing the link to its upright position. Notice that the control law is nonlinear because the proportional gain depends on the pendulum angle, α, and also notice that the control changes sign when $$\dot{\alpha}$$ changes sign and when the angle is ±90 degrees.&#x20;

For energy to change quickly the magnitude of the control signal must be large. As a result, the following swing-up controller is implemented

$$
u = sat_{u,max} (\mu(E - E_r)sign(\dot{\alpha cos(\alpha)}))
$$

(4.6)

where μ is a tunable control gain and sat\__umax function saturates the control signal at the maximum acceleration of the pendulum pivot, u\__max. Taking the sign of $$\dot{\alpha}cos \alpha$$ allows for faster switching.

In order to translate the pivot acceleration into servo voltage, first solve for the voltage in Equation 2.4 to get

$$
V_m = \frac{\tau R_m}{\eta_gK_g\eta_mk_t} + K_gk_m\dot{\theta}
$$

Then substitute the torque-acceleration relationship given in Equation 4.2 to obtain the following

$$
V_m = \frac{R_mm_rL_ru}{\eta_gK_g\eta_mk_t} + K_gk_m\dot{\theta}
$$

​(4.7)

### Self-Erecting Control

The energy swing-up control can be combined with the balancing control in Equation 3.11 to obtain a control law which performs the dual tasks of swinging up the pendulum and balancing it. This can be accomplished by switching between the two control systems.

Basically the same switching used for the balance control in Equation 3.12 is used. Only instead of feeding 0 V when the balance control is not enabled, the swing-up control is engaged. The controller therefore becomes

$$
u = \begin {cases}K(x_d - x) & |x_2| < \epsilon \\ sat_{u,max}(\mu(E-E_r)sign(\dot{\alpha}cos(\alpha))) & otherwise \end {cases}
$$

​(4.8)

### Pre-lab calculation

1. Evaluate the potential energy of the pendulum when it is in the downward and upright positions.
2. Compute the maximum acceleration deliverable by the Rotary Servo. Assume the maximum equivalent voltage applied to the DC motor is 5 V such that&#x20;

$$
V_m - K_gk_m\dot{\theta} = 5
$$

​(4.9)

{% hint style="info" %}
For 2) find the maximum torque using equation 2.4 and with maximum torque use equation 4.2 to find the maximum acceleration ($$u_{max}$$)
{% endhint %}

### Procedure

1. Run the _setup\_rotpen.m_ script (with instructor mode (**summer only**))
2. Check the gain K value.
3. Open _q\_rotpen\_swingup\_student\_1_ Simulink diagram.
4. Build, connect and run to check that the balance control runs well. (Manually bring the pendulum upright, the motor will engage at specified range of degree)
5. Open the SwingUp subsystem.
6. Go into EnergyBased SwingUp Control | Pendulum Energy block. Complete the diagram by connecting blocks and changing sign values. Refer to the total energy of the pendulum given in equation 4.4.
7. Build, connect and run. Manually rotate the pendulum up to the upright position. While the inverted pendulum is balancing, **record the total energy reading** displayed in Pen Energy (J) numeric indicator. Is the value as expected?
8. Turn on the power amplifier.
9. Build the controller
10. Connect and run the simulink.
11. Rotate the pendulum up to the upright position. While the inverted pendulum is balancing, **record the total energy reading displayed in Pen Energy (J)** numeric indicator. Is the value as expected?
12. Open _q\_rotpen\_swingup\_student\_2_ Simulink diagram.
13. Implement the energybased swingup controller by completing the EnergyBased SwingUp Control subsystem. Use the Source block with the variable Er as well as the inputs u\_max (m/s2) and mu that are already included. Make sure you are using the full pendulum angle α, i.e. not the upright based angle used in the feedback for the inverted pendulum balance control.
14. Then Build, Connect and Run to start the controller.

The pendulum should be moving back and forth slowly. Gradually increase the umax and/or μ until the pendulum goes up. Do not increase the umax above the maximum acceleration you found for the servo in Section 4.2. When the pendulum swings up to the vertical upright position, the balance controller should engage and balance the link. Show the response of the arm and pendulum angles as well as the control voltage and record the swingup parameters. Did the swingup behave with the parameters you expected?

10\. Save the data.



### Lab Report

1. Swing up response (theta, alpha u(V))
2. Er , u\_max, mu used to bring the pendulum up.





## Appendix

### Table A Main Rotary Servo Base Unit Specifications

|      |                                                               |                              |
| ---- | ------------------------------------------------------------- | ---------------------------- |
| Vnom |  Motor nominal input voltage                                  |  6.0 V                       |
| Rm   |  Motor armature resistance                                    |  2.6 Ω ± 12%                 |
| Lm   |  Motor armature inductance                                    |  0.18 mH                     |
| kt   |  Motor current-torque constant                                |  7.68 × 10−3 N-m/A ± 12%     |
| km   |  Motor back-emf constant                                      |  7.68 × 10−3 V/(rad/s) ± 12% |
| Kg   |  High-gear total gear ratio                                   | 70                           |
| ηm   |  Motor efficiency                                             |  0.69 ± 5%                   |
| ηg   |  Gearbox efficiency                                           |  0.90 ± 10%                  |
| Jm   |  rotor Rotor moment of inertia                                |  3.90 × 10−7 kg-m2 ± 10%     |
| Jeq  |  High-gear equivalent moment of inertia without external load |  1.823 × 10−3 kg-m2          |
| Beq  |  High-gear Equivalent viscous damping coefficient             |  0.015 N-m/(rad/s)           |
| mb   |  Mass of bar load                                             |  0.038 kg                    |
| Lb   |  Length of bar load                                           |  0.1525 m                    |
| md   |  Mass of disc load                                            |  0.04 kg                     |
| rd   |  Radius of disc load                                          |  0.05 m                      |
| mmax |  Maximum load mass                                            |  5 kg                        |
| fmax |  Maximum input voltage frequency                              |  50 Hz                       |
| Imax |  Maximum input current                                        |  1 A                         |
| ωmax |  Maximum motor speed                                          |  628.3 rad/s                 |

## Reference

\[3] K. J. Åström and K. Furuta. Swinging up a pendulum by energy control. 13th IFAC World Congress, 1996. ROTARY PENDULUM Workbook INSTRUCTOR v

