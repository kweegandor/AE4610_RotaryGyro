---
description: >-
  This manual documents the modelling of an inverted pendulum, developing a full
  state feedback control using LQR to balance the unstable equilibrium and
  evaluatin the LQR on the hardware setup.
---

# Inverted Pendulum

## Introduction

### Proportional (Derivative) Control Determined through LQR Analysis

This experiment will demonstrate the control of a system using **full state feedback**, where the control system is designed using the principles of Linear Quadratic Regulator (LQR) theory. This is an analysis technique that would be seen in more detail during advanced studies. You are not required to completely understand the derivation of the method itself. However, _you should understand how to formulate the required matrices (Q and R) and why the overall goal (to minimize the cost, J) leads to the requirements on those matrices, as well as the reasoning behind the formulation of the cost equation_, which serves as the purpose behind the method’s derivation.    &#x20;

![Figure 7.1: Inverted pendulum mechanism \[Image source: https://grabcad.com/library/inverted-pendulum-1\]](../.gitbook/assets/lab6.JPG)

### Linear Quadratic Theory (LQR)

Consider the system

$$
\dot{X}=AX+BU \tag{7.1}
$$

where $$X$$ is a  state vector and $$U$$ is a $$m \times1$$ control input vector. Therefore, $$A$$ and $$B$$ are matrices of dimensions  $$n\times n$$ and $$n\times m$$ , respectively. Let us suppose that we wish to design a state-feedback controller

$$
U=-KX \tag{7.2}
$$

such that the closed-loop system

$$
\dot{X}=(A-BK)X \tag{7.3}
$$

is stable. Here $$K$$ is a matrix of gains of dimensions $$m\times n$$ . In achieving closed-loop stability, we wish to use as little control effort as possible. We also wish to avoid large overshoot of the trajectories during the transients. One way to achieve both of these objectives is to find a control law which minimizes the following quadratic cost

$$
J=\int_{0}^\infty(X^TQX+U^TRU) dt \tag{7.4}
$$

where $$Q$$ and $$R$$ are weighing matrices of appropriate dimensions. In order for the previous problem to make sense we must ensure that the two terms in the integral are positive. The requirement that

$$
X^TQX\geq0 \tag{7.5}
$$

for all vectors $$X$$ is satisfied if the matrix $$Q$$ is **positive semi-definite** (all its eigenvalues are greater than or equal to zero). The requirement that

$$
U^TRU>0 \tag{7.6}
$$

for all vectors $$U$$ is satisfied if the matrix $$R$$ is **positive definite** (all its eigenvalues are greater than zero). This optimization problem has a closed-form solution. The optimal gain matrix $$K$$ is given by

$$
K=R^{-1}B^TP \tag{7.7}
$$

where the $$n\times n$$ matrix $$P$$ is the positive definite solution of the following **Algebraic Riccati Equation** (ARE):

$$
0=A^TP+PA-PBR^{-1}B^TP+Q \tag{7.8}
$$

### Example of a Simple Mass/Spring/Damper System

We will demonstrate the previous LQR design using a simple mass-spring-damper system as shown in Fig. 7.2. The equation of motion is

$$
m\ddot{x}+c\dot{x}+kx=F
$$

or in state-space form

$$
\begin{bmatrix}
\dot{x}_1 \\[1ex]
\dot{x}_2
\end{bmatrix} = \begin{bmatrix}
0 & 1 \\[1ex]
-\frac{k}{m} & -\frac{c}{m} 
\end{bmatrix} \begin{bmatrix}
x_1 \\[1ex]
x_2
\end{bmatrix} + \begin{bmatrix}
0 \\[1ex]
\frac{1}{m}
\end{bmatrix}F
$$

where $$x_1=x$$ and $$x_2=\dot{x}$$ . Let $$m=1,\space k=2$$ and $$c=1$$ . The system matrices are

$$
A = \begin{bmatrix}
0 & 1 \\[1ex]
-2 & -1
\end{bmatrix}, \qquad B = \begin{bmatrix}
0 \\[1ex]
1
\end{bmatrix}
$$

![Figure 7.2: A simple mass / spring / damper system](<../.gitbook/assets/image (20).png>)

Choose the weighting matrices as

$$
Q = \begin{bmatrix}
12 & 0 \\[1ex]
0 & 4
\end{bmatrix}, \qquad R = 1
$$

The Riccati equation gives

$$
\begin{bmatrix}
0 \! & \! 0 \\[0.5ex]
0 \! & \! 0
\end{bmatrix} \! = \! \begin{bmatrix}
0 \! & \! -2 \\[0.5ex]
1 \! & \! -1
\end{bmatrix} \! \begin{bmatrix}
p_{11} \! & \! p_{12} \\[0.5ex]
p_{21} \! & \! p_{22}
\end{bmatrix} \! + \! \begin{bmatrix}
p_{11} \! & \! p_{12} \\[0.5ex]
p_{21} \! & \! p_{22}
\end{bmatrix} \!  \begin{bmatrix}
0 \! & \! 1 \\[0.5ex]
-2 \! & \! -1
\end{bmatrix} \! - \! \begin{bmatrix}
p_{11} \! & \! p_{12} \\[0.5ex]
p_{21} \! & \! p_{22}
\end{bmatrix} \! \begin{bmatrix} \! 
0 \\[0.5ex]
1
\end{bmatrix} \! \begin{bmatrix}
0 \! & \! 1
\end{bmatrix} \! \begin{bmatrix}
p_{11} \! & \! p_{12} \\[0.5ex]
p_{21} \! & \! p_{22}
\end{bmatrix} \! + \!\begin{bmatrix}
12 \! & \! 0 \\[0.5ex]
0 \! & \! 4
\end{bmatrix}
$$

Expanding, we obtain

$$
p_{12}^2+4p_{12}-12=0\\[5pt]
p_{11}-2p_{22}-p_{12}-p_{12}p_{22}=0\\[5pt]
p_{22}^2-2p_{12}+2p_{22}-4=0
$$

Solving these equations, we get

$$
p_{11}=10, \quad p_{12}=2, \quad p_{22}=2
$$

or that

$$
P=\begin{bmatrix}
10 & 2\\
2 & 2
\end{bmatrix}
$$

Notice that the solution matrix P is positive definite. Its eigenvalues are $$\lambda_{1,2}=10.472,\space1.528$$.&#x20;

The optimal gain matrix is given by

$$
K=R^{-1}B^TP\\[5pt]
K=\begin{bmatrix}
0 & 1 \end{bmatrix}
\begin{bmatrix}
10 & 2 \\
2 & 2 \end{bmatrix} = \begin{bmatrix}
2 & 2 \end{bmatrix}
$$

The closed-loop system matrix is

$$
A-BK=\begin{bmatrix}
0&1\\
-4 & -3 \end{bmatrix}
$$

and the eigenvalues are $$\lambda_{1,2}=-1.5\pm i1.323.$$&#x20;

{% hint style="info" %}
The closed-loop system is stable.
{% endhint %}

## Control of an Inverted Pendulum

### Objective

The purpose of this experiment is to design a balance control using the principles of linear quadratic regulator (LQR) theory for an inverted pendulum mechanism.

### Equipment Required

* [ ] Inverted Pendulum mechanism
* [ ] Q8-USB interface board
* [ ] MATLAB, SIMULINK and QUARC software.
* [ ] Power module for the PCI Multi-Q board and the pendulum mechanism.

### Introduction

In this lab, we demonstrate a control design using modern “state-space” methods. The plant consists of an inverted pendulum on a cart.  The plant has two distinct equilibrium points of which one is stable and the other is unstable. In this experiment, we will show how the plant in its unstable configuration can be controlled using a suitable controller. The methodology used to design the control law is based on the linear quadratic regulator (LQR) theory. The inverted pendulum mechanism to be used in this lab is shown in Fig. 7.3.

![Figure 7.3: Inverted Pendulum mechanism in the lab](<../.gitbook/assets/image (88).png>)

### Mathematical Model of the Inverted Pendulum Mechanism

Consider an inverted pendulum of mass $$m_p$$ and length $$2l_p$$ connected on a sliding cart of mass $$m_c$$ as shown in Fig. 7.4.

![Figure 7.4: Inverted pendulum mechanism](<../.gitbook/assets/image (37).png>)

The angle $$\theta$$ is measured from the up-right position. We can apply a force to the cart through the cartwheel motors. We wish to design a controller to balance the pendulum to the upright position $$\theta=0$$. Note that the problem of balancing a pendulum is akin to the problem of stabilizing a rocket during ascent.

![Figure 7.5: Powered rocket during ascent](<../.gitbook/assets/image (62).png>)

The equations of motion for the system can be easily derived using the free-body diagrams shown in Fig. 7.6 as follows (here we assume that the track on which the cart moves without sliding):

$$
\begin{equation}
\tag{7.9}
\begin{aligned}(m_p+m_c)\ddot{x} + m_p\ddot{\theta}l_p\cos(\theta) -m_p\dot{\theta}^2l_p\sin(\theta) &= F 
\\[5pt]
m_pl_p\cos(\theta)\ddot{x} + I\ddot{\theta} + m_p\ddot{\theta}l_p^2 - m_pgl_p\sin(\theta) &= 0
\end{aligned}
\end{equation}
$$

where $$I$$ is the moment of inertia about the pendulum's mass center given by $$I=\cfrac{m_pl_p^2}{3}$$.

![Figure 7.6: Free Body Diagrams](<../.gitbook/assets/image (65).png>)

This system has two equilibria $$(\dot{x}=\ddot{x}=\dot{\theta}=\ddot{\theta}=0)$$ which are $$(x_{e1},\theta_{e1})=(0,\pi)$$ and $$(x_{e2}, \theta_{e2})=(0,0)$$.&#x20;

The first equilibrium corresponds to the down position (stable) and the second equilibrium corresponds to the upright position (unstable) of the pendulum. Linearizing the previous equations about the equilibrium of interest $$(0, 0)$$ we obtain the following equations:

$$
\frac{d}{dt} \begin{bmatrix}
x \\[1ex]
\theta \\[1ex]
\dot{x} \\[1ex]
\dot{\theta}
\end{bmatrix} = \begin{bmatrix}
0 & 0 & 1 & 0 \\[1ex]
0 & 0 & 0 & 1 \\[1ex]
0 & -\cfrac{3m_pg}{m_p+4m_c} & 0 & 0 \\[1ex]
0 & \cfrac{3(m_p+m_c)g}{(m_p+4m_c)l_p} & 0 & 0 
\end{bmatrix} \begin{bmatrix}
x \\[1ex]
\theta \\[1ex]
\dot{x} \\[1ex]
\dot{\theta}
\end{bmatrix} + \begin{bmatrix}
0 \\[1ex]
0 \\[1ex]
\cfrac{4}{m_p+4m_c} \\[1ex]
-\cfrac{3}{(m_p+4m_c)l_p}
\end{bmatrix}F \tag{7.10}
$$

The input force $$F$$ in the above equations can be written in terms of the input voltage $$V$$ applied to the motor of the cart and the cart velocity $$\dot{x}$$ as shown below.

First, we write the motor equation as

$$
V=I_mR_m+K_mK_g\omega_g=I_mR_m+K_mK_g\frac{\dot{x}}{r}
$$

or

$$
I_m=\frac{V}{R_m}-\frac{K_mK_g}{R_m}\frac{\dot{x}}{r} \tag{7.11}
$$

where

* $$V$$ : voltage applied to motor (Volts)
* $$I_m$$ : current in motor (Amp)
* $$K_m$$ : back EMF constant (Volt/(rad/sec))
* $$K_g$$ : gear ratio in motor gearbox
* $$R_m$$ : motor armature resistance (Ohms)
* $$\omega_g$$ : angular velocity of the wheel (rad/sec)
* $$\dot{x}$$ : cart velocity (m/sec)
* $$r$$ : radius of motor pinion that meshes with the track (m)

The torque generated at the output of the motor is given by

$$
T_m=K_tK_gI_m \tag{7.12}
$$

where $$K_m$$ is the motor torque constant given by

$$
K_t=k K_m \tag{7.13}
$$

where $$k$$ is the conversion factor from volt-amp to N-m given by

$$
k = 1.3558 \mathrm{~N-m/s/(volt-amp)}
$$

Thus, the force transmitted to the cart via the pinion is

$$
F=\frac{T_m}{r}=\frac{K_tK_gV}{rR_m}-\frac{K_tK_gK_mK_g}{R_m}\frac{\dot{x}}{r^2}=\frac{kK_mK_gV}{rR_m}-\frac{k(K_mK_g)^2}{R_m}\frac{\dot{x}}{r^2} \tag{7.14}
$$

and on substituting the values of

$$
K_m=0.00767~ \mathrm{volt-amp/rad/s},\quad K_g=3.7
\\
k=1.3558~\mathrm{N-m/s/(volt-amp)},\quad r=0.00635~\mathrm{m},\quad R_m=2.6\space \mathrm{ohms}
$$

we get

$$
F=2.3305V-10.4152\dot{x} \tag{7.15}
$$

Letting  $$a_1=2.3305$$ and $$a_2=10.4152$$ , the input force F can be written as&#x20;

$$
F=a_1V-a_2\dot{x} \tag{7.16}
$$

Substitution of the above expression for F in the linearized equations results in

$$
\frac{d}{dt} \begin{bmatrix}
x \\[1ex]
\theta \\[1ex]
\dot{x} \\[1ex]
\dot{\theta}
\end{bmatrix} = \begin{bmatrix}
0 & 0 & 1 & 0 \\[1ex]
0 & 0 & 0 & 1 \\[1ex]
0 & -\cfrac{3m_pg}{m_p+4m_c} & -\cfrac{4a_2}{m_p+4m_c} & 0 \\[3ex]
0 & \cfrac{3(m_p+m_c)g}{(m_p+4m_c)l_p} & \cfrac{3a_2}{(m_p+4m_c)l_p} & 0 
\end{bmatrix} \begin{bmatrix}
x \\[1ex]
\theta \\[1ex]
\dot{x} \\[1ex]
\dot{\theta}
\end{bmatrix} + \begin{bmatrix}
0 \\[1ex]
0 \\[1ex]
\cfrac{4a_1}{m_p+4m_c} \\[3ex]
-\cfrac{3a_1}{(m_p+4m_c)l_p}
\end{bmatrix}F \quad \tag{7.17}
$$

The above set of linear differential equations is of the form $$\dot{X}=AX+BV$$ , where $$X=[x,~\theta,~ \dot{x},~\dot{\theta}]^\top$$. We seek a linear control law of the form  $$V=-KX$$, where $$K$$ is a $$1 \times 4$$ vector of gains.

### Linear Quadratic Regulator (LQR) Design

The LQR formulation provides a linear control law

$$
V=-KX \tag{7.18}
$$

which minimizes the quadratic cost given by

$$
J=\int_0^{\infty}(X^T QX+V^T RV) dt \tag{7.19}
$$

for some user-defined matrices $$Q=Q^T$$ and  $$R=R^T$$ . It is assumed that $$Q$$ is positive semi-definite (all the eigenvalues are non-negative) and $$R$$ is positive definite (all the eigenvalues are positive). Here $$V$$ is the input voltage, $$X$$ given by

$$
X=\begin{bmatrix}
x\\[1ex]
\theta \\[1ex]
\dot{x} \\[1ex]
\dot{\theta} \end{bmatrix} \tag{7.20}
$$

is the state vector, and $$K$$ given by

$$
K = [ k_1 ~~ k_2 ~~ k_3 ~~ k_4] \tag{7.21}
$$

is the controller gain vector. Note that the input voltage $$V$$ is scalar and therefore, $$R$$ is also scalar. In practice, the weighting matrices $$Q$$ and $$R$$ are chosen to be diagonal. A simple way for choosing  $$Q$$ and $$R$$ is the following (Bryson's rule):&#x20;

Assuming the time-domain design specifications are:

$$
|V|\leq\bar{V}, \quad|X_i|\leq\bar{X_i}
$$

one can choose

$$
R=\frac{1}{\bar{V}^2},\quad Q=\mathrm{diag} \left(\frac{1}{\bar{X}_i^2} \right) \tag{7.22}
$$

Further tuning of $$R$$ and $$Q$$ may be required for improving the controller performance. Note that increasing $$R$$ means increasing the penalty on the control input (using less control effort). Similarly, increasing $$Q$$ means increasing the penalty on the states (states reaching equilibrium position quickly and with less overshoot).

The block diagram of the experimental setup in the lab is shown in Fig. 7.7.

![Fig 7.7: Block diagram of the experimental set-up](<../.gitbook/assets/image (99).png>)

(Note: Conventional error signal shown, but $$u=-K\Delta X, \Delta X=X-X_C$$)

where $$x$$ is position of the cart, $$\dot{x}$$ is velocity of the cart, $$\theta$$ is angular position of the rod from the vertical, $$\dot{\theta}$$ is angular velocity of the rod, and $$K$$ is a vector of feedback gains $$K = [ k_1 ~ k_2 ~ k_3 ~ k_4]$$.

## Controller Design Procedure

1.  Obtain the A and B matrices of the system using $$m_p=0.21\space kg,\space m_c=0.815\space kg$$ , and $$l_p=0.305\space m$$ .


2.  Choose $$Q\geq0$$ and $$R>0$$ based on the following:

    Assume that:

    1. $$0.4m\leq|x_{max}|\leq0.5m$$~~~~
    2. ~~~~$$\frac{\pi}{6}rad\leq|\theta_{max}|\leq\frac{\pi}{4}rad$$
    3.  $$6\space volts\leq|V_{max}|\leq10\space volts$$&#x20;



    No constraints on $$\dot{x}_{\max}$$  and $$\dot{\theta}_{max}$$ . (They can go to infinity…)

    According to Bryson’s rule, this will result in the following $$Q$$ and $$R$$ matrices:

    &#x20;$$Q= \begin{bmatrix} 4 & 0&0&0 \\ 0&1.6211&0&0 \\ 0&0&0&0 \\ 0&0&0&0 \end{bmatrix}$$ and $$R= [0.01]$$&#x20;


3.  Using the `lqr` command in MATLAB, calculate the optimal feedback gain vector K. The `lqr` command basically solves the Algebraic Ricatti Equation that you met earlier. `>> [k,s,e] = lqr(A,B,Q,R);`


4.  Compute the open-loop and closed-loop eigenvalues of the system and comment on the stability of the open and closed-loop systems.


5. Compute the natural frequencies and damping ratios of the closed-loop system.

## Controller Implementation and Evaluation

{% hint style="danger" %}
CAUTION: Always be ready to stop the experiment if the cart goes unstable. This can be done either by pressing the stop button in the Simulink or turning the power off.
{% endhint %}

1.  Turn the power on.


2.  Open the MATLAB and locate the file **InvPendulum\_LQR.mdl** under the path **C:\AE4610\_Controls\_Lab\Inverted\_Pendulum** and open it. This is the block diagram for this part of the experiment.


3.  To build the model, click the down arrow on **Monitor & Tune** under the Hardware tab and then click **Build** **for monitoring** ![](<../.gitbook/assets/image (117).png>). This generates the controller code.


4.  Switch the Command Signal block to 0.


5.  Enter the gains you obtained in the corresponding gain blocks.


6.  Put the cart right in the middle of the track and hold up the pendulum fixed and straight. Do not release it until the controller is on.


7.  Double click on the scopes and open then.


8.  Press **Connect** <img src="../.gitbook/assets/image (121).png" alt="" data-size="line"> button under Monitor & Tune and then press **Start** <img src="../.gitbook/assets/image (119).png" alt="" data-size="line"> **** .


9.  Gently disturb the pendulum to see the effect of the controller and then stop the simulation.


10. Switch the command signal to step input. It should make the cart to move 20 cm to the right after 10 seconds.


11. Save the data with a descriptive name. **DO NOT SAVE THE MODEL**.


12. Turn the POWER OFF

### Analysis

*   Individual: Include the results from the controller design procedure Steps 1-5.


* Group: Build a SIMULINK block diagram using Fig. 7.7 and obtain the simulated response to the same command input used in the experiment. Compare the simulated response with the experimental response and explain any differences between them.

{% hint style="info" %}
Average the first 5 seconds of the experimental data to obtain initial conditions for your state-space block in SIMULINK&#x20;
{% endhint %}

