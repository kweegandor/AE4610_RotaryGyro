# Lab 3: Rotary Flexible Link

## Objective

The purpose of this experiment is to understand the existence of vibrations in a rotary flexible link and the resulting mode shapes. Any beam-like structure exhibits vibrations either due to external changing loads or due to reorientation via actuators. The experiment deals with the modeling and identification of modal frequencies and mode shapes of free vibrations for a rotary flexible link. This analysis is particularly useful to understand structural vibrations and modes and how to contain them in real-world applications like aircraft and spacecraft structures as well as robotic links/manipulators.

![Image Credit: Quanser](<.gitbook/assets/image (7).png>)

## Equipment Required

*   [ ] Flexible link plant (Quanser FLEXGAGE module)


*   [ ] Power amplifier (Quanser VoltPaq-X2)


*   [ ] Data acquisition board (Quanser Q2-USB)


*   [ ] Rotary servo plant (Quanser SRV02)


* [ ] MATLAB and SIMULINK

## Modeling and Vibration Analysis

This experiment involves system identification and modeling of the flexible link. The objective is to find the stiffness of the flexible link and conduct a frequency sweep across a range to determine the structural frequencies and mode shapes of the link.

### Rotary Flexible Link Model

This experiment is performed using the Quanser Rotary Flexible Link mounted on an SRV-02 servo motor.  This system, shown in Fig. 1, consists of an electromechanical plant, where a flexible link is rotated using a servo motor. The base of the flexible link is mounted on the load gear of the servo motor system. The servo angle, $$\theta$$, increases positively when it rotates counter-clockwise (CCW). The servo (and thus the link) turn in the CCW direction when the control voltage is positive, i.e., $$V_m>0$$ .

![Figure 1. Rotary Flexible Link Setup](<.gitbook/assets/image (71).png>)

The main components of the setup are labeled in Figs. 2 and 3 and are listed in Table 1.

**Table 1. Setup Components**

| No. | Component                 |
| --- | ------------------------- |
| 1   | SRV02 Plant (Servo motor) |
| 2   | FLEXGAGE Module           |
| 3   | FLEXGAGE Link             |
| 4   | Strain Gauge              |
| 5   | Strain Gauge Circuit      |
| 6   | Thumbscrews               |
| 7   | Sensor Connector          |
| 8   | OFFSET Potentiometer      |
| 9   | GAIN Potentiometer        |

![Figure 2. FLEXGAGE coupled to SRV02](<.gitbook/assets/image (59).png>)

![Figure 3. Strain Gauge Closeup](<.gitbook/assets/image (27).png>)

The FLEXGAGE module consists of the strain gauge, the strain gauge circuitry, and a sensor connector. The flexible link is attached to this module and the strain gauge is fixed at the root of the link. The module is mounted onto the servo motor, which is the actuator for this system. The strain gauge sensor produces an analog signal proportional to the deflection of the link tip.

The link can be schematically represented as shown in Fig. 4. The flexible link has a total length of $$L_1$$, a mass of $$m_1$$, and its moment of inertia about the center of mass is $$a = J_1$$ . The deflection angle of the link is denoted as $$\alpha$$ and increases positively when rotated CCW.

![Figure 4. Rotary Flexible Link Angles](<.gitbook/assets/image (105).png>)

The complete flexible link system can be represented by the diagram shown in Fig. 5. The control variable is the input servo motor voltage, $$V_m$$ which is proportional to the angular rate of the servo motor. This generates a torque $$\tau$$, at the load gear of the servo that rotates the base of the link, which is given by                                                $$\tau=\displaystyle{\frac{\eta_g K_g\eta_mk_t(V_m-K_gk_m\dot{\theta})}{R_m}}=C_1V_m-C_2\dot{\theta} \tag{1.1}$$​

where the various constants are SRV02 parameters which are mentioned in Table 2.

![Figure 5. Rotary Flexible Link Model](<.gitbook/assets/image (83).png>)

The viscous friction coefficient of the servo is denoted by $$B_{\rm eq}$$. This is the friction that opposes the torque being applied at the servo load gear. $$J_{\rm eq}$$ represents the moment of inertia of the SRV02 when there is no load. The friction acting on the link is represented by the viscous damping coefficient $$B_l$$. The flexible link is modeled as a linear spring with the stiffness $$K_s$$ and with moment of inertia $$J_l$$ .

**Table 2. Setup Parameters**

| $$m_l$$         | Mass of flexible link                                     | $$0.065\ \mathrm{kg}$$                           |
| --------------- | --------------------------------------------------------- | ------------------------------------------------ |
| $$L_l$$         | Length of flexible link                                   | $$0.419 \ \mathrm{m}$$                           |
| $$b_l$$         | Width or breadth of flexible link                         | $$2.083\times10^{-2} \ \mathrm{m}$$              |
| $$h_l$$         | Thickness of flexible link                                | $$8.128\times10^{-4}\ \mathrm{m}$$               |
| $$E$$           | Young’s modulus of flexible link                          | $$200 \ \mathrm{GPa}$$                           |
| $$I$$           | Area moment of inertia of link cross-section              | $$9.32\times10^{-13} \ \mathrm{m^{4}}$$          |
| $$B_{\rm eq}$$  | High-gear viscous damping coefficient of SRV02            | $$0.015\ \mathrm{N.m/(rad/s)}$$                  |
| $$J_{\rm eq}$$  | Equivalent high-gear moment of inertia of SRV02 (no load) | $$0.00208 \ \mathrm{kg.m^{-2}}$$                 |
| $$R_m$$         | Motor armature resistance                                 | $$2.6 \ \mathrm{\Omega}$$                        |
| $$k_t$$         | Motor torque constant                                     | $$7.68\times10^{-3}\ \mathrm{N.m}$$              |
| $$\eta_m$$      | Motor efficiency                                          | $$0.69$$                                         |
| $$k_m$$         | Back-emf constant                                         | $$7.68\times10^{-3}\ \mathrm{V/(rad/s)}$$        |
| $$K_g$$         | High-gear total gearbox ratio                             | $$70$$                                           |
| $$\eta_g$$      | Gearbox efficiency                                        | $$0.90$$                                         |
| $$B_l$$         | Viscous damping coefficient of flexible link              | <mark style="color:red;">To be calculated</mark> |
| $$J_l$$         | Moment of inertia of flexible link about pivoted end      | <mark style="color:red;">To be calculated</mark> |
| $$K_s$$         | Stiffness of flexible link                                | <mark style="color:red;">To be calculated</mark> |
| $$m$$           | Mass per unit length of flexible link                     | <mark style="color:red;">To be calculated</mark> |

### Equations of Motion

The servo and the flexible link can be modeled as a lumped mass system separated by an equivalent spring and damper, which represent the stiffness and damping coefficient of the flexible link, respectively. The equations that describe the motion of the servo and the link with respect to the servo motor torque, i.e., the dynamics, can be obtained using free body diagram (FBD) analysis of the lumped mass moments of inertia ($$J_{\rm eq}$$ and $$J_l$$).

The torque balance on $$J_{\rm eq}$$ yield Eq. (1.2) and the torque balance on $$J_l$$ yield Eq. (1.3).

&#x20;                                                            $$J_{\rm eq}\ddot{\theta}+B_{\rm eq}\dot{\theta}-B_l\dot{\alpha}-K_s\alpha=\tau \tag{1.2}$$                                                    &#x20;

&#x20;                                                               $$J_l\ddot{\theta}+J_l\ddot{\alpha}+B_1\dot{\alpha}+K_s\alpha=0 \tag{1.3}$$                                                     &#x20;

On rearranging Eq. (1.3) to obtain an expression for $$B_l\dot{a}+K_s\alpha$$ and substituting it in Eq. (1.2), the equations of motion (EOM) for the rotary flexible link system can be obtained as

$$(J_{\rm eq}+J_l)\ddot{\theta}+J_l\ddot{\alpha}+B_{\rm eq}\dot{\theta}=\tau \tag{1.4}$$

$$J_l\ddot{\alpha}+J_l\ddot{\theta}+B_l\dot{\alpha}+K_s\alpha=0 \tag{1.5}$$

### Stiffness Determination

The stiffness of the flexible link can be determined from the free oscillation of the link using a second-order model. The free-oscillatory equation of motion of this second-order system is obtained by setting the $$\ddot{\theta}$$ term to zero in Eq. (1.5), i.e., by holding $$\theta$$ constant, which is shown in Fig. 6. The resulting equation will be

$$J_l\ddot{\alpha}+B_l\dot{\alpha}+K_s\alpha=0 \tag{1.6}$$                                                                                                                                   &#x20;

Assuming the initial conditions $$\alpha(0)=\alpha_0$$ and $$\dot{\alpha}(0)=0$$ , the Laplace transform of Eqn. 1.6 yields:

&#x20;                                                                                                                               $$\displaystyle{ A(s)={\frac{\displaystyle{\frac{\alpha_0}{J_l}}}{\displaystyle{s^2+\frac{B_l}{J_l}s+\frac{K_s}{J_l}}}}} \tag{1.7}$$           &#x20;

![Figure 6. Free Oscillation Response](<.gitbook/assets/image (78).png>)

The prototype characterisitic equation for a second-order system is defined as

$$
s^2+2\zeta\omega_ns+\omega_n^2
$$

where $$\zeta$$ is the damping ratio and $$\omega_n$$ is the natural frequency. Equating this to the characteristic equation in Eq. (1.7) yields                                        $$\displaystyle{\omega_n^2=\frac{K_s}{J_l}} \tag{1.8}$$                                                                                                   $$\displaystyle{2\zeta\omega_n = \frac{B_l}{J_l}} \tag{1.9}$$                                                                                                                   &#x20;

where$$J_l$$ represents the moment of inertia of the link about the pivot. This can be calculated approximately by considering the link as a rod rotating about a pivot at one edge $$\displaystyle{(J_l=\frac{m_lL_l^2}{3})}$$. Equations (1.8) and (1.9) can be used to determine the stiffness and damping of the flexible link once the natural frequency and damping ratio are known.

The damping ratio of this second-order system can be found from its response (underdamped system) using the subsidence or decrement ratio given by

$$\displaystyle{\delta=\frac{1}{n-1}\ln{\frac{O_1}{O_n}}} \tag{1.10}$$                                                                   &#x20;

where $$O_1$$ is the peak of the first oscillation and $$O_n$$ is the peak of the _n_th oscillation. Note that $$O_1>O_n$$ , as this is a decaying response (positive damping).

The damping ratio is defined as

&#x20;                                                                            $$\displaystyle{\zeta=\frac{1}{\sqrt{1+(\displaystyle{\frac{2\pi}{\delta})^2}}}} \tag{1.11}$$                                                                        &#x20;

The period of oscillation in a system response can be found using the equation

&#x20;                                                                             $$\displaystyle{T_{\rm osc}=\frac{t_n-t_1}{n-1}} \tag{1.12}$$                                                                        &#x20;

where $$t_n$$ is the time of the $$nth$$ oscillation, $$t_1$$ is the time of the first peak, and $$n$$ __ is the number of oscillations considered.

From this, the damped natural frequency (in rad/s) is

$$\displaystyle{\omega_d=\frac{2\pi}{T_{\rm osc}}} \tag{1.13}$$                                                                                        &#x20;

and the undamped natural frequency is                    $$\displaystyle{\omega_n=\frac{\omega_d}{\sqrt{1-\zeta^2}}} \tag{1.14}$$                                                                       &#x20;

## Part 1

### Experimental Procedure

{% file src=".gitbook/assets/FlexLink_FreeOsc_Q2_USB.slx" %}

1.  Download and open **FlexLink\_FreeOsc\_Q2\_USB.slx**. This is the block diagram for this part of the experiment.


2.  To build the model, click the down arrow on **Monitor & Tune** under the Hardware tab and then click **Build** **for monitoring** ![](<.gitbook/assets/image (117).png>). This generates the controller code.


3.  Open the scope **alpha**.


4.  Turn on the power supply.


5.  Press **Connect** <img src=".gitbook/assets/image (121).png" alt="" data-size="line"> button under Monitor & Tune and hold on to the base to prevent any rotation at the root.


6.  Press **Start** <img src=".gitbook/assets/image (119).png" alt="" data-size="line"> **** and immediately perturb the flexible link. Keep holding the base until the data is collected for the complete run (5 seconds).


7.  Save the link deflection angle data for the free oscillation using the format **FreeOsc\_1** to your folder.


8.  Repeat the steps 6 and 7 two more times for different perturbation locations along the link or different perturbation angles.



### Analysis

1.  Plot the measured angular deflection of the link vs. time for each case.  From the plot, determine the time period of oscillation by considering the first and, say, the fourth or fifth oscillation. Use this information to determine the damped frequency, the undamped frequency, and the stiffness (usage of`findpeak` MATLAB function might be helpful, but not required)


2.  Average the stiffness value of the link for the three sets of data to get a single value of the link stiffness.&#x20;


3. Compare the damped and undamped frequencies and report your observation. What does this signify with respect to the flexible link damping?



### Calculations to be done before Part 2

1.  Using the system parameters provided in Table 2, determine the mass per unit length of the beam $$(m)$$.


2. Using Eqn. 1.26 and the values of necessary system parameters, calculate the first and second modal frequencies.



## Modal Frequencies and Mode Shapes

The flexible link can be considered as a thin continuous (uniform) cantilever beam anchored at one end and free at the other end. Using the Euler-Bernoulli beam theory, the equation of motion can be written as                                                          $$\displaystyle{EI\frac{\mathrm{\partial}^4Y(x,t)}{\mathrm{\partial }x^4}+m\frac{\partial ^2Y(x,t)}{\partial t^2}=q(x,t)} \tag{1.15}$$                                                    &#x20;

where $$E$$ is the modulus of rigidity of beam material (assumed constant), $$I$$ is the area moment of inertia of the beam cross-section (assumed constant), $$Y(x,t)$$ is the displacement in $$y$$ direction at a distance $$x$$ from the fixed end at time $$t$$, $$\omega$$ is the circular natural frequency, $$m$$ is the mass per unit length ($$m=\rho A$$, $$\rho$$ is the material density, $$A$$ is the cross-section area), $$x$$ is the distance measured from the fixed end and $$q$$ is the external applied force per unit length.

Also, the angle of deflection $$\alpha$$ is related to the displacement as                                                                                 $$\displaystyle{\alpha=\frac{\partial Y}{\partial x}} \tag{1.16}$$                                                                              &#x20;

The general solution to Eq. (1.15) can be obtained using separation of variables, as in Eq. (1.17) below. &#x20;

&#x20;                                                                 $$Y(x,t)=v(x)s(t) \tag{1.17}$$                                                                     &#x20;

Substituting (1.17) in (1.15), setting $$q(x,t)=0$$ and rearranging gives

&#x20;                                                       $$\displaystyle{\frac{\mathrm{d}^4 v(x)}{\mathrm{d}x^4}s(t)+v(x)\frac{m}{EI}\frac{\mathrm{d}^2s(t)}{\mathrm{d}t^2}=0} \tag{1.18}$$                                                           &#x20;

Equation (1.18) can be rewritten as

&#x20;$$\displaystyle{\frac{\displaystyle{\frac{\mathrm{d}^4 v(x)}{\mathrm{d}x^4}}}{v(x)}=-\frac{m}{EI}\frac{\displaystyle{\frac{\mathrm{d}^2s(t)}{\mathrm{d}t^2}}}{s(t)}} \tag{1.19}$$

Since the left side of Eq. (1.19) is only a function of $$x$$ and the right side of Eq. (1.19) is only a function of $$t$$, they both must equal a constant. Let this constant be $$\beta^4$$. Thus, Eq. (1.19) can be written as the following two equations

&#x20;                                                                      $$\displaystyle{\frac{\mathrm{d}^4v(x)}{\mathrm{d}x^4}-\beta^4v(x)=0} \tag{1.20}$$                                                                &#x20;

&#x20;                                                                                $$\displaystyle{\frac{\mathrm{d}^2s(t)}{\mathrm{d}t^2}+\frac{m}{EI}\beta^4s(t)=0} \tag{1.21}$$                                                  &#x20;

&#x20;The solution of Eq. (1.20) gives the displacement$$v$$(as a function of$$x$$), which will be of the form     &#x20;

$$v(x)=a\sin(\beta x)+b\cos(\beta x)+c\sinh(\beta x)+d\cosh(\beta x) \tag{1.22}$$

where $$a,b,c$$ and $$d$$ are unknown constants. The general solution of Eq. (1.21) is given by

&#x20;                                                             $$s(t)=g\sin(\omega t)+h\cos(\omega t) \tag{1.23}$$                                                       &#x20;

where $$\omega=\displaystyle{\sqrt{\frac{EI}{m}}\beta^2}$$ , $$g$$ and $$h$$ are unknown constants.

The constants in Eq. (1.22) are determined from four boundary conditions, while the constants in Eq. (1.23) are determined from two initial conditions.

For a clamped-free or cantilever beam, the geometric boundary conditions are

$$
Y(x=0, t)=0\to v(x=0)=0
$$

$$
\displaystyle{\left.\frac{\partial Y}{\mathrm{\partial} x}\right|_{(x=0,t)}=0\to \left.\frac{\mathrm{d}v}{\mathrm{d}x}\right|_{x=0}=0}
$$

and the natural boundary conditions are

$$
\displaystyle{M(x=L,t)=0\to EI\left.\frac{\partial^2Y}{\partial x^2}\right|_{x=L,t}=0\to \left.\frac{\mathrm{d}^2v}{\mathrm{d}x^2}\right|_{x=L}=0}
$$

$$
\displaystyle{V(x=L,t)=0\to -EI\left.\frac{\partial^3Y}{\partial x^3}\right|_{(x=L,t)}=0\to \left.\frac{\mathrm{d}^3v}{\mathrm{d}x^3}\right|_{(x=L)}=0}
$$

where $$M$$ represents the bending moment and $$V$$ represents the shear force.

On substitution of the geometric boundary conditions at $$x=0$$ in Eq. (1.22) and its derivative, the following relations can be obtained

$$
v(x=0)=0\to d=-b
$$

$$
\displaystyle{\left.\frac{\mathrm{d}v}{\mathrm{d}x}\right|_{(x=0)}=0\to c=-a}
$$

Hence, Eq. (1.22) becomes

&#x20;                                       $$v(x)=a[\sin(\beta x)-\sinh(\beta x)]+b[\cos(\beta x)-\cosh(\beta x)] \tag{1.24}$$                      &#x20;

On further substitution of the natural boundary conditions at $$x=L$$ in the derivatives of Eq. (1.24) yields

$$
\left.\frac{\mathrm{d}^2v}{\mathrm{d}x^2}\right|_{(x=L)}=0\to a[\sin(\beta L)+\sinh(\beta L)]+b[\cos(\beta L)+\cosh(\beta L)]=0
$$

$$
\displaystyle{\left.\frac{\mathrm{d}^3v}{\mathrm{d}x^3}\right|_{(x=L)}=0\to a[\cos(\beta L)+\cosh(\beta L)]-b[\sin(\beta L)-\sinh(\beta L)]=0}
$$

or

$$
\displaystyle{\begin{bmatrix}\sin(\beta L)+\sinh(\beta L) & \cos(\beta L)+\cosh(\beta L)\\\cos(\beta L)+\cosh(\beta L) & -\sin(\beta L)+\sinh(\beta L)\end{bmatrix}  \begin{bmatrix}a\\b\end{bmatrix}=0}
$$

For a non-trivial solution, the determinant of the above matrix must be 0, i.e.

$$
\displaystyle{\mathrm{det}\begin{bmatrix}\sin(\beta L)+\sinh(\beta L) & \cos(\beta L)+\cosh(\beta L)\\\cos(\beta L)+\cosh(\beta L) & -\sin(\beta L)+\sinh(\beta L)\end{bmatrix}=0}
$$

which gives the following characteristic equation

$$
[\sin(\beta L)+\sinh(\beta L)][-\sin(\beta L)+\sinh(\beta L)]-[\cos(\beta L)+\cosh(\beta L)]^2=0
$$

The above equation simplifies to

$$
\cos(\beta L)\cosh(\beta L)=-1
$$

There are infinite solutions to this characteristic equation, which are given by

$$
\beta L=1.875, 4.694, 7.855,...\space...
$$

$$
\beta_{1,2,3,...}=\frac{1.875}{L},\frac{4.694}{L}, \frac{7.855}{L},...\space...
$$

Thus, the mode shapes are

&#x20;                $$\displaystyle{v_i(x)=[\cosh(\beta _ix)-\cos(\beta_ix)]+\frac{\cos(\beta_iL)+\cosh(\beta_iL)}{\sin(\beta_iL)+\sinh(\beta_iL)}[\sin(\beta_ix)-\sinh(\beta_ix)] \tag{1.25}}$$            &#x20;

Since $$\omega = \sqrt{\frac{EI}{m}}\beta^2$$ , the modal frequencies $$\omega_i$$ are given by

&#x20;                                                                     $$\displaystyle{\omega_i=\sqrt{\frac{EI}{m}}\beta_i^2,\space\space i=1,2,3,...\space...} \tag{1.26}$$                                         &#x20;

or

$$
\omega_1=\sqrt{\frac{EI}{m}}\left(\frac{1.875}{L}\right)^2
$$

$$
\omega_2=\sqrt{\frac{EI}{m}}\left(\frac{4.694}{L}\right)^2
$$

$$
\omega_3=\sqrt{\frac{EI}{m}}\left(\frac{7.855}{L}\right)^2
$$

{% hint style="info" %}
$$m$$ is mass per unit length
{% endhint %}

The mode shapes of a uniform cantilever beam are shown in Fig. 7.

![Figure 7. First three mode shapes of a uniform cantilever beam.](<.gitbook/assets/image (84).png>)

In order to determine the modal frequencies and mode shapes, the corresponding frequencies can be excited by providing a sinusoidal input to the link via external means. When the frequency of the input coincides with either the fundamental frequency or higher frequency modes, the corresponding modes will be excited due to resonance and their mode shapes can be observed physically. Hence, the following experiment involves a frequency sweep across a range provided as input to the flexible link via the servo motor to identify the frequencies and observe the corresponding mode shapes.

## Part 2

### Experimental Procedure

{% file src=".gitbook/assets/FlexLink_ExciteMode.mdl" %}

1.  Download and open **FlexLink\_ExciteMode.mdl**


2.  To build the model, click the down arrow on **Monitor & Tune** under the Hardware tab and then click **Build** **for monitoring** ![](<.gitbook/assets/image (117).png>). This generates the controller code.


3.  Open the scope **alpha**.


4.  Turn on the power supply.


5.  Ensure that the manual switch is connected to the **Chirp signal** input. This signal will provide a sinusoidal signal of fixed amplitude, with frequency increasing at a linear rate with time.


6.  Open the **Chirp signal** command block and make sure that the Initial frequency is **0.1 Hz**, the target time is **0.25 s** and the Frequency at target time is **0.2 Hz**. This will allow the frequency sweep to take place at a reasonable rate and ensure that the relevant frequencies are covered within the span of time.


7.  Press the **Connect** <img src=".gitbook/assets/image (121).png" alt="" data-size="line"> button under Monitor & Tune **** and click on **Start** <img src=".gitbook/assets/image (119).png" alt="" data-size="line">. Run the servo motor with the chirp input voltage for 60 seconds.


8. Save the data using the format **Osc\_ChirpSignal** into your folder. **DO NOT DELETE THE SIMULINK MODEL**.

### Analysis

1.  Plot the measured angular deflection of the link vs. time.  Using the time domain plot, perform frequency analysis using Fast Fourier Transform (FFT) \[refer to the Appendix for the code] or a similar technique to identify the number of dominant frequencies and their magnitudes present in the signal.


2. Compare the first dominant frequency with the natural frequency of the flexible link calculated from the theoretical Analysis (1) and state your observations. Explain any similarities or differences observed.

### &#xD;

## Part 3

### Experimental Procedure

1.  Again, open the **FlexLink\_ExciteMode.mdl** simulink file used in part 2.


2.  To build the model, click the down arrow on **Monitor & Tune** under the Hardware tab and then click **Build** **for monitoring** ![](<.gitbook/assets/image (117).png>). This generates the controller code.


3.  Open the scope **alpha**.


4.  Turn on the power supply.


5.  Connect the manual switch to the **Sine wave** signal input. This signal will provide a sinusoidal signal of fixed amplitude and fixed frequency.


6.  Open the **Sine wave** signal command block and make sure that the Amplitude is **3** and Phase is **0 rad**. Enter the first modal frequency determined from the calculations in **rad/sec**.


7.  Press **Connect** <img src=".gitbook/assets/image (121).png" alt="" data-size="line"> button under Monitor & Tune **** and Press **Start** <img src=".gitbook/assets/image (119).png" alt="" data-size="line">. Run the servo motor with the sine wave for at least 20 seconds.


8.  Tune the frequency value by increasing or decreasing in steps of 1 rad/sec until the first mode shape is clearly visible.


9.  Observe the corresponding mode in the link and note down the number of nodes and their locations.


10. Save the data using the file name **Osc\_Sineinput** into your folder.


11. Repeat steps 6 to 10 for the second modal frequency identified.

### Analysis

1.  Plot the mode shapes observed using Eq. (1.25) in MATLAB (plot v(x) vs x/L) and record the node locations.


2.  Record the number of nodes and their locations for each mode. Determine the location as a ratio of the link length and compare with the values obtained from the mode shape plots.


3. Compare the calculated first and second modal frequencies (after tuning) with the corresponding first and second dominant frequencies obtained from Step 1 of Analysis (2) and explain your observation.

## Appendix

The following code performs FFT analysis on the flexible link angle response ``

```matlab
load data.mat %data that requires to use FFT
dT = time(2)-time(1);
Fs = 1/dT;
L = length(output_alpha)-1;
f = Fs*(0:(L/2))/L;
Ya = fft(output_alpha);
Pa2 = abs(Ya/L);
Pa1 = Pa2(1:L/2+1);
Pa1(2:end-1) = 2*Pa1(2:end-1);
% Frequency Response (FFT)
plot(f,Pa1)
xlim([0 150])
xlabel('Frequency (Hz)')
ylabel('|Amplitude {\alpha}|')
title('Single-Sided Amplitude Spectrum of \alpha')
% Angular Deflection vs. Time
figure
plot(time,output_alpha)
xlabel('Time (s)')
ylabel('Deflection Angle, \alpha (deg)')
title('Flexible Link Angle vs Time')
```

