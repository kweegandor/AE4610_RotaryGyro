# Lab 4: Rotary Gyro

## =-ntroduction

The objective of this experiment is to design a controller that maintains the direction of the gyroscope module while the top base plate is rotated relative to the bottom base plate. While the disk spins, the SRV02 is used to apply the correct amount of counter torque and maintain the gyroscope heading in the event of disturbances (i.e., rotation of the bottom support plate).

Gyroscopes are used in many different devices, e.g., airplanes, large marine ships, submarines, and satellites.

## Part 1: Modeling

In this section, we will develop a model of the gyroscope device from first principles in order to obtain a transfer function of the plant, which we will later use in a control sheme. To do so, however, we need some preliminaries on angular momentum and the gyroscopic effect.

### Angular Momentum

Let's consider an extended rigid body $$B$$ of mass $$m$$, moving with any general motion in space. Assume that the axes x-y-z are attached to the body with origin at its center of mass $$G$$. Now, let $$\mathrm{d}m$$ be a mass element of the body situated by position vector $$\bm{\rho}$$ with respect to the center of mass $$G$$. Given the body's linear velocity $$\bm{v}$$  and its angular velocity $$\bm{\omega}$$ the total angular momentum of the body with respect to the center of mass $$G$$ is given as

$$
\bm{L} = \bm{v}\times \int_B \bm{\rho}\mathrm{d}m + \int_{B} \bm{\rho} \times \left(\bm{\omega}\times\bm{\rho}\right)\mathrm{d}m.
$$

The first term is $$\bm{0}$$ since the integral $$\int_B \bm{\rho}\mathrm{d}m = \bm{0}$$ by definition of the center of mass. By using the tripple product rule for the cross product in the second term, it can be shown that&#x20;

$$
\bm{L} = \int_{B} \bm{\rho} \times \left(\bm{\omega}\times\bm{\rho}\right)\mathrm{d}m =  \int_{B} (\bm{\rho}^\top \bm{\rho} \mathrm{I}_3 -\bm{\rho}\bm{\rho}^\top) \mathrm{d}m \, \bm{\omega} = J\bm{\omega}
$$

where $$J$$ is the $$3\times3$$ inertia tensor of the body about the center of mass $$G$$.

### Change of Angular momentum

Now, since the body's reference frame is rotating, the derivative of the total angular momentum about the center of mass $$G$$ can be written as

$$
\frac{\mathrm{d}}{\mathrm{d}t} \left(\bm{L}\right) = \dot{\bm{L}} + \bm{\omega} \times \bm{L}
$$

where $$\dot{\bm{H}}$$is the time derivative of the angular momentum as seen in the rotating x-y-z frame. We can view the term $$\dot{\bm{H}}$$ as the part due to the change in magnitude of $$\bm{H}$$ and the second term $$\bm{\omega}\times\bm{H}$$ as the part due to the change in direction of $$\bm{H}$$. Assuming now that the inertia of the rotating body is fixed in the body axes, we have that $$\dot{\bm{H}} = \dot{J}\bm{\omega} + J\dot{\omega} = J\dot{\omega}$$. By virtue of the Newton Euler equation, we have

$$
\bm{\tau} = \frac{\mathrm{d}}{\mathrm{d}t}\left(\bm{L}\right) = J\dot{\bm{\omega}} + \bm{\omega}\times J\bm{\omega}
$$

and notice that either a change in magnitude of the angular moment or a change in direction of the angular momentum generates a torque. Inversely, a torque can generate a change in either magnitude or direction of the angular mometum, depending about which axis it is applied.

## Part 1: Modeling

### Servo Model

The Servo Base Unit (SRV02) open-loop transfer function is given by

$$
f(x) = x * e^{2 pi i \xi x}
$$
