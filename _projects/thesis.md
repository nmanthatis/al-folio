---
layout: page
title: Master Thesis
description: Nonlinear Control of a Tricopter.
img: assets/img/Skoupa_render.png
importance: 1 
---
<h4> Abstract  </h4>
The current Masters (Diploma) thesis was co-authored with my classmate [Faidon Tsamis](mailto:faitsamis@gmail.com) as a part of the [MPU](https://lfmt.gr/mpu-rx4/) project. Its subject is the design of a control system for the trajectory tracking of the altitude and the orientation of a tilt-rotor tricopter at the state of hovering. An initial control systen was developed, which is based on the `feedback linearization` technique, to achieve the desired error dynamics of the output states. For the disturbance cancellation, the initial control law is augmented with the construction of a `γ-suboptimal controller` based on the Bouned Real Lemma.


<h4> Modelling  </h4>
The aerial vehicle is equipped with 5 actuators, in the form of 3 propellers and
2 servo motors. The propellers produce thrust and the servomotors tilt the 
vector of the thrust produced by the front propellers. As seen in Figure 1 
all the forces can be represented on an equivalent triangle, due to the 
vehicle's geometry. 

<div class="row">
	<div class='col-sm mt-3 mtmd-0'>
        {% include figure.html path="assets/img/rx4_free_body_diagram.jpg" title="Vehicle Free Body Diagram" class="img-fluid rounded" description='asd' %}
	</div>
</div>
<div class='caption'>
	Fig.1: Free Body Diagram - Forces acted on the equivalent triangle of the aerial vehicle.
</div>

The tricopter was modelled as a solid non-deformable body. The kinematic and kinetic equations were extracted using euler angles as orientation parametrization. The equations of motion had the form:
\begin{gather}
m\mathbf{\dot v} =R \sum \mathbf{T}_i + m\mathbf{g}\nonumber
\end{gather}
\begin{gather}
I_G \dot{\omega} = \mathbf{M}_G - {\omega}\times{I_G{\omega}}.\nonumber
\end{gather}

Where $$v$$ and $$\omega$$ are translational and rotational velocities and $$T$$ and $$M_G$$ are forces and moments respectively. As a next step, an $$1-1$$ mapping was created to uniquely match the forces and moments to the control variables $$u$$. The equation of the system can be written in affine (with respect to the vector of forces and moments $$ d = (T, M)^T$$ form as follows:

\begin{equation}
	\mathbf{\dot x}= f_0(\mathbf x) + G(\mathbf e)d, 
	\label{affine}
\end{equation} 

where $$x$$ the state variables.

Each propeller produces thrust ($$T$$) and moment ($$N$$). The two variables are mostly dependent on the rotational speed of the propeller and the air velocity. Their equations can be written as:
\begin{equation}
	T(\omega, V) = \alpha\omega^2 + \beta V^2 \nonumber
\end{equation}
\begin{equation}
	N(\omega, V) = \gamma\omega^2 + \delta V^2. \nonumber
\end{equation}

The parameters $$\alpha, \beta, \gamma, \delta$$ were computed using the least squares' method by approximating the data given by the manufacturer. The results of the approximation for the thrust of a single propeller are given in Figure 2, where data and the approximation surface are plotted.
<div class="row justify-content-sm-center">
	<div class="col-sm-8 mt-3 mt-md-0">
		{% include figure.html path="assets/img/ThrustApproximation.png" title="Thrust Approximation" class="img-fluid rounded " %}
	</div>
</div>
<div class='caption'>
	Fig.2:  Thrust Approximation.
</div>
While our analysis is about hovering, the contribution of the air velocity to the propellers' load is insignificant. So, we keep only the curve that results from the intersection of the plane $$V=0$$ and the approximation surface. To validate our model we measured the produced thrust of propellers. As seen in Figure 3, we used a 10kg loadcell, an Arduino UNO to log the measurements and the necessary equipement that the motor needs to work.
<div class="row">
<div class="col-sm mt-3 mt-md-0">
{% include figure.html path="assets/img/schematic.png" title="example image" class="img-fluid rounded z-depth-1" %}
</div>
</div>
<div class='caption'>
Fig.3:  Thrust Measurung Device diagram.
</div>

<h4> Experimental Configuration  </h4>
The resources needed to manufacture an original MPU aerial vehicle led to the
design and construction of an inexpensive and easily repairable experimental 
device. The design was done with Autodesk Inventor and aluminium and 3D printed
PLA were used as building materials. A tricopter in "T" shape was designed that
adequately reproduces the function of the original vehicle. In Figure 4 an 
intersection of the tilting mechanism is presented.



<div class="row justify-content-sm-center">
<div class="col-sm-7 mt-3 mt-md-0">
{% include figure.html path="assets/img/skoupa_mech.png" title="Tilting Mechanism" class="img-fluid rounded " %}
</div>
</div>
<div class='caption'>
Fig.4: Tilting Mechanism intersection and exploded view.
</div>

The full assembly can be seen in Figure 5, where the rendered and the 
constructed test beds are presented. The experimental configuration constists
of 3 pairs of motor-propeller-ESC, 2 servomotors, a LiPo battery and the
[Pixhawk flight controller](https://docs.px4.io/v1.12/en/flight_controller/pixhawk-2.html).

<div class="row justify-content-sm-center">
<div class="col-sm-6 mt-3 mt-md-0">
{%include figure.html path="assets/img/Skoupa_render.png" title="Tricopter Render" class="img-fluid rounded " %}
</div>
<div class="col-sm-6 mt-3 mt-md-0">
{%include figure.html path="assets/img/skoupa_real.png" title="Tricopter Manufactured" class="img-fluid rounded z-depth-1" %}
</div>
</div>
<div class='caption'>
Fig.5:  Designed and Manufactured Experimental Configuration.
</div>

<h4> Non Linear Controller  </h4>
The developed control law aims at the autonomous hovering and the trajectory
tracking of the tricopter. Firstly, a non-linear control law is composed that 
utilizes feedback linearization to impose the desired first order error 
dynamics on the state variables. Moreover, taking into consideration the 
constant and variable disturbancies of the system, γ-suboptimal control is
being used. The mathematics around the proecess of the control development are
going to be presented simplistically in order to fit with the context of the 
website.

Consider the affine system (\ref{affine}) and we are selecting a set of forces
(input variable $$d$$) that casts away the non linear term $$f_0$$ so we can
impose our desired dynamics. Let the desired vector $$x_d$$, if we select
$$d$$ to be $$d = G(e)^{-1}(-f_0(x) + K(x - x_d) + \dot{x}_d)$$ then our system 
will become
\begin{equation}
	\dot{x} = f_o(x) + G(e)G(e)^{-1}(-f_0(x) + K(x - x_d) + \dot{x}_d) 
	\Rightarrow \nonumber
\end{equation}
\begin{equation}
	\dot{x} = K(x - x_d) + \dot{x}_d \nonumber	
\end{equation}
letting $$\widetilde{x} = x - x_d$$
\begin{equation}
	\dot{\widetilde{x}} = K\widetilde{x}.
	\nonumber
\end{equation}

This way we constructed the first order error dynamics to our system. Next we
tune our controller by choosing the matrix $$K$$ that satisfies the needs of 
our application. In the next figure we observe the step response for the 
output vector.
<div class="row">
<div class="col-sm mt-3 mt-md-0">
{% include figure.html path="assets/img/fig_testeig.png
" title="Step Response of output variables" class="img-fluid rounded " %}
</div>
</div>
<div class='caption'>
	Fig.6: Step Response of output variables.
</div>

Figure 6 depicts the time constants as black dots that our tuning suceeds.
Then we are testing our control law to a possible scenario which is to reach
a altitidue of $$30m$$ and have a defined heading of $$ 50^{\circ} $$ to the
north pole while having non-zero initial orientation. This is a possible 
mission for the tricopter as it is getting the pose to continue flying as a
regural plane.


<div class="row">
<div class="col-sm mt-3 mt-md-0">
{% include figure.html path="assets/img/fig_sigmoid_1.png
" title="Scenario 1" class="img-fluid rounded " %}
</div>
<<div class="row">
<div class="col-sm mt-3 mt-md-0">
{% include figure.html path="assets/img/fig_sigmoid_2.png
" title="Scenario 1" class="img-fluid rounded " %}
</div>
</div>
</div>
<div class='caption'>
	Fig.6: Flight Scenario.
</div>

At Figure 6 we see that our control law achieves the desired outcome (top)
by not exceeding the actuator limits ($$\omega_i, \theta_i$$). 

<!---
As a first step the affine system (\ref{affine}) gets divided into two 
subsystems, with the first one containing kinetic equation (
control variables) and the second one the kinematic equations.

\begin{equation}
	%\dot x = f_0(x, y) + \sum_i^md_if_i(x, y) \nonumber
	\dot x = f_0(x, y) + F(x, y)d \nonumber
\end{equation}
\begin{equation}
	\dot y = A(y)x \nonumber
\end{equation}

where the variables 

$$x = (v_z,  \omega_x, \omega_y, \omega_z)^T \in \mathbb{R}^n$$,   
$$y = (z_E, e^T)^T\in V$$,   
$$d = (T_x, T_z, M_x, M_y, M_z)^T\in \mathbb{R}^m$$.

The vector $$x$$ represents the velocities (translational and rotational), the vector $$y$$ the displacements and $$d$$ the forces expressed on the local reference system, respectively.

Note: An [appendix](#app) is located at the end of the page in order to declutter the
text. The necessary definitions that the text is missing can be found there.

If for the input variable $$d$$, for random $$(t, x, y) \in R \times R^n \times V$$ we choose 
$$d(t, x, y) = F^{\dagger}(x, y)(-f_0(x, y) + v(t, x, y))$$ then our system becomes

\begin{equation}
	%\dot x = f_0(x, y) + \sum_i^md_if_i(x, y) \nonumber
	\dot x = v(t, x, y)\nonumber
\end{equation}
\begin{equation}
	\dot y = A(y)x \nonumber
\end{equation}

where $$F^{\dagger}$$ is a right pseudoinverse.

Note that F can produce a bigger space than the desired, so we choose a 
solution.This way we cast the non-linear term $$f_0$$ away. Now let the
mapping $$y_r:\mathbb{R} \to V$$ be the reference signal for our output
variable y. Moreover, $$ \alpha : \mathbb{R}\times V \to \mathbb{R^n}$$
with $$a(t, y) = K1(y - y_r(t)) + \dot{y}_r(t)$$ where $$ K_1 \in 
\mathcal{L}(\mathbb{R}^n)$$. If $$\dot{y}$$ becomes equal to $$\alpha$$ 
then we have

\begin{equation}
    \dot{y} = a(t,y) \Rightarrow \dot{y} = K_1\left(y-y_r(t)\right) + 
    \dot{y}_r(t) \Rightarrow \dot{y} - \dot{y}_r(t) = K_1(y - y_r(t)).
    \nonumber
#\end{equation}
#
#By setting a deviation variable $$\widetilde{y}: \mathbb{R}\times \mathbb{R}^n
#\to \mathbb{R}^n$$ with $$\widetilde{y} = y - y_r(t)$$ we get 
#
#\begin{equation}
#	\dot{\widetilde{y}} = K_1\widetilde{y}.
#	\nonumber
#\end{equation}
#
#This way we create the first order error dynamics that can be tuned by 
#choosing the matrix $$K_1$$. But the input variable $$d$$ acts on the 
#output variable $$y$$ indirectly through variable $$x$$.
#
#
#We define $$f:GL\left(\mathbb{R}^n\right)\to GL\left(\mathbb{R}^n
#\right)$$ with $$f\left(A\right)= A^{-1}$$ end $$\phi:\mathbb{R}\rightarrow 
#\mathbb{R}^n$$ with $$\phi(t)= \left(\left(f \circ A\right)\left(y\left(t\right)
#\right)\right)\alpha\left(t\right)$$. Applies that 
#
#\begin{equation}
#	\dot{\phi} = -\left(A(y)^{-1} \circ \left(A'(y) \dot{y}\right)\right)\phi
#	 + A(y)^{-1}\dot{\alpha}.
#	\nonumber
#\end{equation}
#
#<h5> Mathematical Appendix </h5> <a name="app"></a>
#Let the smooth mappings
#
#$$f_0:\mathbb{R}^n\times V \to \mathbb{R}^n, f_0 = \pmatrix{g \\ -I_G^{-1}
#(\omega \times I_G\omega)}$$, $$
#%f_i:\mathbb{R}^n\times V \to \mathbb{R}^n,
#%(f_1, ..., f_m) = \pmatrix{m^{-1}(R_{3, 1}& R_{3, 3}), 0_{1\times3}\\ 0_{3\times2}& I_G^{-1}}$$
#$$ F:\mathbb{R}^n\times V \to \mathcal{L(\mathbb{R^{n \times m}})},  = \pmatrix{m^{-1}(R_{3, 1}& R_{3, 3}), 0_{1\times3}\\ 0_{3\times2}& I_G^{-1}}$$,  
#$$ A: V \to \mathcal{L(\mathbb{R^n})}, A(y) = \pmatrix{1 & 0_{1\times3}\\ 0_{3\times1} & R_eul}$$
#
#V 
#
#$$F^\dagger$$
--->

You can download the thesis (in Greek) [here](/assets/pdf/manthatis_thesis.pdf).


