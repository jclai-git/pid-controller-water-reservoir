# pid-controller-water-reservoir
This repository is for my Automatic Control Systems project on a PID control for a water reservoir system.

The water reservoir system is a unity-feedback system with a controller, C(s), water pump motor, P(s), and STH (motor speed to water height) block, G(s), all in series in the forward path.

`water_reservoir.m` contains my self-defined functions used for the system modeled in this project. They can be accessed using object-oriented syntax where the class is `water_reservoir`.

`pid_controller_water_reservoir.m` is the main code which simulates and analyzes the closed-loop system under varying controllers.
First, I test the system with no controller, then with 2 P controllers, 1 PI controller, 1 PD controller, and 2 PID controllers, where each is implemented separately.

The closed-loop stabilty is inferred by analyzing the open-loop behavior via two method: (1) bode plot, and (2) root locus plot.
