# Constrained Manual Control

Constrained manual control is an approach to manual driving that
imposes limits on the commands given to the drivetrain, so that
the middle control layer is only exposed to **feasible trajectories**.

Examples of feasibility constraints include:

* limiting jerk, acceleration, and velocity
* respecting wheel steering rate
* limiting tire scrub

More details are available in the 
[project doc.](https://docs.google.com/document/d/1OiS-p7kSfjr5zGTwTKOOMetWvmA6SG_rx9CdtBwSZKA/edit)

The contents of this project should really just be unit tests.