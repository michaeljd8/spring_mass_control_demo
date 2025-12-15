# Motion Control Simulation Demo in C++

## Overview

This repo is for a demo of motion controls of a spring-mass system on a cart. There will be a simulation plant model, and the controls will be in C++.

The system will comprise of:

- Velocity-controlled drive 
- Spring and mass in series
- Mass is modeled as a cart

There will be no damper in the system. The friction in the axles will be minimal in the plant model, but the Coulomb friction will be exaggerated to make the system more difficult to control. Increasing the Coulomb friction will increase the stiction at the start of motion, which will cause a larger response in the spring.

C++ object-oriented programming principles will be used to create the plant model and controls. The plant model and controls will be modular and reusable to scale up to multiple-axis systems in the future. The modularity will also allow for easy testing and validation of the plant model and controls while keeping the controls separate from the plant model so no changes are needed to move the controls to Hardware in the Loop (HIL) testing or real hardware.

### Fig. 1 - Free Body Diagram

![alt text](<images/Screenshot from 2025-12-15 12-02-50.png>)

- m1 = mass
- k = spring constant
- v1 = velocity at drive (control velocity)
- v2 = velocity at cart (dependent velocity)

## Controls

The system must control the velocity at the cart. The controls must drive the cart at a user-defined target velocity at a target distance while minimizing the time to reach the two parameters. Therefore, the challenge of the system is to travel as fast as possible to the target distance while being able to slow down to the target velocity while staying in control of the motion at the end of the spring.

### Requirements

- There will be a user-defined target velocity.
- There will be a user-defined target distance.
- The mass of the cart will be configurable.
- The spring constant and cart behavior will remain constant.
- The system must be able to reach a steady-state velocity without the velocity exceeding +/- 2% of the target velocity after the target distance. 
    - This requirement must be met regardless of the parameters or mass changing. 

## Plant Model

There will be a mathematical model of the plant in C++ that will simulate the behavior of the spring-mass system. The plant model will be discrete-time and will run at a fixed time step. The plant model will take in the control velocity as an input and output the cart velocity and position.


### Testing Harness

A testing harness will be created to test the plant model and controls. The testing harness will allow for the following:
- Configuration of target velocity and distance
- Configuration of mass
- Running the simulation for a fixed time
- Output of results for analysis
- Basic visualization of the drive velocity and cart velocity over time
