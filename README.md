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

The system must control the velocity at the cart. The controls must drive the cart at a user-defined target velocity at a target distance while minimizing

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

Testing harnesses will be created to test the plant model and controls. The testing harness will allow for the following:
- Configuration of target velocity and distance
- Configuration of mass
- Running the simulation for a fixed time
- Output of results for analysis
- Basic visualization of the drive velocity and cart velocity over time

## Model in Loop (MIL) Level 1

The MIL Level 1 test will integrate the systems ability to calculate the velocity path and use a PID controller to set the drive velocity based on the user defined inputs. All other aspects of the system will be part of the plant model for Level 1.

### Classes:

- spring mass control demo
- plant model
- testing harness

### **Methods:**
- Calculate velocity path
- PID controller
- set target velocity
- set target distance
- set travel velocity
- set approach distance
- set acceleration
- set deceleration

### **Fields:**
- **target velocity:** The desired velocity at the cart when the target distance is reached.
- **target distance:** The position the cart much reach the target velocity within the defined tolerance.
- **travel velocity:** User defined velocity to travel to the target distance.
- **approach distance:** The distance at which the target velocity will be reached.
- **acceleration:** User defined acceleration to reach the travel velocity.
- **deceleration:** User defined deceleration to reach the target velocity at the target distance.
- **max velocity:** Max velocity the system is capable of reaching.
- **min velocity:** Min velocity the system is capable of running reliably. 

## Flow Diagram

```mermaid
flowchart TD
    subgraph testing harness
        A[START] --> B[Initialize Class Objects]
        B --> C[Run Simulation] 
    end
    subgraph spring mass control demo
        B --> D[Calculate Velocity Path]
        C --> E[PID Controller]
    end
    subgraph plant model
        E -->|drive velocity| F[Cart Velocity Simulation]
                F -->|cart velocity sim| E
    end
