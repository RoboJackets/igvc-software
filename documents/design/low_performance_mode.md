# “Low performance mode” for simulation

*Issue #572*

**Author:**
- Jessica Zheng

## The Problem

The problem I am trying to solve is the low realtime factor of the simulation by creating a “low performance mode,” which will omit a few factors in order to increase performance. Currently, my laptop runs qualification.launch at 0.02 fps. It’s important to solve this problem because having a higher realtime factor allows us to run gazebo more efficiently, even on an average laptop. While this would neglect a few factors and risk a few inaccuracies, it would overall save time and energy when testing. The second part of this problem is benchmarking the current simulation in order to find out what exactly is causing this lack in performance and if this real-time factor could be increased without yielding some accuracy. Ideally the low-performance simulation would run above a 0.5 realtime factor for a laptop with average specs.

## Proposed Solution

- Begin by duplicating the current world file for the low-performance world
    - This way the current simulation stays the same (slow but accurate) while the world created from duplicating can be altered
- Create a new launch file too 
    - So that we don’t have to edit the world parameter everytime we roslaunch to access the low-performance mode
- Try reducing physics simulation rate in this new world file and note the difference in real time factor
    - Also see how much accuracy is affected by this change
- Try reducing rate and detail of sensor plugins and note the difference in real time factor
- Try disabling collisions on barrels
- Benchmark the current simulation using PERF (Performance Events for Linux)

## Questions & Research
So far, the only things I will need to look up are what variables to change in order to reduce the real time factor. I have a general idea that I will need to alter these values, but I need to research what they mean or if there is a limit to how much I should alter it. 
For reducing the physics simualation rate, it looks like I will be changing either the real_time_update_rate or max_step_size variable found in the .world file.

I believe that reducing the rate/detail of sensor plugins would mean changing the values in igvc-software/igvc-description/urdf/jessi.urdf. Does this mean that I would also need to duplicate this file also? Do I need to duplicate every file I need for this low-performance mode?

For benchmarking the current simulation, I will probably be using PERF: 
sandsoftwaresound.net/perf/perf-tutorial-hot-spots/ 

## Overall Scope

### Affected Packages

- Creating and adding another world file to igvc-software/igvc_description/urdf/worlds/
- Creating and adding another launch file to igvc-software/igvc_gazebo/launch/
- Possibly duplicate jessi.urdf with different variable values

### Schedule

Subtask 1 (11/24): Create the .world files and .launch files that will be used for low performance mode and ensure that they run correctly.

Subtask 2 (1 meeting): Alter physics simulation rate in the new .world file and observe change to real time factor.

Subtask 3 (1 meeting): If needed, reduce the rate and detail of sensor plugins, possibly duplicate jessi.urdf file and edit new .launch file. Observe change to real time factor.

Subtask 4 (1 meeting): If needed, disable collisions on barrels and observe change to real time factor.

Subtask 5 (1 week): Benchmark the current simulation using PERF

Code Review: 
