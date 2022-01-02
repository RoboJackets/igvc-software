# Simulated Swervi swerve modules

*Issue #862*

**Author:**
- Vivek Mhatre

## The Problem

Although we have a urdf model of Swervi, we have no way of controlling Swervi in simulation. It is essential to have a way to control the swerve modules on Swervi in gazebo so we can actually test our new code for Swervi. The end goal of this project is to have a node that is able to control all four swerve modules in simulation.

## Proposed Solution

1. Setup a new joint controller configuration for the swerve modules on Swervi.
2. Create a new message type to control the swerve modules.
 - The existing `velocity_pair` message only works for differential drive. Need to create a new message type for swerve modules.
3. Write a new gazebo control node for Swervi that uses the new message type to control Swervi in simulation.

## Questions & Research

- How will the new joint controller work with two degrees of movement? (spinning and moving forward)
- How should the new message type be structured?
- How should the firmware be rewritten to interface with the new swerve modules?

## Overall Scope

### Affected Packages

- The `igvc_gazebo`, `igvc_msgs`, and the `igvc_description` packages will be affected. 