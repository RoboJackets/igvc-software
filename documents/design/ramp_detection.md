# Ramp Detection

 *Issue #563*

 **Author:**
 Monica Gupta

 ## The Problem

 What is the problem you are trying to solve? Why is it important to solve this problem?
 
 -Ramp detection, so that the robot does not classify the ramp as an obstacle and moves over it.
 
 What is the end goal of this project? This section should similar to the body of the GitHub issue but include more details about it. This will be a qualitative description of the
 project. Use as many details you can to describe the requirements of the project.
 
 -The end goal is to make sure the ramp isn't detected as an obstacle and to make sure that the white lines on the ramp are detected correctly so that the robot doesn't move off the side of the ramp.

 ## Proposed Solution

 - How are you going to solve this problem?
 -Try different approaches to detect the ramp using the non-ground LIDAR output. One idea is to use surface normals and detect an L shape.
 - What are the steps you need to take to complete the project?
 -Write a node to detect the ramp and visualise how the detection works in Gazebo.
 - Break down the problem above into smaller, individual components with specific metrics of success
 -Single component problem: create node to detect ramp.
 - You can think of this section as a quantitative description of the project
 - Each bullet should represent a single action
     - Use sub-bullets to provide more details and justifications for each step
 - *(Replace these bullet points with your own)*

 _It's OK if your approach changes later in the project as you learn more. Treat this like
 a road map that you can come back to to figure out what to work on next. If you aren't sure about the
 technical details, make sure your ideas are clear. It's also OK to come up with a couple solutions
 and decide which to pursue later. There should be some justification with your solution (i.e. how 
 does each step address part of the problem above)._

 ## Questions & Research

 Are there things you are unsure about or don't know? What do you need to research to be able to
 complete this project? If you need information from the mechanical or electrical subteams,
 be sure to describe that here. If your solution requires more research to implement, descibe
 what kind of topics you need look at. Link any research papers/articles that you find here.

-Learn how to work with LIDAR output. Learn general object detection concepts related to LIDAR perception.

 ## Overall Scope

 ### Affected Packages

 - What parts of the software will you have to change (if any)?
 -IGVC Perception
 - Which packages are relevant to the success of the project?
 - Your first step of the project should be reviewing the relevant packages and code

 ### Schedule

 Subtask 1 (present - Dec 14): Learn how to use rviz, gazebo, use the output of LIDAR.

 Subtask 2 (Dec 14 - Jan 3): Make sure ramp detection works for at least a few cases.

 Subtask 3 (Jan 3 - Jan 15): Make ramp detection robust.

 Code Review (Jan 15): 