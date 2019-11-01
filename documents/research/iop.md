# IOP Research

**Author: Oswin So**

tl;dr: Some German dudes did 99% of the hard work so it's worth doing because there's almost nothing to do.

## Table of Contents
1. [Overview of IOP](#overview-of-iop)
2. [Tooling](#tooling)
3. [fkie/iop_core](#fkieiop_core)

## Overview of IOP
**IOP**, which stands for **I**nter**o**perability **P**rofiles, is a challenge for the IGVC competition.
Essentially, the challenge is in conforming to the **J**oint **A**rchitecture for **U**nmanned **S**ystems (JAUS),
which is an architecture for Unmanned Ground Systems started by the DOD (!!!!) in 1998 (!!!!).

JAUS defines a bunch of things that need to happen which is very similar to ROS. Except, unlike ROS, the JAUS tools
all suck. 

The main concepts (IMO) in JAUS:
- JAUS is separated into **Subsystems**, **Nodes**, and **Components**
    - A **subsystem** is a vehicle, robot, or operator control unit (OCU).
    - A **node** is usually a single computing resource such as a PC or embedded processor 
    - A **component** is a software element that can communicate with other JAUS components.
      So, like ROS nodes.
        - **Server**: Provide one or more **services**
        - **Client**: Uses one or more **services**
- There's a DSL (**J**AUS **S**ervice **I**nterface **D**efinition **L**anguage) for defining the layout of each **message**.
  So like ROS messages.
  
So, as you can see, it's basically like ROS. Except, of course, that the tooling sucks.

## Tooling
AFAIK there's two main tools for **JAUS**:
1. [OpenJAUS](http://openjaus.com/)
    - They've got no prices, so spooky
    - They've got the word "Open" in their name, but they're not open source. Bamboozle = Banboozle
    - [Manipal](http://www.igvc.org/design/2019/6.pdf) claims that this is better than JAUS Toolset
2. [JAUS Toolset (JTS)](http://jaustoolset.org/)
    - It's actually open source.
    - Contracted by like the military
    - [Some German dudes](https://github.com/fkie/) made this [really nice framework that easily allows ROS nodes to
    communicate with IOP services](https://github.com/fkie/iop_core)
    - It generates boilerplate code for a component / service given the JSIDL (C++, C#, Java)

I don't know about OpenJAUS. Given that their website looks more modern, and they're paid, it seems like their product is
pretty good. JAUS Toolset, on the other hand, seems like some horrible piece of software written in Java. Look at their
GUI, who thinks drawing state machines to describe your entire system is fun and nice?

![](http://jaustoolset.org/wp-content/uploads/2012/12/standards-990x397.png)


## `fkie/iop_core`
[fkie/iop_core](https://github.com/fkie/iop_core) is a framework that wraps around JTS.

> This repository lets your ROS software communicate with IOP services 

I've tried it out, and it looks like quite a lot of work was put in to this project. You can read
their README (which is pretty good ngl) for an overview of how the framework works, but essentially:

- The default JSIDL is modified from the default ones (still not sure what the difference is)
- JTS is used to generate boilerplate C++ from the JSIDLs
- Each _JAUS service_ is written as plugin for [pluginlib](http://wiki.ros.org/pluginlib),
  extending `iop::PluginInterface`
- These _pluginlib JAUS services_ are then specified as a YAML parameter for the `iop_component` ROS node, ie
  ```yaml
  [
    fkie_iop_discovery: "DiscoveryClient",
    # added client for primitive driver and also his subservices
    fkie_iop_client_primitive_driver: "PrimitiveDriverClient",
  ]
  ```
  with parameters for each service also specified in YAML as a parameter, ie.
  ```yaml
  EventsClient:
      use_queries: false
  DiscoveryClient:
      register_own_services: false
      enable_ros_interface: true
  # configuration for primitive client
  # see https://github.com/fkie/iop_jaus_mobility_clients#fkie_iop_client_primitive_driver-primitivedriverclient
  PrimitiveDriverClient:
      # do not use the stamped twist messages
      use_stamped: false
      # new parameter added to handle velocities greater than 1.0
      # you should update the https://github.com/fkie/iop_jaus_mobility_clients repository
      max_linear: 2
      max_angular: 2
      # added remap to catch commands from the right topic or use 'remap' of launch file
      topic_sub_joy_cmd_vel: cmd_vel
  ```
- There's a `nm.cfg` file that defines network configs, ie.
  ```xml
  <UDP_Configuration
      
      UDP_Port        = "3794"
      MulticastTTL    = "16"
      MulticastAddr   = "239.255.0.1"
      MaxBufferSize   = "70000"
  />
  ```
- You run a `JTSNodeManager` that handles all the communications between the components
  and the outside world (hmm `roscore`?)
    - fkie has their [own version](https://github.com/fkie/iop_node_manager)
- ????
- Stuff works

## IOP challenge requirements for IGVC
Coming back to the IOP challenge for IGVC:

> Each entry will interface with the **Judges Testing Client (JTC)** as specified in the sections that follow.
> The **JTC** will be running testing software called the **Conformance Verification Tool (CVT)**, which evaluates
> JAUS services interfaces, and will also be running Common Operating Picture (COP) software that will display
> information coming from the entrant’s platform while it is executing the tasks defined by the Interoperability
> Profiles Challenge. 

Also **CVT**
(Taken from [OpenJaus](https://support.openjaus.com/support/solutions/articles/35000112742-accessing-the-conformance-verification-tool-cvt-):
> OpenJAUS is aware of 3 ways to obtain a copy of the **CVT**:
> - If you are a member of the **National Advanced Mobility Consortium (NAMC)**, you can request a copy through NAMC
> - If you are **under contract with the Government** and that contract contains the proper clauses and scope for access to the CVT, you can request a copy through the relevant contracting office
> - If you are a **sub-contractor to a company that meets the requirements for #2 above**, the primary contractor can get written permission to provide the CVT to you 

So, either you're military, or you're government. Fun.

> A team will provide **two JAUS Components** - a **Navigation and Reporting JAUS Component** that contains all
> the services defined by the Navigation and Reporting capability, and **platform management JAUS Component**
> specified by the Platform Management attribute basic value.

So we need two JAUS components. The services that are required of each component are below:
- Platform Management
    - Transport
    - Events
    - Access Control
    - Liveness
    - Discovery
- Navigation and Reporting
    - Transport
    - Events
    - Access Control
    - Management
    - Liveness
    - Waypoint Driver
    - Waypoint List Driver
    - Velocity State Sensor
    - Local Pose Sensor
    - Primitive Driver

All of which are standard services defined in JAUS.

Thankfully, `fkie/iop_core` has as few implementations of these services that are already done:
- `fkie_iop_transport`
- `fkie_iop_events`
- `fkie_iop_accesscontrol`
- `fkie_iop_management`
- `fkie_iop_liveness`
- `fkie_iop_discovery`
- `fkie_iop_local_waypoint_driver`
- `fkie_iop_local_waypoint_list_driver`
- `fkie_iop_velocity_state_sensor`
- `fkie_iop_local_pose_sensor`
- `fkie_iop_primitive_driver`

So, long list of services that we need to implement:
- ¿؟⁇?

Yeah. The list of thing(s) that we would need to do would be:
- Implement our own version of the secret tool that can only be given to military or government (CVT), because
  Sometimes it happens that way, hyung 
  - Thankfully some Indian dude part of Manipal asked about this last year on an issue for the `fkie/iop-core` repo,
  and the German dudes actually gave [a really good response](https://github.com/fkie/iop_core/issues/2).
  - Basically, we can use [wireshark](https://www.wireshark.org/) and some
  [LUA plugin that some other repo trying to do ROS+JAUS integration did](https://github.com/udmamrl/ROSJAUS/blob/master/Wireshark-dissector/Wireshark_JAUS_dissector.lua)
  - and then it's like unit test writing time, to check that all the specs mentioned in the IGVC competition manual are
  fulfilled

## My thoughts on IOP
Finally, thoughts on whether we should do IOP
- Initially I thought we would need to craft our own UDP packets and like build this up from OSI Layer 5 all the way
  up to Layer 7, and then somehow integrate that with ROS
- And then I realized that `fkie/iop_core` + JTS handles all of this for us
- So... the amount of work we need to do is like just making sure that this works

So, IMO IOP is worth doing.
