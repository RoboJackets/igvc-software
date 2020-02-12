# IOP Research

tl;dr: There's a repo `fkie/iop_core` that implements all the JAUS services we need to implement
for the IOP challenge, so all we need to do (software-wise) is create a "Conformance Verification
Tool" (CVT) to make sure that `fkie/iop_core` implements all the specs outlined in the rules.
Because most of the work has been done for us, I think that it is worthwhile for us to do IOP this
year.

**Author: Oswin So**

## Table of Contents
1. [Overview of IOP](#overview-of-iop)
2. [Tooling](#tooling)
3. [fkie/iop_core](#fkieiop_core)
4. [Tasks for other subteams](#tasks-for-other-subteams)
5. [My thoughts on IOP](#my-thoughts-on-iop)

## Overview of IOP
**IOP**, which stands for **I**nter**o**perability **P**rofiles, is a challenge for the IGVC competition.
Essentially, the challenge is in conforming to the **J**oint **A**rchitecture for **U**nmanned **S**ystems (JAUS),
which is an architecture for Unmanned Ground Systems started by the DOD in 1998.

JAUS defines a bunch of things that need to happen which is very similar to ROS.

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

## Tooling
AFAIK there are two main tools for **JAUS**:
1. [OpenJAUS](http://openjaus.com/)
    - They've got no prices
    - [Manipal](http://www.igvc.org/design/2019/6.pdf) claims that this is better than JAUS Toolset
2. [JAUS Toolset (JTS)](http://jaustoolset.org/)
    - It's actually open source.
    - [Some German dudes](https://github.com/fkie/) made this [really nice framework that easily allows ROS nodes to
    communicate with IOP services](https://github.com/fkie/iop_core)
    - It generates boilerplate code for a component / service given the JSIDL (C++, C#, Java)

I don't know about OpenJAUS. Given that their website looks more modern, and they're paid, it seems like their product is
pretty good. JAUS Toolset, on the other hand, seems less modern and more clunky, and requires users to define services
and components by drawing a state machine.

![](http://jaustoolset.org/wp-content/uploads/2012/12/standards-990x397.png)


## `fkie/iop_core`
[fkie/iop_core](https://github.com/fkie/iop_core) is a framework that wraps around JTS.

> This repository lets your ROS software communicate with IOP services 

I've tried it out, and it looks like quite a lot of work was put in to this project. You can read
their README (which is pretty good) for an overview of how the framework works, but essentially:

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

So, since we're not military or government, we probably won't have access to a CVT.

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

There are 2 main parts to the scoring:
1. IOP Interfaces Task
    - Basically we get points for implementing the required services, and get penalized if there's some error
2. Performance Tasks
    - The judges use the interfaces to, at the end, drive around a few waypoints, after making sure that the system is
    safe.

The time for the performance task waypoint run is used to break ties.

Now, back to the services.

Thankfully, `fkie/iop_core` has implementations of some services:
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

Which overlaps with all the services that we need, so we shouldn't need to write any extra services.

So, the list of thing(s) that we would need to do would be:
- Implement our own version of the CVT
  - Thankfully some Indians part of Manipal asked about this last year on an issue for `fkie/iop-core` repo,
  and the German dudes gave [a really good response](https://github.com/fkie/iop_core/issues/2).
  - Basically, we can use [wireshark](https://www.wireshark.org/) and some
  [LUA plugin that some other repo trying to do ROS+JAUS integration did](https://github.com/udmamrl/ROSJAUS/blob/master/Wireshark-dissector/Wireshark_JAUS_dissector.lua)
  - Then it'like unit test writing time, to check that all the specs mentioned in the IGVC competition manual are
  fulfilled

## Tasks for other subteams
The other important tasks that need to be done:

> The teams will implement a wireless 802.11 b/g or **hardwired Ethernet data link**

We didn't include a router in our budget, so hardwired Ethernet data link it is

> For the Wired Network, the judges will provide a **Gigabit Ethernet switch** that a team may plug their subsystem
> into using a standard RJ-45 Ethernet connector 

> The team shall provide one **Connector Type A connector as specified in the Payloads IOP**.
> This connector shall be provided at a location that is easily accessible to the judges.
> At some point in the future of this competition, this connector may be used to add a judges’ or team payload to the
> platform. For the purposes of this IOP Challenge, the team **does not need to provide Gigabit Ethernet** at this
> connector – **Fast Ethernet** (**Translator's note**: Fast Ethernet means that we can
> "_carry traffic at the nominal rate of 100 Mbit/s_") is acceptable.
> The team **shall NOT connect power to this connector** at this time – **only the data lines shall be populated**

Regarding the "Connector Type A connector" (lmao weird wording), I have no idea what this is. But then also from
a previous section:

> #### 1.5.2 Payloads Requirements
> There are currently no payloads requirements 

Googling "Connector Type A" yields only USB Type A Connector, while "Payloads IOP" yields to
[this document](https://apps.dtic.mil/dtic/tr/fulltext/u2/a558824.pdf), which contains

> #### 5.1.2 Connectors
> This section defines requirements associated with the **physical/electrical connectors** employed to integrate
> subsystems and payload(s) to the UGV platform. This is defined in the **UGV IOP Payloads Profile**

From Wikipedia:

> The National Advanced Mobility Consortium (NAMC) makes the IOPs available at the
> https://namcgroups.org website for registered users.  

So..... for now I think it's a safe bet to say that this "Connector Type A connector" refers to just normal RJ45, since
it seems like thats what the payloads have on them.

## My thoughts on IOP
Since all the hard work has been done for us with `fkie/iop_core`, and we only need to write a CVT to verify that
everything works properly, I think that **it is worth doing this year**, though it probably **won't be that high of a
priority**.
