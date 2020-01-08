# Firmware Refactor Design Document

Author: Dallas Downing

## Motivation

The firmware for IGVC has largely been neglected for the past few years, being an afterthought once more pressing problems have been addressed. However, we have reached a point where the basic code running on the mbed can no longer support the needs of the team. The goal of this firmware redesign is to tackle some of the main challenges with the firmware at last competition and make it easier to maintain and alter in the future. This will also (hopefully) motivate a new incentive to apply actual design decisions in making changes to the code later.

Here is a short list of the problems that have been identified with the firmware and the mbed. Note that this is not a comprehensive list as many issues depend on the electrical system and the main computer. Most of the issues are explained later in the design section.

All mbed code is contained in one `main.cpp` file, which makes understanding and editing the code difficult
Asynchronous networking code (for communicating with the computer) runs in the same thread as “real-time” code (PID control)
There are no errors messages sent from the mbed to the computer, making it difficult to see what the mbed is doing when it fails or encounters a problem
Messages sent between the computer and the mbed follow the same format with many optional fields instead of using dedicated message formats for different commands
The E-stop signal is treated as a normal digital input but should be an interrupt input to stop the robot as soon as possible

There are also a number of new features being added to the electrical system that the mbed now must support. Part of the redesign was made keeping these features in mind. The specifics of their implementation has been left out of this document, but it should be clear how one might implement such features using the new firmware.
A “virtual bumper” system using lidar lites that can warn and stop the robot preemptively before it hits an obstacle
A diagnostics system with various sensors for detecting hardware issues with the robot 

## Background

The mbed is one of the most important components of the robot. Without it, the computer would just be a fancy computer and the rest of the robot would be a metal box filled with wires and silicon. The mbed is the bridge that connects the computer with the rest of the electrical system. This section will briefly go over what the mbed does to provide context for the upcoming design changes.

Here are the main tasks that it performs:

- Connects to the computer to receive PID values and motor commands (in the form of speeds for the left and right motor)
- Performs PID calculations for bringing the motors to the right speeds
- Receives signals from the encoders and keeps track of motor speeds and direction
- Sends serial commands to the motor controller to properly drive the motors
- Checks for when the e-stop is triggered and disables the motors when it is
- Enables the flashing safety light when the robot is operating
- Sends feedback to the computer about the state of the robot (current motor speed, e-stop status, battery voltage)

The mbed and the computer communicate via Ethernet. The mbed basically acts as a server and sets up a TCP socket. The motor controller node on the computer connects to the socket and sends messages to the mbed. Here are the basic tasks that the ROS node performs:

- Sends the PID values that the mbed should use
- Subscribes to the `/motor` ROS topic and receives motor speeds to send to the mbed
- Advertises the encoder speeds, robot enable status, and battery voltages to the ROS topics `/encoders`, `/robot_enabled`, and `/battery`, respectively
- Keeps track of battery voltages over time and warns when it gets too low

Since much of the code on the mbed is changing, the motor controller node will also need to be updated to communicate properly. However, it’s overall structure will largely stay the same. Currently the mbed doesn’t do a lot of error reporting to the computer but the revisions to the electrical system will allow that. The mbed will monitor the robot more and thus will send more error information to the computer. The mbed will also support two more status lights for easier user debugging.

## Design

The new design of the firmware breaks down into 3 main categories: multi-threading, code structure, and message formats. There are some other miscellaneous design features described at the end of this section.

### Multi-Threading

The main problem plaguing the firmware is the fact that all code run by the mbed is contained in the same main while loop. All networking and motor control code occurs sequentially, which is not ideal for the system. The main change I propose is to create three new threads: the networking thread, the motor control thread, and the diagnostics thread.

The threading changes will require using RTOS in Mbed OS 5.

#### Networking

The networking thread will contain all the code for communicating with the computer. It will set up the TCP socket for the computer to connect to and will send and receive messages. The code for parsing requests should be contained in this thread. With the improved message formats proposed later in this document, the logic for parsing and forming the messages should be greatly simplified.

This will also be the starting point for the other threads. Ideally, the other threads will be started before establishing the TCP socket. The other threads would run idly in the background until necessary since the TCP socket could be reset if connection is lost.

#### Motor Control

The motor control thread will contain the code for the PID control loop. It will use a `MotorController` class (described below) to send commands to the motor controller. It will also keep track of the encoder ticks. This thread is fairly simple, as it will likely reuse a lot of existing code.

#### Diagnostics

The diagnostics thread will contain the code for communicating with the CAN devices and diagnostics board. It will be in charge of monitoring those devices and taking appropriate actions when necessary. The main function of the thread is constructing messages to send back to the computer for bagging and debugging. The first step in implementing this will be defining what the devices are and how they work. In the future, wrapper classes for the devices and diagnostics board may be useful. This code will change the most frequently in the coming year.

---

### Code Structure

Currently, the firmware follows no specific structure. All code is kept in one `main.cpp` file and follows no overall design pattern. The overall code structure will be fixed with the changes described in the threading section, but this section will propose some changes that will help justify those changes and will make implementing the threads easier. Most of these changes will remove code from the `main.cpp` file into additional header/cpp files.

#### `MotorController`

A `MotorController` wrapper class will be created to handle communicating with the motor controller and will abstract its functionality. This change will accomplish 2 things: declutter the code for sending commands to the motor and achieve the overall software goal of writing self-documenting code. Here is how the motor controller is currently called in the code:

``` Serial g_serial(p13, NC, 9600); ```

This does nothing to explain what it is or how it works. A similar issue can be found in the PID control loop. The latter half of the `pid()` method is dedicated to converting the speed into motor values. This code should be taken out of that method and moved to the MotorController class. Here are the base functions that the `MotorController` class should provide:

- Set the speeds of left and right motors independently
- Get the speeds sent to the motors
- Stop the motors
- Advanced control of the motors (slowly stop the motors, turn around, etc.)

#### `EncoderPair`

An `EncoderPair` wrapper class will accomplish the same objective as the `MotorController`. It will handle the interrupt input and the digital input for the encoders. It will keep track of and return the tick counts for each encoder. The main advantage for using this is that is removes some global variables and some functions from the main.cpp file. While this change is not super involved or critical, it will allow future modification to improve encoder performance.

#### E-Stop Status Interrupt

Currently the e-stop status is read in the middle of the main while loop. For safety, the e-stop signal should be treated like a real-time signal, and changes to that pin should be read from an interrupt function. This will allow the mbed to stop the motors as soon as possible upon receiving an e-stop signal. 

#### Miscellaneous Functions and Constants 

To further reduce clutter in the `main.cpp` file, a separate header/cpp file should be made for the various functions and constants that do not belong to any classes. For clarity, all constants should be made variables with the `const` keyword instead of macros. Variables and structs that serve little purpose (such as the pwm variables and the `SpeedPair` struct) should be removed entirely.

When the CAN devices and diagnostics boards are complete, new classes should be defined to help abstract their tasks. The details of these components are not included in this document as testing these boards still needs to take place. This also includes the functionality of the “virtual bumper” implemented with Lidar Lites.

---

### Network Messages

The current way that the mbed communicates with the server is with Protobuf messages. The big issue here is that each device (ROS node and the mbed) use their own message with several optional fields (a generic message being used as a multitool). This makes the logic for parsing the messages unnecessarily complex and makes understanding what the message does harder. This section will propose some new message types that a singular purpose, making parsing and constructing messages easier. 

#### `PIDValues`

This message is sent by ROS to the mbed. It will contain the PID values that the mbed should use. This includes the P, I, and D constants for the left and right motor and the KV value for feed-forwarding for both motors. This message should only be sent once when the Motor Controller ROS node is started. This will be the largest message sent by ROS to the mbed. When the mbed sets the PID constants, it should return a new message to the ROS node, confirming that the values were set correctly. Again, the mbed should only send this message once, when it receives the message from ROS.

There should be an additional field in the message for indicating whether the message is setting the constants (i.e. setPID = true, from ROS to mbed) or confirming the constants are correct (i.e. setPID = false, from mbed to ROS). This will be the only message that both sides send.

#### `SetMotorSpeeds`

This message is sent by ROS to the mbed. It will contain the speeds for each motor that the mbed should try to set. This will be the most sent message from ROS and should be the highest priority message for the mbed. The convention for the values is defined in the software stack.

Note that these are the only messages sent by ROS. Should more complex motor control be required, additional messages should be created and additional functions written in the `MotorController` class.

#### `MbedStatus`

This message is sent by the mbed to ROS. It will contain information concerning the motor control status for bagging and debugging purposes. It will also contain the e-stop status and battery voltage. This message will be sent fairly frequently, so it’s overall size should be restricted. It will only be sending the motor speeds, the output sent to motor controller, the time difference for this step, e-stop status, and battery voltage. Battery voltage may be removed from this message due to the addition of the diagnostics board if it is not needed frequently.

#### Diagnostics Messages

Since the diagnostics devices have not been well defined, it is impossible to determine what kind of monitoring information should be sent to the computer. Ideally, a singular summary message should be sent infrequently to the computer to make sure everything is working appropriately (i.e. one every 2 seconds). If certain information is needed more frequently, separate messages containing only the required data should be created.

It will be useful to create a generic string message for sending error messages to the computer. This would only be necessary if there are complex errors with the mbed code that cannot be detected with the various status lights. While its use case may not be easy to see at the moment, having the ability to send a text error message to the computer is a great advantage for debugging.

---

### Other Features

#### Watchdog Timer for Networking

The mbed should have a watchdog timer monitoring the TCP connection with the computer. Should the connection time-out, the mbed should stop the motors and attempt to send the computer a message. The timeout duration will require some testing with actual runs with the robot. The Watchdog class from mbed OS 5 can be used to implement it. 

A watchdog can also be implemented in the motor controller node with a longer timeout than the mbed. This can be used to automatically reconnect with the mbed if the connection is dropped. This should be disabled during testing if it’s implemented, but can be enabled when running the course to prevent run-stopping errors.

#### ROS Node Changes

Most of the changes occur in the firmware on the mbed but changes will also be required in the ROS node. Mainly, the new message formats need to be defined and the functions for parsing and constructing the messages will need to be updated. Additionally, the node will need to support the new diagnostics data sent from the CAN devices and diagnostics board. This may require a new ROS topic for data like this. Further details on this should be discussed after the diagnostics devices have been well-defined.

#### Status Lights

The latest revision of the logic board includes pins for external status LEDs (besides the on-board LEDs). These and the existing LEDs were excluded from this document because the new functionality of the firmware can be defined before deciding what 

---

## Required Work

The section will break down the implementation of this design document into _ major groups. This is intended to help create a timeline for the order of work and help assign people to different tasks.

1. **Define diagnostics devices**
    1. Determine what CAN devices are available
    2. Determine what data those devices collect and send
    3. Determine what data the diagnostics board will send
    4. Determine the priority for the data
    5. Determine what behavior is desired for the diagnostics thread on the mbed
    6. Create messages for the diagnostics data
    * (This will likely require more electrical documentation)
2. **Clean up the code**
    1. Implement the `MotorController` class
    2. Implement the `EncoderPair` class
    3. Implement any classes needed for the diagnostics devices
    4. Move constants and miscellaneous functions to a new header/cpp file
    5. Update e-stop detection with interrupt function
3. **Implement multithreading**
    1. Move PID control loop to its own thread
    2. Create a diagnostics thread (even if it is empty)
    3. Add data protection to global variables
    4. Implement watchdog timer for networking thread
4. **Add new message formats**
    1. Update messages definitions with protobuf in firmware and software
5. **Update ROS node**
    1. Add parsing support for new messages
    2. Add functions for creating new messages
    3. Add watchdog for network connection

## Future Work

The new design of the firmware provides a usable platform for some interesting features. These are some ideas that can be implemented once the design changes have been implemented.

### Slip Detection

Wheel slippage is a major issue with motor control, especially on grass, wet surfaces, or wet grass. With an encoder class and dedicated PID thread, implementing an algorithm for slipping detection should be easier. While the changes do not explicitly support wheel slip detection, it does make the code easier to modify.

### Joystick Control with Mbed

Currently the mbed does not use it’s additional USB pins. In a future iteration of the logic board, a USB port can be added for connecting a controller. A class can be added in the PID loop to connect to a controller with a joystick to allow controlling the robot without having to boot up the computer. This will make moving the robot long distances easier, given the robot has batteries.
