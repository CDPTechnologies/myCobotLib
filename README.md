# myCobot PI 6-DOF robot arm CDP interface

The **myCobotIO** is a communication I/O server for the myCobot 6-DOF arms with PI support.

Architecture:
* the IOServer is intended to be added to the application that will be run on the myCobot PI (using the CDP Studio ARMv6 64-bit toolkit)
* the IOServer controls the myCobot arm directly via serial interface(usually /dev/ttyAMA0). The serial transport parameters can be configured in the SerialTransport element of the IOServer.

Controllable features:
* Arm 6 joint states (angles) can be fetched
* Arm 6 joint states (angles) can be repositioned to the desired position using 2 different modes: by setting target positions only with global speed; or by setting target positions and target reach speeds for every joint.
* Arm actuator position (X/Y/Z + orientation), calculated by the myCobot can be fetched
* Arm actuator ATOM controller can be controlled: the LED color can be set and the button state under the the LED can be fetched
* myCobot arm Adaptive Gripper add-on can be controlled: both it's current state can be fetched and the gripper open/close state can be set

Dependencies:
* A myCobot arm with PI support (see https://www.elephantrobotics.com/en/mycobot-pi/)
* Optional myCobot arm Adaptive Gripper (see https://shop.elephantrobotics.com/products/adaptive-gripper)
