# Motor-Movement
A library for used for running the  two stepper motors of an XY plotter

This library was used in a project where Makeblock's XY plotter drawing robot's firmware was upgraded to an enhanhed version. Originally, the controller software runs on Arduino which was replaced with NXP's LPCXpresso1549 microcontroller. The robot receives commands from the original Makeblock's mDraw software that turns vector images to G code commands. The robot then using its stepper motors and a servo motor controlled pencil and draws the image abiding by those commands.

Note that this library is only one part of the project.
