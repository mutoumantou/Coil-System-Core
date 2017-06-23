# Coil-System-Core
The core code for 8 amplifiers. Aimed to provide a stable core script for everyone to start with.

P.S. This core is based on Omid's code. Future work will include:
Remove advanced functions.
Update dependent packages. (e.g. openCV)
Organize callback.c
Compare the codes written by different people => update the core.

==== List of files ====<br />
826api.h: Include file for applications for Sensoray's model 826 board<br />
AccelStepper.h: Arduino AccelStepper library. It provides an object-oriented interface for 2, 3 or 4 pin stepper motors. It is modified to control the amplifier driver.<br />
AutoFabrication.h:<br />
callbacks.h: Trigger functions with GUI. <br />
coilFieldControl.h: Relate current output with magnetic field.<br />
constantValue.h: Store constant values.<br />
FWcamera.h: camera control.<br />
math_subroutine.h: Mathematical functions.<br />
s826_subroutine.h: Amplifier driver control.<br />
twistField.h:<br />
vision.h:<br />
