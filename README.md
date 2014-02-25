iCub Tactile Grasp
=================

The Tactile Grasp is a module which perfors several types of object grasping using different levels of control and compliance.
Available grasping modes are:
 * Soft grasp
 * Tough grasp
 * Compliant grasp


Soft Grasp
----------
The Soft Grasp uses feedback from the fingertip tactile sensors to stop the grasping action once contact with an object is detected.


Tough Grasp
-----------
The Tough Grasp does not use any feedback and will continue with the grasping action regardless if the fingertips are sensing anything.


Compliant Grasp
--------------
The Compliant Grasp uses a PID controller to obtain and maintain a desired level of contact with the grasped object.
