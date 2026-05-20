# manipulation_launchpad
This should be your one-stop shop for all things related to organization, safety, scheduling, onboarding etc. for the GT-B159 Arm Manipulation lab.

# Schedule
If you would like to reserve time to work on the UR10e or Franka arms, please use the following Google Sheets. Lab members should do their best to honour reservations made on the Google Sheet system, although recognize there may be extenuating circumstances where PIs request urgent demos etc. that may result in a change in schedule. Look out for and support one another!

**Reservations must be made 24 hours in advance. If there is free time, then it is first come, first serve on the day of.**

[UR10e Booking Calendar](https://docs.google.com/spreadsheets/d/1dOwqv0ddb73xd9MdWtdaJUp9k-KB0nYpMaP0OMFp9vw/edit?usp=sharing)

[Franka Booking Calendar](https://docs.google.com/spreadsheets/d/1JxUUr9k4kDnLuvDe_D6YIsBMcdYbYPkfLR9bNocWoUs/edit?usp=sharing) 

# Documents

GT-B159 Safety Orientation.pdf - this is an overview of the safety training and safety rules for the lab space.

UR 10e Setup Instructions.pdf - this is an overview of how to turn on and connect to the UR10e robot arm.

ur10e_debug.md - this is a file that keeps a running log of debugged errors on the UR10e arm, so that you can avoid bugs caught by other users.

ur10e_utils.py - this python file contains forward and inverse kinematics for the UR10e robot arm. Note that the IK does not deal with singularities currently (so don't try axis aligned final poses), and is in the modified DH convention, while the FK uses the classical convention (as does the robot when you are sending it to a joint position). An overview for the derivation of the IK can be found at [Williams](https://people.ohio.edu/williams/html/PDF/UniversalRobotKinematics.pdf). This means you must convert from your IK solutions to the appropriate FK values before passing joint movement commands.

ur10e_example.py - this python file provides a simple overview of how to move the UR10e using the urx library.

# Safety
You need to complete both the generic [Caltech laboratory safety training](https://safety.caltech.edu/root-pages/lab-safety-orientation?utm_source=copilot.com), and an arm-lab-specific safety orientation to be approved for access to the arm lab. This orientation follows what is set out in the Safety Orientation pdf file. Once completed, the Safety Coordinator can send a signed safety sheet to Lynn Seymour who can apply for you to have access.


