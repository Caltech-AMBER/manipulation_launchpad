#import robot/gripper controllers
import urx
from pyrobotiqur import RobotiqGripper
#import helpers
import time
import numpy as np
from ur10e_utils import *

#set accel/vel limits
aj = 1      # rad/s²
vj = 0.5      # rad/s

al = 0.1    # m/s²
vl = 0.03   # m/s

#robot's ip address on our private network
UR_IP = "192.168.0.2" 

#initialize the robot, loosely based on our gripper
rob = urx.Robot(UR_IP)
rob.set_tcp((0, 0, 0.1, 0, 0, 0))
rob.set_payload(1.5, (0, 0, 0.07))
time.sleep(0.2)  #leave some time to robot to process the setup commands

#initialize the gripper and connect/activate it
g = RobotiqGripper(UR_IP, port=63352, timeout=20.0)
g.connect()
g.activate()



#open, close the gripper, then leave it halfway
print("Opening gripper...")
g.open(speed=128, force=10)
p = 50
print(f"Moving gripper to {p}%...")
g.move_percent(p, speed=128, force=128)
print("Closing gripper...")
g.close(speed=200, force=200)


pos_a = (
        0.0,
        -np.pi/2-.1,
        0.1,
        -np.pi/2,
        0.0,
        0.0
)

pos_b = (
        0.0,
        -np.pi/2,
        0.0,
        -np.pi/2,
        0.0,
        0.0
)

# #we want to start not at a singluarity wrt to the directions we care about
# rob.movej(pos_a, aj, vj, wait=True)
# rob.stopj(aj)
# print("Tool pose at start: ",  rob.getl())
# startPose = rob.getl()


# #we want to move so that we avoid commands outside the workspace
# rob.movel((0, 0, -.2, 0, 0, 0), al, vl, relative=True,wait=True)  # move relative to current pose
# rob.stopl(al)

# rob.translate((-0.2, 0, 0), al, vl)  #move tool and keep orientation
# rob.stopl(al)

# rob.translate((0, 0, -.2), al, vl)  # move relative to current pose
# rob.stopl(al)

# rob.translate((.4,0, 0), al, vl)  #move tool and keep orientation
# rob.stopl(al)

# rob.translate((-.2,0, 0), al, vl)  #move tool and keep orientation
# rob.stopl(al)


# print("Center tool pose is: ",  rob.getl())


# rob.rx -= 0.1  # rotate tool around X axis
# rob.ry -= 0.1  # rotate tool around Y axis
# rob.rx += 0.1  # rotate tool around X axis
# rob.ry += 0.1  # rotate tool around Y axis

# curPos = rob.getl()
# offset = (0, 0, 0.2, 0, 0, 0)
# rob.movel((curPos[0]+offset[0],curPos[1]+offset[1],curPos[2]+offset[2],curPos[3]+offset[3],curPos[4]+offset[4],curPos[5]+offset[5]), al, vl, relative=False)  # move relative to current pose
# rob.stopl(al)
# secondPose = rob.getl()

rob.movej(pos_b, aj, vj,wait=True)
rob.stopj(aj)
homePose = rob.getl()
print("Current tool pose is: ",  rob.getl())
# print("The start pose was: ",  startPose)
# print("The second pose was: ",  secondPose)


### IK Example
T_bg = np.eye(4)
T_bg[0:3, 0:3] = R_from_RPY(35/36*np.pi, 0.0, 0.0) # face nearly downwards, but avoid singular configuration
T_bg[0:3, 3] = np.array([0.0, -0.7, 0.3])

T_flange_g = np.eye(4)
T_flange_g[2, 3] = 0.2 # assume gripper is 20cm long

curr_angles = rob.getj() # get current position as reference
joint_angles = IK(T_bg, T_flange_g, curr_angles) # solve for requisite joint angles

rob.movej(joint_angles, aj, vj,wait=True) # go to calculated joint angles
rob.stopj(aj)



rob.close()
