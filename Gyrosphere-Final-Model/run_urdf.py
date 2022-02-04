# urdf runner

import pybullet as p
import numpy as np
import time
import pybullet_data
from scipy.spatial.transform import Rotation
import os 

physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
plane = p.loadURDF("plane.urdf")

parent_dir = os.path.dirname(__file__)
bot = p.loadURDF(
        parent_dir+"/Gyrosphere/urdf/GyroSphere.urdf",
        [0,0,2.6],
        useFixedBase = 1,
    )

for i in range(p.getNumJoints(bot)):
    print(p.getJointInfo(bot,i))


p.setJointMotorControlMultiDof(bot,22,p.POSITION_CONTROL,[0,0,0,1], targetVelocity=[0,0,0], positionGain=0,velocityGain=1,force=[0.005,0.005,0.005]) #enabling torque Control

motor_torque = 1000
Tx = 0
Ty = 0
while True:
  
        while(True):
                keys = p.getKeyboardEvents()
                for k,v in keys.items():
                        if(k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
                                Ty = -0.1*motor_torque
                                print("Upward")
                        if(k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
                                Ty= 0.1*motor_torque
                                print("Downward")
                        if(k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
                                Tx = 0.1*motor_torque
                                print("leftWard")
                        if(k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
                                Tx = -0.1*motor_torque
                                print("RightWard")
                        if(v & p.KEY_WAS_RELEASED):
                                Tx = 0
                                Ty = 0
                        if(k == p.B3G_RETURN and (v & p.KEY_IS_DOWN)):
                                p.resetSimulation()

                                p.setGravity(0,0,-9.8)
                                plane = p.loadURDF("plane.urdf")
                                bot = p.loadURDF(
                                        "/home/astitva/Projects/robotics/RoboReG-research-Gyrosphere/urdf_test/Model-2/urdf/GyroSphere.urdf",
                                        [0,0,2.6],
                                        useFixedBase = 1,
                                    )
                                p.setJointMotorControlMultiDof(bot,22,p.POSITION_CONTROL,[0,0,0,1], targetVelocity=[0,0,0], positionGain=0,velocityGain=1,force=[0.005,0.005,0.005])

                desired_force = [[Tx],
                                 [Ty],
                                  [0]]
                #print("Base: {}  OuterShel: {}".format(p.getLinkState(bot,0)[4:6],p.getLinkState(bot,22)[4:6]))
                base_pos,base_orn = p.getLinkState(bot,0)[4],p.getLinkState(bot,0)[5]
                #base_pos,base_orn = [0,0,0],[0,0,0,1]
                shell_pos,shell_orn = p.getLinkState(bot,22)[4],p.getLinkState(bot,22)[5]
                tf = p.multiplyTransforms(base_pos,base_orn,shell_pos,shell_orn)
                R_matrix = np.array(Rotation.from_quat(tf[1]).as_matrix())

                actuation_force = R_matrix @ desired_force

                p.setJointMotorControlMultiDof(bot,22,p.TORQUE_CONTROL,force = actuation_force)
                #print(tf)
                #print(actuation_force)
                print(p.getJointStateMultiDof(bot,22)[1])
                p.stepSimulation()
                time.sleep(1./240.)