import pybullet as pb
import pybullet_data
import time
import numpy as np 
import cv2

def Calc_angVels(desired_angVel):                #takes the desired angle of steering and returns the
    A = np.array([[     2.,      0., 0.353627], #driving angular velocity of each wheel in form
                  [    -1.,  1.7319, 0.353521], #np.array([[w1],[w2],[w3]]) (column matrix)
                  [    -1., -1.7319, 0.353521]])
    calc_angvel = np.dot(A,desired_angVel)
    return calc_angvel

physicsClient = pb.connect(pb.GUI)
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
pb.setGravity(0,0,-9.8)

plane = pb.loadURDF("plane.urdf")
body = pb.loadURDF("//home/astitva/Projects/robotics/RoboReG-research-Gyrosphere/urdf/GyroSphere.urdf",[0,0,2.5])
 
fwd = 0
side = 0   
motor_torque = 50  

def callback():#Dummy function for the track-bars
    pass


cv2.namedWindow('controls')
#Creating Track-Bars that can be used to adjust the PID values in real time.

#Setting the lower and upper limits on track-bars
cv2.createTrackbar('P', 'controls', 0, 500, callback)
cv2.createTrackbar('I', 'controls', 0, 500, callback)
cv2.createTrackbar('D', 'controls', 0, 500, callback)

#Creating three different track-bars for each P-I-D
#And Loading the PID constants from the trackbars
P=cv2.getTrackbarPos('P', 'controls')/10 
I=cv2.getTrackbarPos('I', 'controls')/10000
D=10*cv2.getTrackbarPos('D', 'controls')

#press escape key to execute
k=cv2.waitKey(1) & 0xFF #This is needed to keep the track-bars active in real time
# Basically cv2.waitKey(1) returns a 32-bit integer and 0xFF makes first 24 numbers = 0 to cpompare the last 8 bits(i.e. number between 0-255) in order to verify input from our keyboard.

# def moveCar(base_torque, action):  #Enter the motor control here to move the car, give base torque and action calculated as input
    
#     '''
#     2=Front left
#     3=Front Right
#     4=Rear Left
#     5=Rear Right
#     '''
    
#     mode=pb.TORQUE_CONTROL

#     left=base_torque+action
#     right=base_torque-action

#     print("reqd_torque_left=",left)
#     print("reqd_torque_right=",right)

#     pb.setJointMotorControl2(body,jointIndex=2,controlMode=mode,force=left)
#     pb.setJointMotorControl2(body,jointIndex=3,controlMode=mode,force=right)
#     pb.setJointMotorControl2(body,jointIndex=4,controlMode=mode,force=left)
#     pb.setJointMotorControl2(body,jointIndex=5,controlMode=mode,force=right)

desired_state = 10 

integral = 0 
derivative = 0
prev_error = 0

def calc_error(): #You can calculate the error and required action using this function

    global integral
    global derivative
    global prev_error
    global desired_state
    P=cv2.getTrackbarPos('P', 'controls')/10 #Get P from trackbar, dividing P by 10 to get it into range of 0-50 from 0-500 as desired value is in range of 0-50 and track-bar return values between 0-500
    I=cv2.getTrackbarPos('I', 'controls')/10000 #Get I from trackbar, dividing I by 10000 to get it into range of 0-0.05 from 0-500 as desired value is in range of 0-0.05 and track-bar return values between 0-500
    D=10*cv2.getTrackbarPos('D', 'controls') #Get D from trackbar, desired value is in range of 0-5000 only

    # P=291
    # I=146
    # D=340

    k = cv2.waitKey(1) #This is needed to keep the track-bars active in real time real time

    pos, orn = pb.getBasePositionAndOrientation(body) #Get the position and orientation of the base of the robot
    state=pos[1]
    #Getting the state, i.e. the current altitude of the drone
  
    error=state-desired_state # This is basically ep
    derivative = error - prev_error # This is basically ed
     #The D term is the difference in current error and prev error, As the simulation is called at regular intervals and the time difference is constant, so k/delta(t) is another constant. It gives us the rate at which the error is changing.
    prev_error = error #Updating the prev error for using in next loop

    if(error>-0.1 and error<0.1):
        integral+=error
    pid = P * error + D * derivative+I*integral
    action=pid

    return action

while(True):
    keys = pb.getKeyboardEvents()
    if keys.get(pb.B3G_RETURN) == 1:
        # for k,v in keys.items():
        #     if(k == pb.B3G_UP_ARROW and (v & pb.KEY_IS_DOWN)):
        #         fwd = -0.1*motor_torque
        #     if(k == pb.B3G_DOWN_ARROW and (v & pb.KEY_IS_DOWN)):
        #         fwd = 0.1*motor_torque
        #     if(k == pb.B3G_LEFT_ARROW and (v & pb.KEY_IS_DOWN)):
        #         side = 0.1*motor_torque
        #     if(k == pb.B3G_RIGHT_ARROW and (v & pb.KEY_IS_DOWN)):
        #         side = -0.1*motor_torque
        #     if(v & pb.KEY_WAS_RELEASED):
        #         fwd = 0
        #         side = 0
        pb.resetSimulation() #Simulation is reseted
        pb.setGravity(0, 0, -10)

        plane = pb.loadURDF("plane.urdf")
        body = pb.loadURDF("/home/sanidhya/roboclub/Control and Dynamics_CnD/RoboReG-Gyrosphere/kinematicsSim/outershell.urdf",[0,0,2.5])

        while(True):

            pb.stepSimulation()
            time.sleep(1./240.)
            position,orn=pb.getBasePositionAndOrientation(body)
            x=position[0]
            y=position[1]

            action=calc_error()

            fwd=0
            side = -0.1*motor_torque+action

            # print("forward=",fwd)
            # print("side=",side)
            print("position=",y)

            desired_ang_vel = np.array([[side],
                                        [fwd],
                                        [0]])
            angVels = Calc_angVels(desired_ang_vel)
            #print(angVel) 
            w1 = angVels[0][0]
            w2 = angVels[1][0]
            w3 = angVels[2][0]
            #print(angVel)
            #print(pb.getBasePositionAndOrientation(body))
                    
            T1 = np.array([ 0.3333*w1,      0*w1,0.9428*w1]) #Torque generated by motor 1
            T2 = np.array([-0.1667*w2, 0.2887*w2,0.9428*w2]) #Torque generated by motor 2
            T3 = np.array([-0.1667*w3,-0.2887*w3,0.9428*w3]) #Torque generated by motor 3
            
            print(T1+T2+T3)
            # print(pb.getBaseVelocity(body)[1])

            pb.applyExternalTorque(body,-1,T1,pb.LINK_FRAME) #Applying Torque generated by motor 1
            pb.applyExternalTorque(body,-1,T2,pb.LINK_FRAME) #Applying Torque generated by motor 2
            pb.applyExternalTorque(body,-1,T3,pb.LINK_FRAME) #Applying Torque generated by motor 3   
    

        # pb.close()   
#cv2.destroyAllWindows()
