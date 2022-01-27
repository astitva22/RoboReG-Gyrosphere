import numpy as np
import pybullet as p
import controlpy
import scipy
import pybullet_data
import matplotlib.pyplot as plt
import time

M = 1.0                        #mass
I = 0.6666666666667            #moment of inertia
R = 1.0                        #Radius

A = np.array([[  0.,  1.],
              [  0.,  0.]])
B = np.array([[0],
              [R/I]])        


class LQR_control:

    def __init__(self):

        self.Q = np.array([[ 500.0,   0.],
                           [    0.,   500.]])
        self.R = [[1000]]
        self.K,self.S,self.e = controlpy.synthesis.controller_lqr(A,B,self.Q,self.R)

    def callback(self,data,target):
        X = data-target
        u_t=-np.matmul(self.K,X)
        X_dot = (np.matmul(A,X)+np.matmul(B,u_t))
         
        return X_dot

    def callback_q(self,data):
        q = data.data
        self.Q = np.array([[ q,   0],[  0, 10*q]])
        self.K,self.S,self.e = controlpy.synthesis.controller_lqr(A,B,self.Q,self.R)
        
    def callback_r(self,data):
        r = data.data
        self.R = r
        self.K,self.S,self.e = controlpy.synthesis.controller_lqr(A,B,self.Q,self.R)

def synthesizeData(robot):
    #write here
    pos = p.getBasePositionAndOrientation(robot)[0]
    vel = p.getBaseVelocity(robot)[0]

    data = np.array([[pos[0]],        #returning x,y component of position and velocity
                     [pos[1]],         
                     [vel[0]],
                     [vel[1]]])
    ###############
    return data         

if __name__ == "__main__":

    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.8)

    p.loadURDF("plane.urdf")
    bot = p.loadURDF("/home/astitva/Projects/robotics/RoboReG-research-Gyrosphere/kinematicsSim/outershell.urdf",[0,0,2.5])

    my_controller_x = LQR_control()
    my_controller_y = LQR_control()

    trq_min = 100
    trq_max = 0
    trq_x,trq_y = 0,0

    #fig,ax = plt.subplots()
    
    t_mark = time.time()
    
    while True :
        data = synthesizeData(bot)
        data_x,data_y = data[::2],data[1::2]
        
        print(" data_x = \n {} \n data_y = \n {}".format(data_x,data_y))
        
        #ax.plot(time.time()-t_mark,p.getBasePositionAndOrientation(bot)[0][0])       
        trq_y = I/R*my_controller_x.callback(data_x,target = [[10.0],[0]])[1][0]
        trq_x = I/R*my_controller_y.callback(data_y,target = [[10.0],[0]])[1][0]         #uncomment for 2d op
        
        p.applyExternalTorque(bot,-1,[trq_x,trq_y,0],p.WORLD_FRAME)   #applying the torque
        
        trq = np.sqrt(trq_x**2+trq_y**2)

        if(trq<trq_min): trq_min = trq
        if(trq>trq_max): trq_max = trq

        #print("data: {} trq: {}".format(data,trq))
        #print("current torque: {} | max_torque: {} | min_torque: {}".format(trq,trq_max,trq_min))
        p.stepSimulation()
        time.sleep(1./240.)

    plt.show()