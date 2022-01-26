import numpy as np
import pybullet as p
import controlpy
import scipy
import pybullet_data
import matplotlib.pyplot as plt
import time
import os

M = 1.0
I = 0.6666666666667
R = 1.0

A = np.array([[    0.,   0.,   1.,  0.],
              [    0.,   0.,   0.,  1.],
              [    0.,   0.,   0.,  0.],
              [    0.,   0.,   0.,  0.]])
B = np.array([[   0.,  0.],
              [   0.,  0.],
              [  R/I,  0.],
              [   0., R/I]])        


class LQR_control:

    def __init__(self):

        self.Q = np.array([[ 500.,   0.,   0.,   0.],
                           [   0., 500.,   0.,   0.],
                           [   0.,   0., 500.,   0.],
                           [   0.,   0.,   0., 500.]])
        self.R = [[1000.,    0.],
                  [   0., 1000.]]
        self.K,self.S,self.e = controlpy.synthesis.controller_lqr(A,B,self.Q,self.R)

    def callback(self,data,target):
        X = data-target
        #_B = np.matmul(self.K,B)
        #X_dot = np.matmul(A,X)-np.matmul(_B,X)
        u_t=-np.matmul(self.K,X)
        X_dot = (np.matmul(A,X)+np.matmul(B,u_t))
         # returning the difference of current state and next state to compensate error
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
    data = np.array([[pos[0]],
                     [pos[1]],
                     [vel[0]],
                     [vel[1]]])
    ##########################################
    return data         

if __name__ == "__main__":

    current_dir = os.path.dirname(__file__)   ##just to get the directory

    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.8)

    p.loadURDF("plane.urdf")
    bot = p.loadURDF(current_dir+"/outershell.urdf",[0,5.0,2.5])

    my_controller = LQR_control()
    print(my_controller.K)
    print(my_controller.e)
    trq_min = 100
    trq_max = 0
    #fig,ax = plt.subplots()
    t_prev = time.time()
    
    max_target_disp = 0
    
    target_x,target_y = -10.0,-10.0


    
    while True :
        data = synthesizeData(bot)
        #ax.plot(time.time()-t_mark,p.getBasePositionAndOrientation(bot)[0][0])
        desired_target = [[target_x],
                          [target_y],
                          [      0.],
                          [      0.]]        
        x_dot = my_controller.callback(
                data,
                target = desired_target
            ).ravel()

        disp = np.sqrt((data-desired_target).ravel()[0]**2+(data-desired_target).ravel()[1]**2)
        if(disp>max_target_disp):
            max_target_disp = disp

        if(max_target_disp < 0.05):
            print("target reached")
            print(max_target_disp)
            #targetx,target_y = 0,0
            
        trq_y = I/R*x_dot[2]
        trq_x = I/R*x_dot[3]
        trq = np.sqrt(trq_x**2+trq_y**2)

        p.applyExternalTorque(bot,-1,[-trq_x,trq_y,0],p.LINK_FRAME)
        
        #print("data: {} trq: {}".format(data,trq))


        if(trq<trq_min): trq_min = trq
        if(trq>trq_max): trq_max = trq

        print("current torque: {} | max_torque: {} | min_torque: {}".format(trq,trq_max,trq_min))

        if(time.time()-t_prev >=1.0):
            max_target_disp = 0
            t_prev = time.time()
        
        p.stepSimulation()
        time.sleep(1./240.)

    plt.show()