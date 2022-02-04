#mathematical model of proposed Design

import numpy as np
import math
import matplotlib.pyplot as plt
import transformations as tfs

org, xaxis, yaxis, zaxis = [0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]
def get_circle(center,radius,orn):
    #T = np.concatenate((Rz,[[0],[0],[1]]), axis = 1)
    #T = np.concatenate((T,[[0,0,0,1]]),axis = 0)
    phi ,r = np.mgrid[0:2*np.pi:20j, 0:radius:5j]
    x = r*np.cos(phi)
    y = r*np.sin(phi)
    z = 0  

    Rx = tfs.rotation_matrix(orn[0], xaxis)
    Ry = tfs.rotation_matrix(orn[1], yaxis)
    Rz = tfs.rotation_matrix(orn[2], zaxis)
    R = tfs.concatenate_matrices(Rx, Ry, Rz)
    #R = np.dot(Rx,Ry)
    #R = np.dot(R,Rz)
    R.T[:][3] = np.concatenate((center,[1]),axis = None)
    circle = np.dot(R, [[x],[y],[z],[1]])
    print("############################")
    print(R)
    print("############################")
    return circle
#######################################################

fig = plt.figure()
ax = plt.axes(projection = '3d')



v1 = np.array([(8/9)**0.5,0,-1/3])
v2 = np.array([-(2/9)**0.5,(2/3)**0.5,-1/3])
v3 = np.array([-(2/9)**0.5,-(2/3)**0.5,-1/3])
v4 = np.array([0,0,1])

t1 = 0.9*v1 + 0.4*np.array([0.3333,0,0.9428])
t2 = 0.9*v2 + 0.4*np.array([-0.1667,0.2887,0.9428])
t3 = 0.9*v3 + 0.4*np.array([-0.1667,-0.2887,0.9428])
t4 = v4 #+ 0.4*np.array([0,1,0])
origin = np.array([0,0,0])

vertices = np.array([v1,v2,v3,v4])
tangents = np.array([t1,t2,t3,t4])

master_orn = np.array([np.pi/2,np.pi/2,np.pi/2])
Rz = tfs.rotation_matrix(master_orn[0],zaxis)
Ry = tfs.rotation_matrix(master_orn[1],yaxis)
R = tfs.concatenate_matrices(Rz,Ry)

for v in vertices:
    v = np.dot(np.concatenate((v,[1]),axis = None),R)
    v = v[0:3]
print(vertices)

u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
x = np.cos(u)*np.sin(v)
y = np.sin(u)*np.sin(v)
z = np.cos(v)
ax.plot_wireframe(x, y, z, color="k",linewidth = 1.0)

u, v = np.mgrid[0:2*np.pi:20j, np.pi*0.75:np.pi:10j]
x = np.cos(u)*np.sin(v)*0.8
y = np.sin(u)*np.sin(v)*0.8
z = np.cos(v)*0.8
ax.plot_surface(x, y, z,color = 'g')#, cmap ='viridis', edgecolor ='black')

theta = np.pi*0.75
#phi = np.linspace(0,2*np.pi,10)
#r = np.linspace(0,1,10)
phi,r = np.mgrid[0:2*np.pi:20j,0:0.8:10j]

x = r*np.cos(phi)*np.sin(theta)
y = r*np.sin(phi)*np.sin(theta)
z = r*np.cos(theta)

ax.plot_surface(x, y, z, color = 'g')#, cmap ='viridis', edgecolor ='black')


for i in range(4):
    v = vertices[i]
    t = tangents[i]
    ax.plot3D([0,v[0]],[0,v[1]],[0,v[2]],linewidth = 5.0,color = 'b')
    ax.plot3D([t[0],0.9*v[0]],[t[1],0.9*v[1]],[t[2],0.9*v[2]],linewidth = 3.0,color = 'r')

_r = 1/(2*np.cos(0.75*np.pi-np.arccos(-1/3)))*np.sin(0.75*np.pi)
for i in range(3):
    v = vertices[i]*1/2
    ax.plot3D([
                v[0],_r*np.cos(0.66667*math.pi*(i))],
                [v[1],_r*np.sin(0.66667*math.pi*(i))],
                [v[2],_r*np.cos(0.75*np.pi)],
                linewidth = 5.0,
                color = 'y'
    )

#print(circle)

centers = [0.9*v1,0.9*v2,0.9*v3]
#orientations = [[np.pi/2,0,0],[np.pi/2,-np.pi/3,0],[np.pi/2,np.pi/3,0]]
orientations = [[0,0.3398,0],[-np.pi/6,-0.4,0],[np.pi/6,-0.4,0]]
for i in range(3):
    circle = get_circle(centers[i],0.2,orientations[i])
    ax.plot_surface(circle[0][0],circle[1][0],circle[2][0],color = 'b')
plt.show()