import numpy as np
import numpy.linalg as ls

def getEquations(P,x,y):
    P0 = P[0,:]
    P1 = P[1,:]
    P2 = P[2,:]

    eq0 = x*P2 - P0
    eq1 = y*P2 - P1
    return eq0,eq1
if __name__ == "__main__":

    P0 = np.array([[0.999856, -0.0116099,-0.0123839 ,-0.0212208],[0.0108654,   0.998224, -0.0585804, -0.0865201],[0.013042,  0.0584374,   0.998206, -0.0983536]])
    P1 = np.array([[0.991318,  0.0243755 ,  0.129211,   0.166388],[-0.0255439,   0.999646, 0.00739273, -0.0604466],[-0.128985, -0.0106291 ,   0.99159 ,-0.0122441]])
    P2 = np.array([[0.989448, -0.00494251,    0.144807,    0.118725],[-0.005959,    0.997184 ,  0.0747529 ,  -0.170213],[-0.144768 ,  -0.074827 ,   0.986632 ,  -0.354791]])

    x = np.array([ 0.287258, 0.275077, 0.180175])
    y = np.array([ 0.288389, 0.230688, 0.0836684])

    A = np.zeros((6,3))
    B = np.zeros((6,1))
    print
    eq0,eq1 = getEquations(P0,x[0],y[0])
    A[0,:3] = eq0[:3]
    A[1,:3] = eq1[:3]
    B[0] = eq0[-1]
    B[1] = eq1[-1]

    eq0,eq1 = getEquations(P1,x[1],y[1])
    A[2,:3] = eq0[:3]
    A[3,:3] = eq1[:3]
    B[2] = eq0[-1]
    B[3] = eq1[-1]

    eq0,eq1 = getEquations(P2,x[2],y[2])
    A[4,:3] = eq0[:3]
    A[5,:3] = eq1[:3]
    B[4] = eq0[-1]
    B[5] = eq1[-1]

    a,b,c = ls.lstsq(A,B)[0]
    print(a,b,c)