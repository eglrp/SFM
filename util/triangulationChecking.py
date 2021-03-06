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

    # Data from the dataset, undistorted.
    x = np.array([ 0.304645, 0.287684, 0.181598])
    y = np.array([ 0.305772, 0.241292, 0.0839387])
    pointsUndistorted = [(a,b) for a,b in zip(x,y)]

    A = np.zeros((6,3))
    B = np.zeros((6,1))
    eq0,eq1 = getEquations(P0,pointsUndistorted[0][0],pointsUndistorted[0][1])
    A[0,:3] = eq0[:3]
    A[1,:3] = eq1[:3]
    B[0] = eq0[-1]
    B[1] = eq1[-1]

    eq0,eq1 = getEquations(P1,pointsUndistorted[1][0],pointsUndistorted[1][1])
    A[2,:3] = eq0[:3]
    A[3,:3] = eq1[:3]
    B[2] = eq0[-1]
    B[3] = eq1[-1]

    eq0,eq1 = getEquations(P2,pointsUndistorted[2][0],pointsUndistorted[2][1])
    A[4,:3] = eq0[:3]
    A[5,:3] = eq1[:3]
    B[4] = eq0[-1]
    B[5] = eq1[-1]

    # Getting the solution, LS
    a,b,c = ls.lstsq(A,B)[0]
    X = np.zeros((3,1))
    X[0] = a[0]
    X[1] = b[0]
    X[2] = c[0]
    print np.linalg.norm(np.dot(A,X) - B)

    # Using the ground truth as solution
    GT = np.zeros((3,1))
    GT[0] = 0.469954
    GT[1] = 0.455934
    GT[2] = -1.54736
    print np.linalg.norm(np.dot(A,GT) - B)

    # Let's project the points from the GT
    pointsProjectedFromGT = []
    GTHomogeneous = np.vstack((GT,1))

    p = np.dot(P0,GTHomogeneous)
    point = (-p[0]/p[2],-p[1]/p[2])
    pointsProjectedFromGT.append(point)

    p = np.dot(P1,GTHomogeneous)
    point = (-p[0]/p[2],-p[1]/p[2])
    pointsProjectedFromGT.append(point)

    p = np.dot(P2,GTHomogeneous)
    point = (-p[0]/p[2],-p[1]/p[2])
    pointsProjectedFromGT.append(point)

    A = np.zeros((6,3))
    B = np.zeros((6,1))
    eq0,eq1 = getEquations(P0,pointsProjectedFromGT[0][0],pointsProjectedFromGT[0][1])
    A[0,:3] = eq0[:3]
    A[1,:3] = eq1[:3]
    B[0] = eq0[-1]
    B[1] = eq1[-1]

    eq0,eq1 = getEquations(P1,pointsProjectedFromGT[1][0],pointsProjectedFromGT[1][1])
    A[2,:3] = eq0[:3]
    A[3,:3] = eq1[:3]
    B[2] = eq0[-1]
    B[3] = eq1[-1]

    eq0,eq1 = getEquations(P2,pointsProjectedFromGT[2][0],pointsProjectedFromGT[2][1])
    A[4,:3] = eq0[:3]
    A[5,:3] = eq1[:3]
    B[4] = eq0[-1]
    B[5] = eq1[-1]

    # Getting the solution, LS
    a,b,c = ls.lstsq(A,B)[0]
    XGT = np.zeros((3,1))
    XGT[0] = a[0]
    XGT[1] = b[0]
    XGT[2] = c[0]
    print np.linalg.norm(np.dot(A,X) - B)
