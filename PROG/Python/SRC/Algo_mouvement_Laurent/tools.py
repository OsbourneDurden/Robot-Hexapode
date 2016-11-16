import math
import numpy as np

def rotate(vector, new_orientation_vector):
# Return a rotated vector from a previous one, and a new origin, initially defined by [1, 0]
# rotate(v, [1, 0]) returns v
    return np.array([vector[0]*new_orientation_vector[0] - vector[1]*new_orientation_vector[1], +vector[1]*new_orientation_vector[0] + vector[0]*new_orientation_vector[1]])

def intersect(A, B):
    if A[0] == 'C' and (B[0] == 'S' or B[0] == 'L'):
        objectA = A
        objectB = B
        inter_type = 1
    elif B[0] == 'C' and (A[0] == 'S' or A[0] == 'L'):
        objectA = B
        objectB = A
        inter_type = 1
    elif A[0] == 'C' and B[0] == 'C':
        if A[2] > B[2]:
            objectB = A
            objectA = B
            inter_type = 2
        else:
            objectA = A
            objectB = B
            inter_type = 2
    elif A[0] in ['L', 'S'] and B[0] in ['L', 'S']:
        objectA = A
        objectB = B
        inter_type = 3
        
        
    else:
        print "Intersection not handled : A = {0} and B = {1}".format(A[0], B[0])
        return None
    
    if inter_type == 1: #Cercle avec segment/droite
        lambdas = []
        if objectB[0] == 'S':
            u = (objectB[2]-objectB[1])/np.linalg.norm(objectB[2]-objectB[1])
        else:
            u = objectB[2]/np.linalg.norm(objectB[2])
        delta = (np.dot(objectB[1]-objectA[1], u)**2) - ((np.linalg.norm(objectB[1]-objectA[1]))**2 - objectA[2]**2)
        if delta == 0 :
            lambdas += [-np.dot(objectB[1]-objectA[1], u)]
        elif delta > 0:
            lambdas += [(-np.dot(objectB[1]-objectA[1], u)-np.sqrt(delta))]
            lambdas += [(-np.dot(objectB[1]-objectA[1], u)+np.sqrt(delta))]
        inters=[]
        for l in lambdas:
            if objectB[0]=='L' or (l>0 and l<np.linalg.norm(objectB[1]-objectB[2])):
                inters += [objectB[1]+l*u]
        return inters
    if inter_type == 2: #Cercle avec cercle
        D = np.linalg.norm(objectA[1] - objectB[1])
        if D == 0:
            return []
        if (D < (objectB[2] - objectA[2])) or (D > objectA[2] + objectB[2]):
            return []
        elif D == (objectB[2] - objectA[2]):
            return [objectA[1] + objectA[2]/D * (objectA[1] - objectB[1])]
        elif D == (objectB[2] + objectA[2]):
            return [objectA[1] + objectA[2]/D * (objectB[1] - objectA[1])]
        else:
            x = (D**2 + objectA[2]**2 - objectB[2]**2)/(2*D)
            K = objectA[1] + x/D*(objectB[1]-objectA[1])
            u = rotate((objectB[1] - objectA[1])/D, [0, 1])
            return [K + np.sqrt(objectA[2]**2-x**2)*u, K - np.sqrt(objectA[2]**2-x**2)*u]
    if inter_type == 3:
        if objectA[0] == 'S':
            u1 = (objectA[2]-objectA[1])/np.linalg.norm(objectA[2]-objectA[1])
        else:
            u1 = objectA[2]
        if objectB[0] == 'S':
            u2 = (objectB[2]-objectB[1])/np.linalg.norm(objectB[2]-objectB[1])
        else:
            u2 = objectB[2]
        if (u1[0] == u2[0] and u1[1] == u2[1]) or (u1[0] == -u2[0] and u1[1] == -u2[1]):
            return []
        v1 = rotate(u1, [0, 1])
        v2 = rotate(u2, [0, 1])
        l1 = np.dot(objectB[1]-objectA[1], v2)/np.dot(u1, v2)
        if objectA[0]=='S' and (np.linalg.norm(objectA[2]-objectA[1])<l1 or l1<0):
            return []
        l2 = np.dot(objectA[1]-objectB[1], v1)/np.dot(u2, v1)
        if objectB[0]=='S' and (np.linalg.norm(objectB[2]-objectB[1])<l2 or l2<0):
            return []
        return [objectA[1] + l1*u1] 
