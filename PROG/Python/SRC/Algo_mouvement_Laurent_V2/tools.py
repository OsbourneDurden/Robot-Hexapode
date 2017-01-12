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
    
def flight(I, F, h_up, flight_angle, n_points):
    '''Creates a trajectory to be followed going through I and F, with a departure angle of flight_angle and going up to h_up higher than the highest point between I and F.
    TODO : Works by iteration, can probably be optimized to set max height more intelligently
    WARNING : Stability not asserted. Looks like it works...
    Input :
        - I : 2D array-like being the start of the flight
        - F : 2D array-like being the end of the flight
        - h_up : heigth of the flight
        - flight_angle : angle for takeoff. Landing must be vertical since we have less clue about height of this landing.
        - n_points : number of points for this flight
    '''
    max_tries = 10
    height_ref = max(I[1], F[1])
    n = 1
    P1 = I+n*np.array([np.cos(flight_angle), np.sin(flight_angle)])
    P2 = F+n*np.array([0., 1.])
    t = np.linspace(0, 1, n_points)
    B = reshape(kron((1-t)**3,I), (100,2)) + 3*reshape(kron(t*(1-t)**2, P1), (100,2)) + 3*reshape(kron((1-t)*t**2, P2), (100,2)) + reshape(kron(t**3, F), (100,2))
    for n_try in range(max_tries):
        n /= exp((1./h_up)*(max(B[:,1])-height_ref-h_up))
        P2 = F+n*np.array([0., 1.])
        plot(B[:,0], B[:,1])
        B = reshape(kron((1-t)**3,I), (100,2)) + 3*reshape(kron(t*(1-t)**2, P1), (100,2)) + 3*reshape(kron((1-t)*t**2, P2), (100,2)) + reshape(kron(t**3, F), (100,2))
    return B
