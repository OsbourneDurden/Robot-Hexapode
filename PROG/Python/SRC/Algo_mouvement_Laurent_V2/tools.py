import math
import numpy as np

def rotate(vector, new_orientation_vector):
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
    
def flight(I, F, h_up, n_points):
    '''Creates a trajectory to be followed going through I and F, with a departure angle of flight_angle and going up to h_up higher than the highest point between I and F.
    TODO : Works by iteration, can probably be optimized to set max height more intelligently
    WARNING : Stability not asserted. Looks like it works...
    Input :
        - I : 3D array-like being the start of the flight
        - F : 3D array-like being the end of the flight
        - h_up : minimal heigth of the flight
        - n_points : number of points for this flight
    '''
    # We start by putting both points at the same height
    if I[2] < F[2]: # If the arrival point is above the takeoff point, we use an intermidiate point being at the vertical of I
        Cstart = [I[0], I[1], F[2]]
        Cend = F
        height_compensation = np.abs(F[2]-I[2])
        comp_type = "first"
    elif I[2] > F[2]:
        Cstart = I
        Cend = [F[0], F[1], I[2]]
        height_compensation = np.abs(F[2]-I[2])
        comp_type = "last"
    else:
        Cstart = I
        Cend = F
        height_compensation = 0
        n_points_compensation = 0
        comp_type = "none"
    print "Compensation of {0} as type {1}".format(height_compensation, comp_type)

    # Now we start computing the differents values useful for the flight
    D = np.linalg.norm(Cend-Cstart)
    R  = ((D/2)**2 + h_up**2)/(2*h_up)
    C = (Cstart+Cend)/2 - (R - h_up)*np.array([0,0,1])
    thetamax = np.arccos(np.dot(Cstart-C, Cend-C)/R**2)

    L = R*thetamax
    print "Flight computation parameters : D = {0}, R = {1}, C = {2}, thetamax = {3}, L = {4}, n_points = {5}".format(D, R, C, thetamax, L, n_points)
    if comp_type != "none":
        n_points_compensation = min(n_points - 4, max(1,3+int(n_points*(height_compensation/(height_compensation + L)))))

    points = [I]
    if comp_type == "first":
        points += [np.array([x,y,z]) for x,y,z in zip(np.linspace(I[0], Cstart[0], n_points_compensation), np.linspace(I[1], Cstart[1], n_points_compensation), np.linspace(I[2], Cstart[2], n_points_compensation))]

    u = np.cross(np.array([0,0,1]), (Cend-Cstart)/D)
    for theta in np.linspace(0, thetamax, n_points - n_points_compensation)[1:]:
        c = np.cos(theta)
        s = np.sin(theta)
        rotation_matrix = np.array([[c + (1-c)*u[0]**2, u[0]*u[1]*(1-c) - u[2]*s, u[0]*u[2]*(1-c) + u[1]*s],
                                    [u[0]*u[1]*(1-c) + u[2]*s, c + (1-c)*u[1]**2, u[1]*u[2]*(1-c) - u[0]*s],
                                    [u[0]*u[2]*(1-c) - u[1]*s, u[1]*u[2]*(1-c) + u[0]*s, c + (1-c)*u[2]**2]])
        points += [C + np.dot(rotation_matrix, (Cstart-C))]

    if comp_type == "last":
        points += [np.array([x,y,z]) for x,y,z in zip(np.linspace(Cend[0], F[0], n_points_compensation), np.linspace(Cend[1], F[1], n_points_compensation), np.linspace(Cend[2], F[2], n_points_compensation))]

    print "FInal flight generated : {0}".format(points)
    return points

def file_loader(filename):

    data_dictionnary = {}
    f = open(filename, 'r')
    for line_raw in f:
        line = line_raw[:-1]
        if line.count('&') == 1:
            data_dictionnary[line.split('&')[0]] = line.split('&')[1]
        elif line.count('&') > 1:
            data_dictionnary[line.split('&')[0]] = [value for value in line.split('&')[1:]]
        else:
            #Empty line
            None
    f.close()

    return data_dictionnary
