import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

patte_longueur_avant_bras = 1.
patte_longueur_bras = 0.7

corps_largeur = 0.8
corps_longueur = 2.

N_pattes = 6
pied_angle_defaut = [-20, 0, 20]
hanche_angle_defaut = [15, 15, 15]
genou_angle_defaut = [15, 15, 15]

def plot_structure():
    fig = plt.figure()
    ax = p3.Axes3D(fig)

    H = get_Height()
    body_points = [[-corps_longueur/2, -corps_largeur/2, H], [corps_longueur/2, -corps_largeur/2, H], [corps_longueur/2, corps_largeur/2, H], [-corps_longueur/2, corps_largeur/2, H]]
    for i in range(4):
        ax.plot([body_points[i][0], body_points[(i+1)%4][0]], [body_points[i][1], body_points[(i+1)%4][1]], [body_points[i][2], body_points[(i+1)%4][2]], 'k-')
    
    pattes_attaches = [np.array(body_points[0]), (np.array(body_points[0]) + np.array(body_points[1]))/2, np.array(body_points[1])]
    print pattes_attaches

    side = -1
    for i in range(N_pattes/2):
        inter_point = pattes_attaches[i] + patte_longueur_bras*np.array([cos(torad(hanche_angle_defaut[i]))*sin(torad(pied_angle_defaut[i])), side*cos(torad(hanche_angle_defaut[i]))*cos(torad(pied_angle_defaut[i])), sin(torad(hanche_angle_defaut[i]))])
        print inter_point
        ax.plot([pattes_attaches[i][0], inter_point[0]], [pattes_attaches[i][1], inter_point[1]], [pattes_attaches[i][2], inter_point[2]],'r-')
        final_point = inter_point + patte_longueur_avant_bras*np.array([sin(torad(genou_angle_defaut[i]))*sin(torad(pied_angle_defaut[i])), side*sin(torad(hanche_angle_defaut[i]))*cos(torad(pied_angle_defaut[i])), -cos(torad(hanche_angle_defaut[i]))])
        ax.plot([final_point[0], inter_point[0]], [final_point[1], inter_point[1]], [final_point[2], inter_point[2]],'r-')

    pattes_attaches = [np.array(body_points[3]), (np.array(body_points[3]) + np.array(body_points[2]))/2, np.array(body_points[2])]
    print pattes_attaches

    side = +1
    for i in range(N_pattes/2):
        inter_point = pattes_attaches[i] + patte_longueur_bras*np.array([cos(torad(hanche_angle_defaut[i]))*sin(torad(pied_angle_defaut[i])), side*cos(torad(hanche_angle_defaut[i]))*cos(torad(pied_angle_defaut[i])), sin(torad(hanche_angle_defaut[i]))])
        print inter_point
        ax.plot([pattes_attaches[i][0], inter_point[0]], [pattes_attaches[i][1], inter_point[1]], [pattes_attaches[i][2], inter_point[2]],'r-')
        final_point = inter_point + patte_longueur_avant_bras*np.array([sin(torad(genou_angle_defaut[i]))*sin(torad(pied_angle_defaut[i])), side*sin(torad(hanche_angle_defaut[i]))*cos(torad(pied_angle_defaut[i])), -cos(torad(hanche_angle_defaut[i]))])
        ax.plot([final_point[0], inter_point[0]], [final_point[1], inter_point[1]], [final_point[2], inter_point[2]],'r-')
    while 1:
        a = raw_input('')
        print a
    plt.show(block=False)

def get_Height():
    heights = []
    for i in range(N_pattes/2):
        heights += [-patte_longueur_bras*sin(torad(hanche_angle_defaut[i])) + patte_longueur_avant_bras*cos(torad(genou_angle_defaut[i]))]
    return max(heights)

def torad(x):
    return x*(pi/180)
