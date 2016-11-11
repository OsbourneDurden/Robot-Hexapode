print('passage_entre_obstacle loaded') 
""" dernier edit 11/11/23:45"""
""" a rajouter : passage a cote d'un obstacle """
from math import sqrt

def passage_entre_obstacle(obst_1x,obst_1y,obst_2x,obst_2y,dist_1,dist_2,desti_x,desti_y):
    """ utilisation : liste1,liste2 = passage_entre_obstacle(...)
    obst_1 correspond a l'obstacle de droite et obst_2 a l'obstacle de gauche"""

    """cet algo considere que le robot est toujours a la position (0,0) ce qui simplifie les calculs -au moins davatange que ce que coute le changement de base-"""
    """a est une valeur qui varie entre 0 et 1 probablement le pas dans l algo"""
    a=0
    """on peut envisager de changer la valeur ce parametre, a priori 0.01 de base"""
    pas =0.01
    trajectx = []
    trajecty = []
    dist = dist_1+dist_2
    """ calcul des points repulses """

    repulse_1x,repulse_1y,repulse_2x,repulse_2y=repulse1(obst_1x,obst_1y)
    repulse_3x,repulse_3y,repulse_4x,repulse_4y=repulse2(obst_2x,obst_2y)

    while a<=1:
        
        direct1x=( 3*repulse_1x*a*(1-a)**2 + 3*repulse_2x*a**2*(1-a) + desti_x*a**3 )
        direct1y=( 3*repulse_1y*a*(1-a)**2 + 3*repulse_2y*a**2*(1-a) + desti_y*a**3 )
        direct2x=( 3*repulse_3x*a*(1-a)**2 + 3*repulse_4x*a**2*(1-a) + desti_x*a**3 )
        direct2y=( 3*repulse_3y*a*(1-a)**2 + 3*repulse_4y*a**2*(1-a) + desti_y*a**3 )
        temp1= (dist_1/dist)*direct1x + (dist_2/dist)*direct2x
        temp2= (dist_1/dist)*direct1y + (dist_2/dist)*direct2y
        trajectx.append(temp1)
        trajecty.append(temp2)
        """a=a+pas   // arrondit degueux qui casse tout"""
        a=round(a+pas,4)
    return trajectx , trajecty
    

def passage_entre_obstacle_direction(obst_1x,obst_1y,obst_2x,obst_2y,dist_1,dist_2,desti_x,desti_y,direction_x,direction_y):
    """ utilisation : liste1,liste2 = passage_entre_obstacle(...)
    obst_1 correspond a l'obstacle de droite et obst_2 a l'obstacle de gauche
    les composantes x,y de la direction sont a definir lorsque le projet avancera davantage."""

    """cet algo considere que le robot est toujours a la position (0,0) ce qui simplifie les calculs -au moins davatange que ce que coute le changement de base-"""
    """a est une valeur qui varie entre 0 et 1 probablement le pas dans l algo"""
    a=0
    """on peut envisager de changer la valeur ce parametre, a priori 0.01 de base"""
    pas =0.01
    trajectx = []
    trajecty = []
    dist = dist_1+dist_2
    """ calcul des points repulses """

    repulse_1x,repulse_1y,repulse_2x,repulse_2y=repulse1(obst_1x,obst_1y)
    repulse_3x,repulse_3y,repulse_4x,repulse_4y=repulse2(obst_2x,obst_2y)

    while a<=1:
        
        direct1x=( 4*direction_x*a*(1-a)**3 + 6*repulse_1x*a**2*(1-a)**2 + 4*repulse_2x*a**3*(1-a) + desti_x*a**4 )
        direct1y=( 4*direction_y*a*(1-a)**3 + 6*repulse_1y*a**2*(1-a)**2 + 4*repulse_2y*a**3*(1-a) + desti_y*a**4 )
        direct2x=( 4*direction_x*a*(1-a)**3 + 6*repulse_3x*a**2*(1-a)**2 + 4*repulse_4x*a**3*(1-a) + desti_x*a**4 )
        direct2y=( 4*direction_y*a*(1-a)**3 + 6*repulse_3y*a**2*(1-a)**2 + 4*repulse_4y*a**3*(1-a) + desti_y*a**4 )
        temp1= (dist_1/dist)*direct1x + (dist_2/dist)*direct2x
        temp2= (dist_1/dist)*direct1y + (dist_2/dist)*direct2y
        trajectx.append(temp1)
        trajecty.append(temp2)
        """a=a+pas /// arrondit degueux qui casse tout"""
        a=round(a+pas,4)
    return trajectx , trajecty


def repulse1(obst_1x,obst_1y):
    """return la liste des points necessaires au calcul de la trajectoire
    fonction a utiliser uniquement dans les passage_entre_obstacle"""
    repulser = []
    vect1= []
    vect2= []
    vect1.append(obst_1x)
    vect1.append(obst_1y)
    vect2=perpendiculaire(vect1[0],vect1[1])
    vect2[0]=vect2[0]*sqrt(3)/2
    vect2[1]=vect2[1]*sqrt(3)/2
    vect1[0]=vect1[0]/2
    vect1[1]=vect1[1]/2
    repulser.append(vect1[0]+vect2[0])
    repulser.append(vect1[1]+vect2[1])
    repulser.append(obst_1x+repulser[0])
    repulser.append(obst_1y+repulser[1])
    return repulser


def repulse2(obst_2x,obst_2y):
    """return la liste des points necessaires au calcul de la trajectoire
    fonction a utiliser uniquement dans les passage_entre_obstacle"""
    repulser = []
    vect1= []
    vect2= []
    vect1.append(obst_2x)
    vect1.append(obst_2y)
    vect2=perpendiculaire(vect1[0],vect1[1])
    vect2[0]=vect2[0]*sqrt(3)/2
    vect2[1]=vect2[1]*sqrt(3)/2
    vect2[0]=-vect2[0]
    vect2[1]=-vect2[1]
    vect1[0]=vect1[0]/2
    vect1[1]=vect1[1]/2
    repulser.append(vect1[0]+vect2[0])
    repulser.append(vect1[1]+vect2[1])
    repulser.append(obst_2x+repulser[0])
    repulser.append(obst_2y+repulser[1])
    return repulser


def perpendiculaire(vect_x,vect_y):
    vect = []
    vect.append(-vect_y)
    vect.append(vect_x)
    return vect
    
def norm(vect_x,vect_y):
    """ calcul de la norm, a priori inutile ici"""
    norm = vect_x*vect_x+vect_y*vect_y
    norm = sqrt(norm)
    return norm    


