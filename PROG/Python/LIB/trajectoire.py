print('cmd_trajectoire loaded')

""" a rajouter : passage a cote d'un obstacle """
from math import sqrt

def cmd_trajectoire():
    """liste des fonctions:
    passage_entre_obstacle , passage_entre_obstacle_direction , repulse1 , repulse2 ,
    esquive_obstacle_droite , esquive_obstacle_direction_droite , esquive_obstacle_gauche ,
    esquive_obstacle_direction_gauche , perpendiculaire , norm , fleche , fleche_norm , reliste ,
    passage_entre_obstacle_vect , fleche_vect , fleche_norm_vect , norm_vect ,
    choix_sur_fleche_vect , distance_"""

    
def passage_entre_obstacle(obst_1x,obst_1y,obst_2x,obst_2y,dist_1,dist_2,desti_x,desti_y):
    """ utilisation : liste1,liste2 = passage_entre_obstacle(...)
    Avec obst_1 la position a l'obstacle de droite et obst_2 la position a l'obstacle de gauche
    Avec dist la distance, desti la position de la destination
    Avec liste1,liste2 les coordonnees  en x (resp y) de la trajectoire

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

    """ calcul de la trajectoire """
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
    Avec obst_1 la position a l'obstacle de droite et obst_2 la position a l'obstacle de gauche
    Avec dist la distance, desti la position de la destination et direction la direction initiale
    Avec liste1,liste2 les coordonnees  en x (resp y) de la trajectoire
    
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

    """ calcul de la trajectoire """
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
    fonction a utiliser uniquement dans les passage_entre_obstacle et esquive_obstacle
    repulse1 s'occupe de l'obstacle de droite"""
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
    fonction a utiliser uniquement dans les passage_entre_obstacle et esquive_obstacle
    repulse2 s'occupe de l'obstacle de gauche"""
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


def esquive_obstacle_droite(obst_x,obst_y,desti_x,desti_y):
    """ utilisation : liste1,liste2 = esquive_obstacle_droite(...)
    Avec obst_x et obst_x la position de l'obstacle a eviter
    Avec desti_x et desti_y la position de la destination
    Avec liste1,liste2 les coordonnees  en x (resp y) de la trajectoire"""
    
    """cet algo considere que le robot est toujours a la position (0,0) ce qui simplifie les calculs -au moins davatange que ce que coute le changement de base-"""
    """a est une valeur qui varie entre 0 et 1 probablement le pas dans l algo"""
    a=0
    """on peut envisager de changer la valeur ce parametre, a priori 0.01 de base"""
    pas =0.01
    trajectx = []
    trajecty = []

    """ calcul des points repulses """

    repulse_1x,repulse_1y,repulse_2x,repulse_2y=repulse1(obst_x,obst_y)

    """ calcul de la trajectoire """
    while a<=1:
        
        directx=( 3*repulse_1x*a*(1-a)**2 + 3*repulse_2x*a**2*(1-a) + desti_x*a**3 )
        directy=( 3*repulse_1y*a*(1-a)**2 + 3*repulse_2y*a**2*(1-a) + desti_y*a**3 )
        trajectx.append(directx)
        trajecty.append(directy)
        """a=a+pas /// arrondit degueux qui casse tout"""
        a=round(a+pas,4)
    return trajectx , trajecty

    
def esquive_obstacle_direction_droite(obst_x,obst_y,desti_x,desti_y,direction_x,direction_y):
    """ utilisation : liste1,liste2 = esquive_obstacle_direction_droite(...)
    Avec obst_x et obst_x la position de l'obstacle a eviter
    Avec desti_x et desti_y la position de la destination et direction la direction initiale
    Avec liste1,liste2 les coordonnees  en x (resp y) de la trajectoire"""

    """cet algo considere que le robot est toujours a la position (0,0) ce qui simplifie les calculs -au moins davatange que ce que coute le changement de base-"""
    """a est une valeur qui varie entre 0 et 1 probablement le pas dans l algo"""
    a=0
    """on peut envisager de changer la valeur ce parametre, a priori 0.01 de base"""
    pas =0.01
    trajectx = []
    trajecty = []

    """ calcul des points repulses """

    repulse_1x,repulse_1y,repulse_2x,repulse_2y=repulse1(obst_x,obst_y)

    """ calcul de la trajectoire """
    while a<=1:
        
        directx=( 4*direction_x*a*(1-a)**3 + 6*repulse_1x*a**2*(1-a)**2 + 4*repulse_2x*a**3*(1-a) + desti_x*a**4 )
        directy=( 4*direction_y*a*(1-a)**3 + 6*repulse_1y*a**2*(1-a)**2 + 4*repulse_2y*a**3*(1-a) + desti_y*a**4 )
        trajectx.append(directx)
        trajecty.append(directy)
        """a=a+pas /// arrondit degueux qui casse tout"""
        a=round(a+pas,4)
    return trajectx , trajecty


def esquive_obstacle_gauche(obst_x,obst_y,desti_x,desti_y):
    """ utilisation : liste1,liste2 = esquive_obstacle_gauche(...)
    Avec obst_x et obst_x la position de l'obstacle a eviter
    Avec desti_x et desti_y la position de la destination
    Avec liste1,liste2 les coordonnees  en x (resp y) de la trajectoire"""

    """cet algo considere que le robot est toujours a la position (0,0) ce qui simplifie les calculs -au moins davatange que ce que coute le changement de base-"""
    """a est une valeur qui varie entre 0 et 1 probablement le pas dans l algo"""
    a=0
    """on peut envisager de changer la valeur ce parametre, a priori 0.01 de base"""
    pas =0.01
    trajectx = []
    trajecty = []

    """ calcul des points repulses """

    repulse_1x,repulse_1y,repulse_2x,repulse_2y=repulse2(obst_x,obst_y)

    """ calcul de la trajectoire """
    while a<=1:
        
        directx=( 3*repulse_1x*a*(1-a)**2 + 3*repulse_2x*a**2*(1-a) + desti_x*a**3 )
        directy=( 3*repulse_1y*a*(1-a)**2 + 3*repulse_2y*a**2*(1-a) + desti_y*a**3 )
        trajectx.append(directx)
        trajecty.append(directy)
        """a=a+pas /// arrondit degueux qui casse tout"""
        a=round(a+pas,4)
    return trajectx , trajecty

    
def esquive_obstacle_direction_gauche(obst_x,obst_y,desti_x,desti_y,direction_x,direction_y):
    """ utilisation : liste1,liste2 = esquive_obstacle_direction_gauche(...)
    Avec obst_x et obst_x la position de l'obstacle a eviter
    Avec desti_x et desti_y la position de la destination et direction la direction initiale
    Avec liste1,liste2 les coordonnees  en x (resp y) de la trajectoire"""

    """cet algo considere que le robot est toujours a la position (0,0) ce qui simplifie les calculs -au moins davatange que ce que coute le changement de base-"""
    """a est une valeur qui varie entre 0 et 1 probablement le pas dans l algo"""
    a=0
    """on peut envisager de changer la valeur ce parametre, a priori 0.01 de base"""
    pas =0.01
    trajectx = []
    trajecty = []

    """ calcul des points repulses """

    repulse_1x,repulse_1y,repulse_2x,repulse_2y=repulse2(obst_x,obst_y)

    """ calcul de la trajectoire """
    while a<=1:
        
        directx=( 4*direction_x*a*(1-a)**3 + 6*repulse_1x*a**2*(1-a)**2 + 4*repulse_2x*a**3*(1-a) + desti_x*a**4 )
        directy=( 4*direction_y*a*(1-a)**3 + 6*repulse_1y*a**2*(1-a)**2 + 4*repulse_2y*a**3*(1-a) + desti_y*a**4 )
        trajectx.append(directx)
        trajecty.append(directy)
        """a=a+pas /// arrondit degueux qui casse tout"""
        a=round(a+pas,4)
    return trajectx , trajecty

    
def perpendiculaire(vect_x,vect_y):
    """calcul d'un vecteur perpendiculaire d'un vecteur donne
    usage : perpen = perpendiculaire(vect_x,vect_y)
    Avec vect_x et vect_y les composantes du vecteur
    Avec perpen le vecteur perpendiculaire au premier (il aura meme norme)"""
    vect = []
    vect.append(-vect_y)
    vect.append(vect_x)
    return vect
    
def norm(vect_x,vect_y):
    """calcul de la norme d'un vecteur
    usage : norm = norm(vect_x,vect_y)
    Avec vect_x et vect_y les composantes du vecteur
    Avec norm la valeur de la norme du vecteur"""
    norm = vect_x*vect_x+vect_y*vect_y
    norm = sqrt(norm)
    return norm    

def fleche(trajectx , trajecty):
    """calcul la direction du mouvement
    usage : flechex,flechey = fleche_norm(trajectx , trajecty)
    Avec trajectx et trajecty les positions du mouvement
    Avec flechex,flechey les directions entre ces mouvements"""
    flechex = []
    flechey = []
    i=0
    while i<len(trajectx)-1:
        
        flechex.append( trajectx[i+1]-trajectx[i] )
        flechey.append( trajecty[i+1]-trajecty[i] )
        i=i+1
    return flechex,flechey

def fleche_norm(trajectx , trajecty):
    """calcul la direction du mouvement et la normalise
    usage : flechex,flechey = fleche_norm(trajectx , trajecty)
    Avec trajectx et trajecty les positions du mouvement
    Avec flechex,flechey les directions entre ces mouvements"""
    flechex = []
    flechey = []
    i=0
    while i<len(trajectx)-1:
        
        norm_=norm( trajectx[i+1]-trajectx[i],trajecty[i+1]-trajecty[i] )
        flechex.append( (trajectx[i+1]-trajectx[i])/float(norm_) )
        flechey.append( (trajecty[i+1]-trajecty[i])/float(norm_) )
        i=i+1
    return flechex,flechey
    
def reliste(liste1,liste2):
    """mastic temporaire permettant de regrouper les coordonnees dans une liste de liste"""
    i=0
    liste=[];
    while i<len(liste2):
        
        liste.append([liste1[i],liste2[i]])
        i=i+1
    return liste
    

def passage_entre_obstacle_vect(obst_1x,obst_1y,obst_2x,obst_2y,dist_1,dist_2,desti_x,desti_y):
    """ utilisation : liste = passage_entre_obstacle(...)
    Avec obst_1 la position a l'obstacle de droite et obst_2 la position a l'obstacle de gauche
    Avec dist la distance, desti la position de la destination
    Avec liste les coordonnees  en x et y de la trajectoire

    les composantes x,y de la direction sont a definir lorsque le projet avancera davantage."""

    """cet algo considere que le robot est toujours a la position (0,0) ce qui simplifie les calculs -au moins davatange que ce que coute le changement de base-"""
    """a est une valeur qui varie entre 0 et 1 probablement le pas dans l algo"""
    a=0
    """on peut envisager de changer la valeur ce parametre, a priori 0.01 de base"""
    pas =0.01
    traject = []

    dist = dist_1+dist_2
    """ calcul des points repulses """

    repulse_1x,repulse_1y,repulse_2x,repulse_2y=repulse1(obst_1x,obst_1y)
    repulse_3x,repulse_3y,repulse_4x,repulse_4y=repulse2(obst_2x,obst_2y)

    """ calcul de la trajectoire """
    while a<=1:
        
        direct1x=( 3*repulse_1x*a*(1-a)**2 + 3*repulse_2x*a**2*(1-a) + desti_x*a**3 )
        direct1y=( 3*repulse_1y*a*(1-a)**2 + 3*repulse_2y*a**2*(1-a) + desti_y*a**3 )
        direct2x=( 3*repulse_3x*a*(1-a)**2 + 3*repulse_4x*a**2*(1-a) + desti_x*a**3 )
        direct2y=( 3*repulse_3y*a*(1-a)**2 + 3*repulse_4y*a**2*(1-a) + desti_y*a**3 )
        temp1= (dist_1/dist)*direct1x + (dist_2/dist)*direct2x
        temp2= (dist_1/dist)*direct1y + (dist_2/dist)*direct2y
        traject.append([temp1,temp2])
        """a=a+pas   // arrondit degueux qui casse tout"""
        a=round(a+pas,4)
    return traject


def fleche_vect(traject):
    """calcul la direction du mouvement
    usage : fleche,fleche = fleche_norm(traject)
    Avec traject les positions du mouvement
    Avec fleche les directions entre ces mouvements"""
    fleche = []
    i=0
    while i<len(traject)-1:
        
        fleche.append( [traject[i+1][0]-traject[i][0] , traject[i+1][1]-traject[i][1]] )
        i=i+1
    return fleche


def fleche_norm_vect(traject):
    """calcul la direction du mouvement et la normalise
    usage : fleche = fleche_norm(traject)
    Avec traject les positions du mouvement
    Avec fleche les directions entre ces mouvements"""
    fleche = []
    i=0
    while i<len(traject)-1:
        
        norm_=norm( traject[i+1][0]-traject[i][0],traject[i+1][1]-traject[i][1] )
        fleche.append( [(traject[i+1][0]-traject[i][0])/float(norm_),(traject[i+1][1]-traject[i][1])/float(norm_)] )
        i=i+1
    return fleche


def norm_vect(vect):
    """calcul de la norme d'un vecteur
    usage : norm = norm(vect_x,vect_y)
    Avec vect_x et vect_y les composantes du vecteur
    Avec norm la valeur de la norme du vecteur"""
    norm = vect[0]*vect[0]+vect[1]*vect[1]
    norm = sqrt(norm)
    return norm


def choix_sur_fleche_vect(traject,position_,longueur):
    """calcul d'un point a 'distance' du robot
    usage : point=choix_sur_fleche_vect(traject,position_,longueur)
    Avec traject les positions du mouvement, position_ la position actuel du robot, longueur la distance de deplacement
    /!\ /!\ longueur est un parametre tres important a fixer soit meme /!\ /!\
    de meme position_ est la position du robot par rapport a l'origine du repere du calcul de la trajectoire utilisee
    Avec point les coordonnees du point obtenu"""
    point=[]
    i=0
    d=0
    parcourt=norm_vect(position_)
    while ( ( (d<longueur) or (parcourt>=norm_vect(traject[i])) ) and i<100 ):
        
        i=i+1
        d=distance_(traject[i],position_)

    point=traject[i]
    return point


def distance_(traject_i,position):
    """calcul de la distance euclidienne entre deux points
    usage : dis=distance_(traject_i,position)
    Avec traject_i et position les deux points
    Avec dis le resultat du calcul de la distance entre ces deux points""" 
    dis=sqrt( (traject_i[0]-position[0])*(traject_i[0]-position[0]) + (traject_i[1]-position[1])*(traject_i[1]-position[1]) )
    return dis
        
        
