print('cmd_decision loaded')

''' vérifier (notemment sur scout, que les orientations matrice sont tjs bon)'''
from math import sqrt
'''from numpy import *'''
import numpy as np
import scipy as sci
from scipy import ndimage
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import time

def disstance_rapide(xd,yd,x,y):
    return abs(xd-x)+abs(yd-y)
    
    
def detect_direction(x,y,x_suiv,y_suiv):
    '''il va peut etre falloir changer les valeurs ici pour que cela represente
    encore les meme directions que sur matlab'''
    if y_suiv<y:
        return 1

    if y_suiv>y:
        return 3

    if x_suiv<x:
        return 2

    if x_suiv>x:
        return 4

        
def cre_graph(col,row,Carte_explo):
    
    graph =np.zeros([col.size,col.size],int)
    i=0
    j=0
    while i<col.size:
        while j<col.size:
            if abs(col[i]-col[j])+abs(row[i]-row[j])==1:
                if (Carte_explo[col[i],row[i],2]==128) or (Carte_explo[col[j],row[j],2]==128):
                    graph[i,j]=2
                else:
                    graph[i,j]=1
            j=j+1
        i=i+1
        j=0
    return graph
 
    
def envi(Carte_decision):

    h=np.array(np.mat('0 1 0; 1 0 1; 0 1 0'))
    BBB=sci.ndimage.filters.convolve(Carte_decision[:,:,0],h)
    BBB[BBB==1]=0
    Carte_decision[:,:,1]=BBB
    return Carte_decision    
    
def autorrize(autorise,x,y,Carte_nav_dym):
    if Carte_nav_dym(y,x,1)==0:
        autorize=0
    else:
        autorize=1;
    return autorize

def choix(Carte_decision,Carte_nav,Carte,position,position_depart,graph,col,row,suppr):

    BBB=Carte_decision[:,:,1]
    [col2,row2]=np.nonzero(BBB==2)
    '''attention il semblerait que si il n'y a qu'un seul element il bug un peu'''
    ''' (array([8]), array([2])) si un seul element contre (array([3, 8]), array([3, 2])) si deux elements
    problem de tupple il semblerait'''
    x=position[0]
    y=position[1]
    ignore = []
    for i in range(0,row.size):
        if graph[i,:].any()==0:
            ''' possible problem de notation ici peut etre qu'il faut le np.array'''
            ignore+=   [ [col[i] , row[i]] ] 
    
    ignore +=   [ [y , x] ]  
    ignore +=   [ suppr ] 
    ''' verifier cette ligne'''

    AA=np.array([col,row])
    BB=np.array([col2,row2])
    arr=(np.swapaxes(AA,0,1)).tolist()  
    rows=(np.swapaxes(BB,0,1)).tolist()
    '''elm.array pour le sens inverse'''
    
    c = []

    for arr_elem in arr:

        if arr_elem in rows:
            c += [arr_elem] 

    if len(ignore)!=0:
        c = [i for i in c if i not in ignore]

    ratio=[]
    for i in range(0,len(c)):
        if Carte_nav[c[i][0],c[i][1],1]==255:
            eloigne=abs(c[i][0]-position_depart[1])+abs(c[i][1]-position_depart[0])
            but=abs(c[i][0]-position[1])+abs(c[i][1]-position[0])
            ratio.append(float(but)/float(eloigne))
        else:
            ratio.append(0)




    if sum(i > 0 for i in ratio)!=0:
        ppoids1=min(i for i in ratio if i>0)
        choixx=ratio.index(ppoids1)
        yd=c[choixx][0]
        xd=c[choixx][1]
    else:
        xd=[]
        yd=[]

    return xd,yd,ppoids1


def choix3(Carte_decision,Carte_nav,Carte,position,position_depart,graph,col,row,suppr):

    GROS=4
    BBB=Carte_decision[:,:,1]
    [col2,row2]=np.nonzero(BBB==3)
    '''attention il semblerait que si il n'y a qu'un seul element il bug un peu'''
    ''' (array([8]), array([2])) si un seul element contre (array([3, 8]), array([3, 2])) si deux elements
    problem de tupple il semblerait'''
    x=position[0]
    y=position[1]
    ignore = []
    for i in range(0,row.size):
        if graph[i,:].any()==0:
            ''' possible problem de notation ici peut etre qu'il faut le np.array'''
            ignore+=  [ [col[i] , row[i]] ]
    
    ignore +=   [ [y , x] ]  
    ignore +=   [ suppr ] 
    ''' verifier cette ligne'''

    AA=np.array([col,row])
    BB=np.array([col2,row2])
    arr=(np.swapaxes(AA,0,1)).tolist()  
    rows=(np.swapaxes(BB,0,1)).tolist()
    '''elm.array pour le sens inverse'''
    
    c = []

    for arr_elem in arr:

        if arr_elem in rows:
            c += [arr_elem] 

    if len(ignore)!=0:
        c = [i for i in c if i not in ignore]

    ratio=[]
    for i in range(0,len(c)):
        if Carte_nav[c[i][0],c[i][1],1]==255:
            eloigne=abs(c[i][0]-position_depart[1])+abs(c[i][1]-position_depart[0])
            but=abs(c[i][0]-position[1])+abs(c[i][1]-position[0])
            valeur=float(but)/float(eloigne)
            if disstance_rapide(position[0],position[1],c[i][1],c[i][0])<=GROS:
                valeur=valeur/10
            ratio.append(valeur)
        else:
            ratio.append(0)


    if sum(i > 0 for i in ratio)!=0:
        ppoids2=min(i for i in ratio if i>0)
        choixx=ratio.index(ppoids2)
        yd=c[choixx][0]
        xd=c[choixx][1]
    else:
        xd=[]
        yd=[]

    return xd,yd,ppoids2
''' eventuellement rajouter une condition de protection s'il n'y a pas de
valeur superieur a 0 dans la liste ratio'''


def dijkkstraat(graph,x,y,xd,yd,col,row):
    ''' F est le facteur de trop lent'''
    
    AA=np.array([col,row])
    Elem=(np.swapaxes(AA,0,1)).tolist()
    pos_deb = Elem.index([y,x])
    pos_fin = Elem.index([yd,xd])
    pass_X=[xd]
    pass_Y=[yd]

    Table_traitement=np.ones([len(col),1],int)*-1
    Table_retenu=np.zeros([len(col),len(col)],int)

    Table_traitement[pos_deb]=pos_deb
    Table_retenu[pos_deb,:]=graph[pos_deb,:]
    next_min=0
    F=15
    dist=disstance_rapide(xd,yd,x,y)

    
    while Table_traitement[pos_fin]==-1:
        TT=Table_retenu.reshape(len(col)*len(col),1)
        if next_min>F*dist:
            return pass_X,pass_Y
        
        try:
            next_=min(i for i in TT if i >next_min)
        except(ValueError,IndexError) as err:
            return pass_X,pass_Y

        [rown,coln]= np.nonzero(Table_retenu==next_)
        c = [i for i in coln if i not in Table_traitement]
        if len(c)==0:
            next_min=next_
        else:
            addition=graph[c[0],:]
            addition= [i+next_ if i!=0 else i for i in addition]
            Table_retenu[c[0],:]=addition
            Table_traitement[c[0]]=c[0]

    if next_min>F*dist:
        return pass_X,pass_Y
    
    rown = pos_fin

    if 'next_' not in locals():
        return pass_X,pass_Y
    else:
        while Table_traitement[pos_deb]!=-1:
            mat_temp=Table_retenu[:,rown]
            prec=min(i for i in mat_temp if i>0)
            rown=np.nonzero(mat_temp==prec)
            c = [i for i in rown[0] if i in Table_traitement]
            if len(c)==0:
                Table_traitement[pos_deb]=-1;
            else:
                pass_X= [row[c[0]]] + pass_X
                pass_Y = [col[c[0]]] + pass_Y
                Table_traitement[c[0]]= -1
                rown=c[0]
    return pass_X,pass_Y

                
def pathss(x,y,Carte_explo,Carte_nav_dym,Carte_decision,Carte,position,position_depart,suppr):
    
    col,row=np.nonzero(Carte_nav_dym[:,:,1]!=0)

    graph = cre_graph(col,row,Carte_nav_dym)
    
    xd,yd,ppoids1=choix(Carte_decision,Carte_nav_dym,Carte,position,position_depart,graph,col,row,suppr)
    xd2,yd2,ppoids2=choix3(Carte_decision,Carte_nav_dym,Carte,position,position_depart,graph,col,row,suppr)

    if type(xd)==list:
         xd=xd2
         yd=yd2
    else:
        if ppoids1>10*ppoids2:
            xd=xd2
            yd=yd2

    pass_X,pass_Y=dijkkstraat(graph,x,y,xd,yd,col,row)
    erreur=0
    if len(pass_X)==1:
        suppr+=[ [yd,xd] ]

        while len(pass_X)==1:
            
            xd,yd,ppoids1=choix(Carte_decision,Carte_nav_dym,Carte,position,position_depart,graph,col,row,suppr)
            xd2,yd2,ppoids2=choix3(Carte_decision,Carte_nav_dym,Carte,position,position_depart,graph,col,row,suppr)
            if type(xd)==list:
                xd=xd2
                yd=yd2
            else:
                if ppoids1>10*ppoids2:
                    xd=xd2
                    yd=yd2
                    
            pass_X,pass_Y=dijkkstraat(graph,x,y,xd,yd,col,row)
            suppr+= [ [yd,xd] ]
            erreur=erreur+1
            
def update_carte_nav(Carte_explo):

    List1=np.nonzero(Carte_explo[:,:,0]==0)
    List2=np.nonzero(Carte_explo[:,:,1]==0)
    LList1=(np.swapaxes(List1,1,1)).tolist()
    AAA=np.swapaxes(LList1,0,1)
    LList2=(np.swapaxes(List2,1,1)).tolist()
    BBB=np.swapaxes(LList2,0,1)
    c=[]
    for j in range(0,len(LList2[0])):
        c+=[ i for i in AAA if (BBB[j]==i).all()]

    A=Carte_explo[:,:,0]
    AA=Carte_explo[:,:,1]

    for i in range(0,len(c)):
        cx=c[i][0]
        cy=c[i][1]
        AA[cx-1][cy]=0
        AA[cx+1][cy]=0
        AA[cx][cy-1]=0
        AA[cx][cy+1]=0
        A[cx-1][cy]=255
        A[cx+1][cy]=255
        A[cx][cy-1]=255
        A[cx][cy+1]=255

        
        AA[cx-2][cy]=0
        AA[cx+2][cy]=0
        AA[cx][cy-2]=0
        AA[cx][cy+2]=0
        AA[cx-1][cy-1]=0
        AA[cx-1][cy+1]=0
        AA[cx+1][cy-1]=0
        AA[cx+1][cy+1]=0
        A[cx-2][cy]=255
        A[cx+2][cy]=255
        A[cx][cy-2]=255
        A[cx][cy+2]=255
        A[cx-1][cy-1]=255
        A[cx-1][cy+1]=255
        A[cx+1][cy-1]=255
        A[cx+1][cy+1]=255

    Carte_nav_dym=Carte_explo
    Carte_nav_dym[:,:,0]=A
    Carte_nav_dym[:,:,1]=AA
    Carte_nav_dym[:,:,2]=A
                 
    return Carte_nav_dym


    
def pseudo_main():
    #N=1,W=2,S=3,E=4
    direction =1
    Carte=[]
    suppr=[]
    Carte_explo=np.zeros([100,100,3])
    Carte_decision=np.zeros([100,100,2])
    Carte_nav_dym=np.zeros([100,100,3])
    position = np.array([50,50])
    position_depart = np.array([50,50])
    x = position[0]
    y = position[1]
    # on peut changer les valeurs position,position_depart et la taille de Carte_explo comme l'on souhaite
    #
    # il faut definir la fonction scout (partie vision)
    #
    Carte_explo,Carte_decision=scout(x,y,direction,Carte_explo,Carte,Carte_decision)
    Carte_nav_dym = update_carte_nav(Carte_explo)
    # voir la fonction scout pour savoir ce qui est important
    # pour afficher : plt.imshow(img) -et diviser img par 255-
    STOP=0
    GGG=0
    while STOP==0:
        pass_X,pass_Y=pathss(x,y,Carte_explo,Carte_nav_dym,Carte_decision,Carte,position,position_depart,suppr)
        autorise = 1
        GGG=GGG+1
        for i in range(0,len(pass_X)-1):
            direction = detect_direction(x,y,pass_X(i+1),pass_Y(i+1))
            Carte_explo,Carte_decision=scout(x,y,direction,Carte_explo,Carte,Carte_decision)                    
            Carte_nav_dym = update_carte_nav(Carte_explo)
            autorise = autorrize(pass_X(i+1),pass_Y(i+1),Carte_nav_dym)
            if autorise ==0:
                position = np.array([x,y])
                position_depart=(GGG*position_depart+position)/float(GGG+1)
                break
            x=pass_X(i+1)
            y=pass_Y(i+1)
            #cette partie corespond à l'attente du déplacement jusqu'au
            #prochain point
            #deplacement = 1
            #while deplacement==1 and STOP==0:
            #    time.sleep(1)
            #    deplacement = check_deplacement()
            Carte_explo,Carte_decision=scout(x,y,direction,Carte_explo,Carte,Carte_decision)
        position = np.array([x,y])
        position_depart=(GGG*position_depart+position)/float(GGG+1)
        Carte_decision=envi(Carte_decision)
        # rajouter une variable pour arreter le programme
        STOP = check_STOP()

def lissage_pathss(pass_X,pass_Y):
    new_pass_X=np.zeros([len(pass_X)])
    new_pass_Y=np.zeros([len(pass_X)])

    if len(pass_X)<3:
        return pass_X,pass_Y
    else:
        for i in range(2,len(pass_X)):
            a,b,c=bezzier(pass_X[i-2:i+1],pass_Y[i-2:i+1])
            new_pass_X[i-2]+=a[0]
            new_pass_X[i-1]+=b[0]
            new_pass_X[i]+=c[0]
            new_pass_Y[i-2]+=a[1]
            new_pass_Y[i-1]+=b[1]
            new_pass_Y[i]+=c[1]
        new_pass_X[1]=new_pass_X[1]/2
        new_pass_X[len(new_pass_X)-2]=new_pass_X[len(new_pass_X)-2]/2
        new_pass_Y[1]=new_pass_Y[1]/2
        new_pass_Y[len(new_pass_Y)-2]=new_pass_Y[len(new_pass_Y)-2]/2
        new_pass_X[2:len(new_pass_X)-2]=new_pass_X[2:len(new_pass_X)-2]/3
        new_pass_Y[2:len(new_pass_Y)-2]=new_pass_Y[2:len(new_pass_Y)-2]/3
        return new_pass_X,new_pass_Y

def bezzier(pass_Xt,pass_Yt):
    X=np.zeros([101])
    Y=np.zeros([101])
    for i in range(0,101):
        X[i]=(1-0.01*i)**2*pass_Xt[0]+2*(1-0.01*i)*0.01*i*pass_Xt[1]+(0.01*i)**2*pass_Xt[2]
        Y[i]=(1-0.01*i)**2*pass_Yt[0]+2*(1-0.01*i)*0.01*i*pass_Yt[1]+(0.01*i)**2*pass_Yt[2]
    a=[X[0],Y[0]]
    b=[X[50],Y[50]]
    c=[X[100],Y[100]]
    return a,b,c
    
def scout(x,y,direction,deviation_position,deviation_angle,Carte_explo,Carte,Carte_decision):
    print('mdr')
    # la fonction scout doit return Carte_explo et Carte_decision
    # voir au minimum Carte_explo
    # le but de cette fonction est de rajouter obstacle/zone libre etc...
    # sur la carte
    # elle necessite donc x y (la position actuelle) ainsi que la direction
    # actuelle du robot pour savoir ou rajouter les nouvelles informations
    # il lui faut donc carte_explo et carte_decision pour pouvoir les updates.
    # ici 'Carte' est le vecteur local de ce que le robot voit.
    
    
def check_STOP():
    # verifie une variable d'arrêt
    # renvoie 0 si on continue
    # renvoie 1 si on s'arrête
    print('stop')













    
    
