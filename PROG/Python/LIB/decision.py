print('cmd_decision loaded')

''' vérifier (notemment sur scout, que les orientations matrice sont tjs bon)'''
from math import sqrt
'''from numpy import *'''
import numpy as np
import scipy

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
    BBB=scipy.ndimage.filters.convolve(Carte_decision[:,:,0],h)
    BBB[BBB==1]=0
    Carte_decision[:,:,1]=BBB
    return Carte_decision    
    

'''function [Carte_explo,Carte_decision]=scout(x,y,direction,Carte_explo,Carte,Carte_decision)'''
def scout(x,y,direction,Carte_explo,Carte,Carte_decision):
    K=1
    ligne = np.size(Carte_explo,0)
    colonne = np.size(Carte_explo,1)
    
    if direction ==1:
        Carte_explo[y-2:y-1,x-1:x+1,:]=Carte[y-2:y-1,x-1:x+1,:]
        Carte_decision[y-2:y-1,x-1:x+1,1]=np.ones([2,3],int)*K

    if direction ==2:
        Carte_explo[y-1:y+1,x-2:x-1,:]=Carte[y-1:y+1,x-2:x-1,:]
        Carte_decision[y-1:y+1,x-2:x-1,1]=np.ones([3,2],int)*K

    if direction ==3:
        Carte_explo[y+1:y+2,x-1:x+1,:]=Carte[y+1:y+2,x-1:x+1,:]
        Carte_decision[y+1:y+2,x-1:x+1,1]=np.ones([2,3],int)*K

    if direction ==4:
        Carte_explo[y-1:y+1,x+1:x+2,:]=Carte[y-1:y+1,x+1:x+2,:]
        Carte_decision[y-1:y+1,x+1:x+2,1]=ones([3,2],int)*K
