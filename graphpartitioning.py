# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import csv
import pymetis
import numpy as np
import time
from matplotlib import pyplot as plt

def read_txt(filename):
    lv=[]
    with open(filename) as file:
        l=file.readlines()
    l2=l[0].replace('\n', '')
    l3=list(filter(None,l2.split(" ")))
    nbv,nbe=l3
    
    for line in l[1:]:
        line2=line.replace('\n', '')
        l4=list(filter(None,line2.split(" ")))
        i,j,w=l4
        lv.append((i,j,w))
    return lv,nbv,nbe
        
def graph_partitioning(proc,lv,nbv,nbe):
    #adjacency: 1*n matrix, where each row i represents the adjacent nodes of node i
    #adjncy:list of adjacent nodes (moves with xadj)
    #xadj: index of the adjacent nodes: adjacent nodes of node i from index xadj[i] to index xadj[i+1]
    timestart=time.time()
    l=[]
    e=[]
    xadj=[]
    idx=[]
    xcount=0
    for k in range(len(lv)):
        idx.append(int(lv[k][0]))
    for i in range(int(nbv)):
        n=idx.count(i)
        xcount+=n
        xadj.append(xcount)
    dtype=[('v1',int),('v2',int),('w',int)]
    a=np.array(lv,dtype)
    lv2=np.sort(a,order='v1')
    for k in range(len(lv)):
        i,j,w=lv2[k]
        l.append(j)
        e.append(w)
    arg={'nparts': proc,'xadj': xadj,'adjncy':l,'eweights': e,}
    r,part=pymetis.part_graph(**arg)
    f=open("data.txt",'w') 
    for i in part:
        f.write(str(i)+"\n")
    timeend=time.time()
    return abs(timestart-timeend)

lv,nbv,nbe=read_txt("64v1024e.txt")