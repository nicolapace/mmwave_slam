import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt 
from sklearn.neighbors import KernelDensity
from scipy.signal import argrelextrema
import pandas as pd
import math


def media_non_zeri(arr):
    i=0
    for _ in arr:
        if _!=0:
            i=i+1
    if i!=0:
        return sum(arr)/i
    else:
        return arr[0]

def my_filter(scan, finestra_media_mobile):

    ranges=list(scan.ranges)
    max_range=25

    print(len(ranges))

      #put 0 to out_of_range data
    for i in range(len(ranges)):
        if(ranges[i]>max_range):
            ranges[i]=0

    #print(ranges)
    
    #rotate array so the first value is a zero
    n=0
    while((ranges[:finestra_media_mobile]!=[0 for _ in range(finestra_media_mobile)]) and n<len(ranges)):
        temp=ranges[0]
        ranges=ranges[1:]
        ranges.append(temp)
        n=n+1

    #print(ranges)

    t=np.linspace(0,len(ranges))
        
    # Calcoliamo la media mobile su una finestra di 3 elementi
    rolling_mean = pd.Series(ranges).rolling(finestra_media_mobile).mean()

    #faccio i cluster dividendo in base a media mobile >0 o no
    clusters=[[ranges[0]]]
    j=0
    rolling_mean[0]=ranges[0]
    for i in range(len(ranges))[1:]:
        if(math.isnan(rolling_mean[i])):
            rolling_mean[i]=ranges[i]

        if(rolling_mean[i-1]==0 and rolling_mean[i]!=0 ):
            if(clusters[j]!=[]):
                j=j+1
                clusters.append([])
            clusters[j].append(ranges[i])
        elif(rolling_mean[i-1]!=0 and rolling_mean[i]==0 ):
            clusters[j].append(ranges[i])
            j=j+1
            clusters.append([])
        else:
            clusters[j].append(ranges[i])
            
    #print(clusters)

    
    for i in range(len(clusters)):
        all_zeros=True
        for el in clusters[i]:
            if el!=0:
                all_zeros=False
                break
        
        #raggruppo gli zeri nei cluster
        if(all_zeros==True):
            if(i>0):
                k=0
                while(clusters[i-1][len(clusters[i-1])-k-1]==0):
                    clusters[i-1].pop(len(clusters[i-1])-k-1)
                    clusters[i].append(0)
            if(i!=len(clusters)-1):
                k=0
                while(clusters[i+1][k]==0):
                    clusters[i+1].pop(k)
                    clusters[i].append(0)
    
    #print(clusters)

    n_clusters = 0
    for i in range(len(clusters))[1:]:
        all_zeros=True
        for el in clusters[i]:
            if el!=0:
                all_zeros=False
                break
        
        #pulisco gli zeri dai cluster buoni
        if(all_zeros==False):
            n_clusters=n_clusters+1
            
            for j in range(len(clusters[i]))[1:]:
                if(clusters[i][j]==0):
                    finestra_media_mobile_opt=finestra_media_mobile
                    if(finestra_media_mobile%2!=0):
                        finestra_media_mobile_opt=finestra_media_mobile+1

                    if(j<=finestra_media_mobile/2):
                        blocco=clusters[i][:finestra_media_mobile]
                    elif(j<=(len(clusters[i])-finestra_media_mobile/2)):
                        blocco=clusters[i][-finestra_media_mobile:]
                    else:
                        blocco=clusters[i][-int(finestra_media_mobile/2):int(j+finestra_media_mobile/2)]

                    clusters[i][j]=media_non_zeri(blocco)


    print(n_clusters)



    ## elaborare cluster:
    ## calcolare la deviazione standard della me


    #ricostruisco ranges dai cluster
    new_ranges=[]
    for cluster in clusters:
        for el in cluster:
            new_ranges.append(el)
    ranges=new_ranges

    while(n!=0):
        temp=ranges[-1]
        ranges=ranges[:-1]
        ranges.insert(0,temp)
        n=n-1

    print(ranges)

    return ranges


class Filter:
    def __init__(self):
        self.scan_sub = rospy.Subscriber("scan_multi", LaserScan, self.scan_callback)
        self.scan_pub = rospy.Publisher("scan_filtered", LaserScan, queue_size=1)

    def scan_callback(self, scan):
        range = np.array(scan.ranges)
        scan.ranges = my_filter( scan, 400 )
        self.scan_pub.publish(scan)

if __name__ == "__main__":
    rospy.init_node("kmeans_filter")
    filter = Filter()
    rospy.spin()
'''0

angle_min: -2.359999895095825
angle_max: 2.359999895095825
angle_increment: 0.005799999926239252
time_increment: 0.0
scan_time: 0.03333333134651184
range_min: 0.44999998807907104
range_max: 25.0
ranges: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9272570013999939, 0.8962552547454834, 0.8962552547454834, 0.8652535080909729, 0.822822630405426, 0.8226433992385864, 0.9070420265197754, 0.8233153820037842, 0.8226433992385864, 0.8226433992385864, 0.8219714164733887, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.067842721939087, 0.0, 0.0, 0.0, 0.0, 0.0, 2.072988986968994, 1.946545124053955, 1.946545124053955, 1.820101261138916, 2.0814809799194336, 1.789838433265686, 2.0793545246124268, 1.789838433265686, 1.789838433265686, 1.7843059301376343, 1.789838433265686, 1.789838433265686, 1.789838433265686, 1.789838433265686, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.7724519968032837, 0.0, 0.0, 0.0, 0.0, 0.0, 1.765303134918213, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8048887848854065, 0.8048429489135742, 0.8048429489135742, 0.8047970533370972, 0.9680715203285217, 0.9680715203285217, 0.9078623056411743, 0.9680715203285217, 0.9680715203285217, 0.9680715203285217, 0.9489616751670837, 0.9680715203285217, 0.9680715203285217, 0.9685670137405396, 0.9680715203285217, 0.9680715203285217, 0.9675760269165039, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
'''