import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import random
from scipy.spatial import Delaunay
import pandas as pd
import math


def in_hull(p, hull):
    """
    Test if points in `p` are in `hull`

    `p` should be a `NxK` coordinates of `N` points in `K` dimensions
    `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the 
    coordinates of `M` points in `K`dimensions for which Delaunay triangulation
    will be computed
    """
    if not isinstance(hull,Delaunay):
        hull = Delaunay(hull)

    return hull.find_simplex(p)>=0

def my_filter(scan, finestra_media_mobile):

    ranges=list(scan.ranges)
    max_range=25

    #print(len(ranges))

     #put 0 to out_of_range data
    for i in range(len(ranges)):
        if(ranges[i]>max_range):
            ranges[i]=0
  
    #rotate array so the first value is a zero
    n=0
    while((ranges[:finestra_media_mobile]!=[0 for _ in range(finestra_media_mobile)]) and n<len(ranges)):
        temp=ranges[0]
        ranges=ranges[1:]
        ranges.append(temp)
        n=n+1
        
    # calculate moving average
    rolling_mean = pd.Series(ranges).rolling(finestra_media_mobile).mean()

    #make cluster checking using moving average>0 o no for a point
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
    
    for i in range(len(clusters)):
        all_zeros=True
        for el in clusters[i]:
            if el!=0:
                all_zeros=False
                break
        
        #move zeros at borders from clusters with data to cluster of zeros 
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
    
    n_clusters = 0
    for i in range(len(clusters))[1:]:
        all_zeros=True
        for el in clusters[i]:
            if el!=0:
                all_zeros=False
                break

        #filter out single-point clusters
        if(all_zeros==False):
            n_clusters=n_clusters+1
            if len(clusters[i])==1:
                clusters[i][0]=0
            
    #print(n_clusters)
        
    #transform angle,range data into cartesian (x,y) data
    angles = np.linspace(0, 2*np.pi, len(ranges))
    x = np.array(ranges)   * np.cos(angles)
    y = np.array(ranges)   * np.sin(angles)
  
    i=0
    for k in range(len(clusters)):
        all_zeros=True
        for el in clusters[k]:
            if el!=0:
                all_zeros=False
                break
        
        #pulisco gli zeri dai cluster buoni
        if(all_zeros==False and len(clusters[k])>2):
            point_list=[]
            for j in range(i,i+len(clusters[k])):
                if(x[j]!=0 and y[j]!=0):
                    point_list.append([x[j],y[j]])
            
            if(len(point_list)>2):
                #upsample data inside poligon of clusters
                for j in range(i,i+len(clusters[k])):
                    if(x[j]==0 and y[j]==0):
                        media = min([_ for _ in clusters[k] if _ != 0])
                        std_dev = 0.5
                        #r_rand=min([_ for _ in clusters[k] if _ != 0]) + abs(min([_ for _ in clusters[k] if _ != 0])-np.random.normal(media,std_dev))
                        r_rand=np.random.normal(min([_ for _ in clusters[k] if _ != 0]),max(clusters[k]))
                        theta=angles[j]
                        x_rand = r_rand * np.cos(theta)
                        y_rand = r_rand * np.sin(theta)
                        point = (x_rand,y_rand)

                        n_tries=0
                        while(( not in_hull(point,point_list) ) and n_tries<20) :
                            #r_rand=min([_ for _ in clusters[k] if _ != 0]) + abs(min([_ for _ in clusters[k] if _ != 0])-np.random.normal(media,std_dev))
                            r_rand=random.uniform(min([_ for _ in clusters[k] if _ != 0]),max(clusters[k]))
                            theta=angles[j]
                            x_rand = r_rand * np.cos(theta)
                            y_rand = r_rand * np.sin(theta)
                            point = (x_rand,y_rand)
                            n_tries=n_tries+1
                            

                        x[j]=x_rand
                        y[j]=y_rand
                        clusters[k][j-i]=r_rand

        i=i+len(clusters[k])

    #reconstruct range data from Cartesian (x,y) data
    for i in range(len(ranges)):
        ranges[i]= (x[i]**2+y[i]**2)**(0.5)
    x = np.array(ranges) * np.cos(angles)
    y = np.array(ranges) * np.sin(angles)

    #re order clusters
    while(n>0):
        temp=ranges[-1]
        ranges=ranges[:-1]
        ranges.insert(0,temp)
        n=n-1

    #print(ranges)
    return ranges



class Filter:
    def __init__(self):
        self.scan_sub = rospy.Subscriber("scan_multi", LaserScan, self.scan_callback)
        self.scan_pub = rospy.Publisher("scan_filtered", LaserScan, queue_size=1)

    def scan_callback(self, scan):
        range = np.array(scan.ranges)
        scan.ranges = my_filter( scan, 7 )
        self.scan_pub.publish(scan)

if __name__ == "__main__":
    rospy.init_node("filter_and_upscale")
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