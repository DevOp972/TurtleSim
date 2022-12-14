from math import *

from numpy import Infinity
def angle(pt1,pt2,pt3):
    mod=sqrt((pow(pt2[0]-pt1[0],2)+pow(pt2[1]-pt1[1],2))*(pow(pt3[0]-pt2[0],2)+pow(pt3[1]-pt2[1],2)))
    # print("points ",pt1,pt2,pt3)
    val=((pt2[0]-pt1[0])*(pt3[0]-pt2[0])+(pt2[1]-pt1[1])*(pt3[1]-pt2[1]))
    if(mod==0 or abs(val-mod)<0.00001):
      return 0
    
    return acos(((pt2[0]-pt1[0])*(pt3[0]-pt2[0])+(pt2[1]-pt1[1])*(pt3[1]-pt2[1]))/mod)
def dist(pt1,pt2):
    return sqrt(pow(pt1[0]-pt2[0],2)+pow(pt1[1]-pt2[1],2))

def scale_down(pt):
    pt=[round(pt[0]*11/400,1),round(pt[1]*11/400,1)]
    return str(pt[0])+" "+str(pt[1])+"\n"
def rev_list(pts):
   return pts.reverse()
       
def main():
    f = open("path2.txt", "r")
    fout=open("scaledpts2.txt","w")
    frev=open("scaledpts2_rev.txt","w")
    line=f.readline()
    pts=[]
    pt1=[float(i) for i in line.split()]
    # print(pt1)
    
    fout.write(scale_down(pt1))
    pts.append(scale_down(pt1))
    line=f.readline()
    pt2=[float(i) for i in line.split()] 
    fout.write(scale_down(pt2))
    pts.append(scale_down(pt2))
    while True:   
        line=f.readline() 
        if not line:
            break
        # print(pts)
        pt3 = [float(i) for i in line.split()]   
        if(degrees(angle(pt1,pt2,pt3))>5):
            print(dist(pt2,pt3))
            fout.write(scale_down(pt3)) 
            pts.append(scale_down(pt3))
        
          
        pt1=pt2
        pt2=pt3
    
              
    fout.write(scale_down(pt3)) 
    pts.append(scale_down(pt3))
    for i in reversed(pts):
        frev.write(i)
        
        
    fout.close()      
    frev.close() 
    f.close()
    
    
if __name__ == '__main__':
    main()
