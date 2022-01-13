import numpy as np
from scipy.special import binom
import matplotlib.pyplot as plt
import math


bernstein = lambda n, k, t: binom(n,k)* t**k * (1.-t)**(n-k)

def bezier(points, num=200):
    N = len(points)
    t = np.linspace(0, 1, num=100)
    curve = np.zeros((num, 2))
    for i in range(N):
        curve += np.outer(bernstein(N - 1, i, t), points[i])
    return curve

class Segment():
    def __init__(self, p1, p2, angle1, angle2, **kw):
        self.p1 = p1; self.p2 = p2
        self.angle1 = angle1; self.angle2 = angle2
        self.numpoints = kw.get("numpoints", 100)
        r = kw.get("r", 0.3)
        d = np.sqrt(np.sum((self.p2-self.p1)**2))
        self.r = r*d
        self.p = np.zeros((4,2))
        self.p[0,:] = self.p1[:]
        self.p[3,:] = self.p2[:]
        self.calc_intermediate_points(self.r)

    def calc_intermediate_points(self,r):
        self.p[1,:] = self.p1 + np.array([self.r*np.cos(self.angle1),
                                    self.r*np.sin(self.angle1)])
        self.p[2,:] = self.p2 + np.array([self.r*np.cos(self.angle2+np.pi),
                                    self.r*np.sin(self.angle2+np.pi)])
        self.curve = bezier(self.p,self.numpoints)


def get_curve(points, **kw):
    segments = []
    for i in range(len(points)-1):
        seg = Segment(points[i,:2], points[i+1,:2], points[i,2],points[i+1,2],**kw)
        segments.append(seg)
    curve = np.concatenate([s.curve for s in segments])
    return segments, curve

def ccw_sort(p):
    d = p-np.mean(p,axis=0)
    s = np.arctan2(d[:,0], d[:,1])
    return p[np.argsort(s),:]

def get_bezier_curve(a, rad=0.2, edgy=0):
    """ given an array of points *a*, create a curve through
    those points. 
    *rad* is a number between 0 and 1 to steer the distance of
          control points.
    *edgy* is a parameter which controls how "edgy" the curve is,
           edgy=0 is smoothest."""
    p = np.arctan(edgy)/np.pi+.5
    a = ccw_sort(a)
    a = np.append(a, np.atleast_2d(a[0,:]), axis=0)
    d = np.diff(a, axis=0)
    ang = np.arctan2(d[:,1],d[:,0])
    f = lambda ang : (ang>=0)*ang + (ang<0)*(ang+2*np.pi)
    ang = f(ang)
    ang1 = ang
    ang2 = np.roll(ang,1)
    ang = p*ang1 + (1-p)*ang2 + (np.abs(ang2-ang1) > np.pi )*np.pi
    ang = np.append(ang, [ang[0]])
    a = np.append(a, np.atleast_2d(ang).T, axis=1)
    s, c = get_curve(a, r=rad, method="var")
    x,y = c.T
    return x,y, a

def get_random_points(n=5, scale=0.8, mindst=None, rec=0):
    """ create n random points in the unit square, which are *mindst*
    apart, then scale them."""
    mindst = mindst or .7/n
    a = np.random.rand(n,2)
    d = np.sqrt(np.sum(np.diff(ccw_sort(a), axis=0), axis=1)**2)
    if np.all(d >= mindst) or rec>=200:
        return a*scale
    else:
        return get_random_points(n=n, scale=scale, mindst=mindst, rec=rec+1)
        
fig, ax = plt.subplots()
ax.set_aspect("equal")

rad = 0.5
edgy = 0.999
vertices = 6

a = get_random_points(n=vertices, scale=200)
x,y, _ = get_bezier_curve(a,rad=rad, edgy=edgy)

#currently we have that small edges have the same number of points of large edges -> very dense points
#-> remove all points in a way such that we
threshold = 2.5
old = 0
tot_dist = 0
to_remove = []
to_remove = np.array(to_remove)

for i in range(1, len(x)):
  dist = np.sqrt((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2)
  #print("dist between", i-1 , "and ", i, " = ", dist)
  tot_dist = tot_dist + dist
  if(tot_dist >= threshold):
    if(i - old > 1):
      rang = np.arange(start = old + 1, stop = i)
      #print("removing range ", rang)
      to_remove = np.concatenate((to_remove, rang), axis=0)
    old = i
    tot_dist = 0
    
to_remove = to_remove.astype(int)
print("to remove: ", len(to_remove), " elements")
print("original number of points: ", len(x))

################################################################################
#calculates left and right cones.
x = np.delete(x, to_remove, 0)
y = np.delete(y, to_remove, 0)

Rx = np.zeros(len(x))
Ry = np.zeros(len(x))
Lx = np.zeros(len(x))
Ly = np.zeros(len(x))

dist = 1.5

#x = [0, 0, 0, 0, 1, 2, 3]
#y = [3, 2, 1, 0, 0, 0, 0]
#x = [1, 3, 2, 1, 1]
#y = [1, 2, 3, 3, 2]
#dist = 0.1

def insidePoly(Dx, Dy, pX, pY):
  inside = False;
  
  i = 0
  j = len(Dx) - 1
  
  while (i < len(Dx)):
    if(((Dy[i] > pY) != (Dy[j] > pY)) and
        (pX < (Dx[j]-Dx[i])*(pY-Dy[i])/(Dy[j]-Dy[i]) + Dx[i])):
      inside = not inside;
    j = i
    i = i + 1
      
  return inside;

spawn_yaw = 0

for i in range(1, len(x) - 1):
  #coordinates of the points behind and ahead of x[i], translated such that x[i]
  #is at coordinates (0,0)
  A_x = x[i-1] - x[i]
  A_y = y[i-1] - y[i]
  C_x = x[i+1] - x[i]
  C_y = y[i+1] - y[i]
  
  #angle of A and C against the x axis
  alpha_a = (math.degrees(math.atan2(A_y,A_x)) + 360) % 360
  alpha_c = (math.degrees(math.atan2(C_y,C_x)) + 360) % 360
  
  #angle of bisector. if the angle is 
  angle = (alpha_a + alpha_c)/2
  
  #print("i = ", i, "; a = ", alpha_a, ", c = ", alpha_c, ", angle = ", angle)
  
  #coordinates of the bisector
  D_x = np.cos(np.deg2rad(angle))
  D_y = np.sin(np.deg2rad(angle))
  
  #print("i = ", i, "; D_x = ", D_x, ", D_y = ", D_y)
  
  if(np.abs(D_x) < .0000001):
    print("i = ", i)
    L_x = x[i]
    L_y = y[i] + dist
    
    R_x = x[i]
    R_y = y[i] - dist
  else:
    m = D_y/D_x
    #print("i = ", i, ", m = ", m)
    
    L_x = x[i] + np.sqrt((dist**2)/(1 + m**2))
    L_y = y[i] + m*np.sqrt((dist**2)/(1 + m**2)) 
    
    R_x = x[i] - np.sqrt((dist**2)/(1 + m**2))
    R_y = y[i] - m*np.sqrt((dist**2)/(1 + m**2))
    
  if(insidePoly(x, y, R_x, R_y)):
    Rx[i] = R_x
    Ry[i] = R_y
    
    Lx[i] = L_x
    Ly[i] = L_y
  else:
    Rx[i] = L_x
    Ry[i] = L_y
    
    Lx[i] = R_x
    Ly[i] = R_y

for i in range(0, len(Rx) - 1):
  if(Rx[i] == 0 and Ry[i] == 0 and Lx[i] == 0 and Ly[i] == 0):
    Rx = np.delete(Rx, [i], 0)
    Ry = np.delete(Ry, [i], 0)
    Lx = np.delete(Lx, [i], 0)
    Ly = np.delete(Ly, [i], 0)
    
    x = np.delete(x, [i], 0)
    y = np.delete(y, [i], 0)

for i in range(1, len(Rx)):
  Rx[i] = Rx[i] - x[0]
  Ry[i] = Ry[i] - y[0]
  Lx[i] = Lx[i] - x[0]
  Ly[i] = Ly[i] - y[0]
  x[i]  = x[i] - x[0]
  y[i]  = y[i] - y[0]
  
Rx[0] = Rx[0] - x[0]
Ry[0] = Ry[0] - y[0]
Lx[0] = Lx[0] - x[0]
Ly[0] = Ly[0] - y[0]
y[0] = 0
x[0] = 0

#calculates the spawn angle:
spawn_yaw = (math.degrees(math.atan2(y[1],x[1])) + 360) % 360

plt.plot(x,y, '.')
plt.plot(Rx, Ry, '.')
plt.plot(Lx, Ly, '.')
plt.show()
plt.figure
################################################################################

with open('test.yaml','w') as f:
  f.write("cones:\n") 
  
  f.write("- position:\n")
  f.write("  - " + Rx[0].astype(str) + "\n")
  f.write("  - " + Ry[0].astype(str) + "\n")
  f.write("  - 0\n")
  f.write("  type: big orange\n")
    
  f.write("- position:\n")
  f.write("  - " + Lx[0].astype(str) + "\n")
  f.write("  - " + Ly[0].astype(str) + "\n")
  f.write("  - 0\n")
  f.write("  type: big orange\n")
  
  for i in range(1, len(Rx)):
    f.write("- position:\n")
    f.write("  - " + Rx[i].astype(str) + "\n")
    f.write("  - " + Ry[i].astype(str) + "\n")
    f.write("  - 0\n")
    f.write("  type: small yellow\n")
    
    f.write("- position:\n")
    f.write("  - " + Lx[i].astype(str) + "\n")
    f.write("  - " + Ly[i].astype(str) + "\n")
    f.write("  - 0\n")
    f.write("  type: small blue \n")
  
  #f.write("description: \"" + str(x[1]) + ", " + str(y[1]) + ", 1.0, 0.0, 0.0, " + str(spawn_yaw) + "\"\n")
  f.write("description: \"0.0, 0.0, 1.0, 0.0, 0.0, " + str(spawn_yaw) + "\"\n")
  f.write("init_offset: 2\n")
  f.write("name: test")
