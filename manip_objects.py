import numpy as np
from Geometry3D import ConvexPolygon, Vector, Renderer, intersection
from Geometry3D import Point as gPoint

sqrt3 = np.sqrt(3)

# Hexagonal prism:
##################
# a = 2 cm, h = 3 cm
h = 1.49*2

p1 = gPoint(-1,-sqrt3,-h/2)
p2 = gPoint(-1,-sqrt3,h/2)
p3 = gPoint(1,-sqrt3,h/2)
p4 = gPoint(1,-sqrt3,-h/2)
p5 = gPoint(2,0,h/2)
p6 = gPoint(2,0,-h/2)
p7 = gPoint(1,sqrt3,h/2)
p8 = gPoint(1,sqrt3,-h/2)
p9 = gPoint(-1,sqrt3,h/2)
p10 = gPoint(-1,sqrt3,-h/2)
p11 = gPoint(-2,0,h/2)
p12 = gPoint(-2,0,-h/2)
surf1 = ConvexPolygon((p1,p2,p3,p4))
surf2 = ConvexPolygon((p4,p3,p5,p6))
surf3 = ConvexPolygon((p6,p5,p7,p8))
surf4 = ConvexPolygon((p8,p7,p9,p10))
surf5 = ConvexPolygon((p10,p9,p11,p12))
surf6 = ConvexPolygon((p12,p11,p2,p1))
surf7 = ConvexPolygon((p2,p11,p9,p7,p5,p3))
surf8 = ConvexPolygon((p10,p12,p1,p4,p6,p8))
n1 = Vector(0,-1,0)
n2 = Vector(1.5,-sqrt3/2,0)
n3 = Vector(1.5,sqrt3/2,0)
n4 = Vector(0,1,0)
n5 = Vector(-1.5,sqrt3/2,0)
n6 = Vector(-1.5,-sqrt3/2,0)
n7 = Vector(0,0,1)
n8 = Vector(0,0,-1)

g1 = gPoint(-0.5,-1,-2)
g2 = gPoint(-0.5,-1,2)
g3 = gPoint(0.5,-1,2)
g4 = gPoint(0.5,-1,-2)

# goal1 = ConvexPolygon((g1,g2,g3,g4))

# g1 = gPoint(-0.5,-1,2)
# g2 = gPoint(-0.5,0,2)
# g3 = gPoint(0.5,0,2)
# g4 = gPoint(0.5,-1,2)

# goal5 = ConvexPolygon((g1,g2,g3,g4))

# g1 = gPoint(-0.5,-1,-2)
# g2 = gPoint(-0.5,0,-2)
# g3 = gPoint(0.5,0,-2)
# g4 = gPoint(0.5,-1,-2)

# goal6 = ConvexPolygon((g1,g2,g3,g4))

# g1 = gPoint(1.5,-1,-1)
# g2 = gPoint(1.5,-1,1)
# g3 = gPoint(1.5,1,1)
# g4 = gPoint(1.5,1,-1)

# goal2 = ConvexPolygon((g1,g2,g3,g4))

g1 = gPoint(-1,-sqrt3,h/2)
g2 = gPoint(-1,sqrt3,h/2)
g3 = gPoint(1,sqrt3,h/2)
g4 = gPoint(1,-sqrt3,h/2)

goal7 = ConvexPolygon((g1,g2,g3,g4))

g1 = gPoint(-1,-sqrt3,-h/2)
g2 = gPoint(-1,0,-h/2)
g3 = gPoint(1,0,-h/2)
g4 = gPoint(1,-sqrt3,-h/2)

goal8 = ConvexPolygon((g1,g2,g3,g4))

hexagonal_prism = {1:(surf1,n1),2:(surf2,n2),3:(surf3,n3),
            4:(surf4,n4),5:(surf5,n5),6:(surf6,n6),7:(surf7,n7,goal7),8:(surf8,n8,goal8)}
# r = Renderer()
# for surf in hexagonal_prism:
#     r.add((hexagonal_prism[surf][0],'b',1))
# r.add((gPoint(1.4226327944572748, 1.0, 1.49),'r',1))
# r.show()

# Rectangular prism:
##################
# w = 2 cm, l = 3 cm, h = 4 cm

p1 = gPoint(-1.5,-1,-2)
p2 = gPoint(-1.5,-1,2)
p3 = gPoint(1.5,-1,2)
p4 = gPoint(1.5,-1,-2)
p5 = gPoint(1.5,1,-2)
p6 = gPoint(1.5,1,2)
p7 = gPoint(-1.5,1,2)
p8 = gPoint(-1.5,1,-2)

surf1 = ConvexPolygon((p1,p2,p3,p4))
surf2 = ConvexPolygon((p4,p3,p6,p5))
surf3 = ConvexPolygon((p5,p6,p7,p8))
surf4 = ConvexPolygon((p8,p7,p2,p1))
surf5 = ConvexPolygon((p2,p7,p6,p3))
surf6 = ConvexPolygon((p8,p1,p4,p5))

n1 = Vector(0,-1,0)
n2 = Vector(1,0,0)
n3 = Vector(0,1,0)
n4 = Vector(-1,0,0)
n5 = Vector(0,0,1)
n6 = Vector(0,0,-1)

g1 = gPoint(-0.5,-1,-2)
g2 = gPoint(-0.5,-1,2)
g3 = gPoint(0.5,-1,2)
g4 = gPoint(0.5,-1,-2)

goal1 = ConvexPolygon((g1,g2,g3,g4))

g1 = gPoint(-0.5,-1,2)
g2 = gPoint(-0.5,0,2)
g3 = gPoint(0.5,0,2)
g4 = gPoint(0.5,-1,2)

goal5 = ConvexPolygon((g1,g2,g3,g4))

g1 = gPoint(-0.5,-1,-2)
g2 = gPoint(-0.5,0,-2)
g3 = gPoint(0.5,0,-2)
g4 = gPoint(0.5,-1,-2)

goal6 = ConvexPolygon((g1,g2,g3,g4))

g1 = gPoint(1.5,-1,-1)
g2 = gPoint(1.5,-1,1)
g3 = gPoint(1.5,1,1)
g4 = gPoint(1.5,1,-1)

goal2 = ConvexPolygon((g1,g2,g3,g4))

g1 = gPoint(-1.5,0,-1)
g2 = gPoint(-1.5,0,1)
g3 = gPoint(-1.5,-1,1)
g4 = gPoint(-1.5,-1,-1)

goal4 = ConvexPolygon((g1,g2,g3,g4))

# rectangular_prism = {1:(surf1,n1,goal1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5,goal5),6:(surf6,n6,goal6)}
rectangular_prism = {1:(surf1,n1,goal1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6)}

# Square prism:
##################
# a = 2.5 cm, h = 8 cm

p1 = gPoint(-1.25,-1.25,-4)
p2 = gPoint(-1.25,-1.25,4)
p3 = gPoint(1.25,-1.25,4)
p4 = gPoint(1.25,-1.25,-4)
p5 = gPoint(1.25,1.25,-4)
p6 = gPoint(1.25,1.25,4)
p7 = gPoint(-1.25,1.25,4)
p8 = gPoint(-1.25,1.25,-4)

surf1 = ConvexPolygon((p1,p2,p3,p4))
surf2 = ConvexPolygon((p4,p3,p6,p5))
surf3 = ConvexPolygon((p5,p6,p7,p8))
surf4 = ConvexPolygon((p8,p7,p2,p1))
surf5 = ConvexPolygon((p2,p7,p6,p3))
surf6 = ConvexPolygon((p8,p1,p4,p5))

n1 = Vector(0,-1,0)
n2 = Vector(1,0,0)
n3 = Vector(0,1,0)
n4 = Vector(-1,0,0)
n5 = Vector(0,0,1)
n6 = Vector(0,0,-1)

g1 = gPoint(-0.5,-1.25,-4)
g2 = gPoint(-0.5,-1.25,4)
g3 = gPoint(0.5,-1.25,4)
g4 = gPoint(0.5,-1.25,-4)

goal1 = ConvexPolygon((g1,g2,g3,g4))

g1 = gPoint(-0.5,-1.25,4)
g2 = gPoint(-0.5,0,4)
g3 = gPoint(0.5,0,4)
g4 = gPoint(0.5,-1.25,4)

goal5 = ConvexPolygon((g1,g2,g3,g4))

g1 = gPoint(-0.5,-1.25,-4)
g2 = gPoint(-0.5,0,-4)
g3 = gPoint(0.5,0,-4)
g4 = gPoint(0.5,-1.25,-4)

goal6 = ConvexPolygon((g1,g2,g3,g4))

g1 = gPoint(1.25,-1.25,-1)
g2 = gPoint(1.25,-1.25,1)
g3 = gPoint(1.25,1.25,1)
g4 = gPoint(1.25,1.25,-1)

goal2 = ConvexPolygon((g1,g2,g3,g4))

g1 = gPoint(-1.25,0,-1)
g2 = gPoint(-1.25,0,1)
g3 = gPoint(-1.25,-1.25,1)
g4 = gPoint(-1.25,-1.25,-1)

goal4 = ConvexPolygon((g1,g2,g3,g4))

square_prism = {1:(surf1,n1,goal1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5,goal5),6:(surf6,n6,goal6)}
# square = {1:(surf1,n1,goal1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6)}

# Rectangular prism tall with flanged corners:
##################
# w = 2 cm, l = 3 cm, h = 8 cm

p1 = gPoint(-1.5,-1,-4)
p2 = gPoint(-1.5,-1,4)
p3 = gPoint(1.5,-1,4)
p4 = gPoint(1.5,-1,-4)
p5 = gPoint(1.5,1,-4)
p6 = gPoint(1.5,1,4)
p7 = gPoint(-1.5,1,4)
p8 = gPoint(-1.5,1,-4)

surf1 = ConvexPolygon((p1,p2,p3,p4))
surf2 = ConvexPolygon((p4,p3,p6,p5))
surf3 = ConvexPolygon((p5,p6,p7,p8))
surf4 = ConvexPolygon((p8,p7,p2,p1))
surf5 = ConvexPolygon((p2,p7,p6,p3))
surf6 = ConvexPolygon((p8,p1,p4,p5))

n1 = Vector(0,-1,0)
n2 = Vector(1,0,0)
n3 = Vector(0,1,0)
n4 = Vector(-1,0,0)
n5 = Vector(0,0,1)
n6 = Vector(0,0,-1)

g1 = gPoint(-0.5,-1,-4)
g2 = gPoint(-0.5,-1,4)
g3 = gPoint(0.5,-1,4)
g4 = gPoint(0.5,-1,-4)

goal1 = ConvexPolygon((g1,g2,g3,g4))

g1 = gPoint(-0.5,-1,4)
g2 = gPoint(-0.5,0,4)
g3 = gPoint(0.5,0,4)
g4 = gPoint(0.5,-1,4)

goal5 = ConvexPolygon((g1,g2,g3,g4))

g1 = gPoint(-0.5,-1,-4)
g2 = gPoint(-0.5,0,-4)
g3 = gPoint(0.5,0,-4)
g4 = gPoint(0.5,-1,-4)

goal6 = ConvexPolygon((g1,g2,g3,g4))

g1 = gPoint(1.5,-1,-1)
g2 = gPoint(1.5,-1,1)
g3 = gPoint(1.5,1,1)
g4 = gPoint(1.5,1,-1)

goal2 = ConvexPolygon((g1,g2,g3,g4))

g1 = gPoint(-1.5,0,-1)
g2 = gPoint(-1.5,0,1)
g3 = gPoint(-1.5,-1,1)
g4 = gPoint(-1.5,-1,-1)

goal4 = ConvexPolygon((g1,g2,g3,g4))

rectangular_prism_tall_flanged = {1:(surf1,n1,goal1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5,goal5),6:(surf6,n6,goal6)}
# square = {1:(surf1,n1,goal1),2:(surf2,n2,goal2),3:(surf3,n3),4:(surf4,n4,goal4),5:(surf5,n5),6:(surf6,n6)}