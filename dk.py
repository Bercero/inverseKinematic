#! /usr/bin/python
from math import *
from ik import ik

l1 = 1
l2 = 1
#alpha y beta tienen que ser float y beta < 0
# para obtener lo mismo en el ik
alpha = pi/4
beta = -pi/2
x = l1 * cos(alpha) + l2 * cos(alpha + beta)
y = l1 * sin(alpha) + l2 * sin(alpha + beta)
print "cinematica directa:"
print "alpha, beta"
print (alpha, beta)
print "x,y"
print (x, y)
print "cinematica inversa:"
print "alpha, beta"
ik(x, y)
