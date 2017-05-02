#! /usr/bin/python
from sympy import *
a0, a1, a2, l0, l1, l2 = symbols('a0 a1 a2 l0 l1 l2')
j = l0+ l1 * cos(a1) + l2 * cos(a1+a2)
i = l1 * sin(a1) + l2 * sin(a1+a2)
x = j * cos(a0)
y = j * sin(a0)
z = i
print 'x = {0}'.format(x)
print 'y = {0}'.format(y)
print 'z = {0}'.format(z)

j00 = diff(x, a0)
j01 = diff(x, a1)
j02 = diff(x, a2)

j10 = diff(y, a0)
j11 = diff(y, a1)
j12 = diff(y, a2)

j20 = diff(z, a0)
j21 = diff(z, a1)
j22 = diff(z, a2)

print 'j00 = {0}'.format(j00)
print 'j01 = {0}'.format(j01)
print 'j02 = {0}'.format(j02)

print 'j10 = {0}'.format(j10)
print 'j11 = {0}'.format(j11)
print 'j12 = {0}'.format(j12)

print 'j20 = {0}'.format(j20)
print 'j21 = {0}'.format(j21)
print 'j22 = {0}'.format(j22)