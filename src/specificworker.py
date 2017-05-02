#
# Copyright (C) 2017 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys, os, traceback, time

from PySide import *
from genericworker import *
from math import *
import numpy as np

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 10
        self.timer.start(self.Period)
        self.x=0.0
        self.y=0.0
        self.l1=1
        self.l2=1

    def setParams(self, params):
        self.elbow = 'arm{0}motor2'.format(params['leg'])
        self.wrist = 'arm{0}motor3'.format(params['leg'])
        self.max_speed = int(params['max_speed'])
        #try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        #except:
        #	traceback.print_exc()
        #	print "Error reading config params"
        print 'Iniciado con exito'
        return True
        
    @QtCore.Slot()
    def compute(self):
        alpha= self.jointmotor_proxy.getMotorState(self.elbow).pos
        beta= self.jointmotor_proxy.getMotorState(self.wrist).pos
        if abs(alpha) <= pi /2 and abs(beta) <= pi /2:
            vAngle = self.invJacobian([alpha, beta], [self.l1, self.l2])*np.array([[self.x] ,[self.y]])
            print vAngle[0].item()
            print vAngle[1].item()
            g0 = MotorGoalVelocity()
            g0.name = self.elbow
            g0.velocity = vAngle[0].item()

            g1 = MotorGoalVelocity()
            g1.name = self.wrist
            g1.velocity = vAngle[1].item()
            
            self.jointmotor_proxy.setVelocity(g0) 
            self.jointmotor_proxy.setVelocity(g1) 
        
        
#  if self.x != 0.0 or self.y != 0.0 :
#     alphaOld = self.jointmotor_proxy.getMotorState(self.elbow).pos
#      betaOld = self.jointmotor_proxy.getMotorState(self.wrist).pos
#      xOld, yOld = self.dk(alphaOld, betaOld)
#      alpha, beta = self.ik(self.x+xOld , self.y+yOld)
    
#      g1 = MotorGoalPosition()
#      g1.name=self.elbow
#      g1.position=alpha
#      g1.maxSpeed=self.max_speed
    
#      g2 = MotorGoalPosition()
#      g2.name=self.wrist
#      g2.position=beta
#      g2.maxSpeed=self.max_speed
    
#     self.jointmotor_proxy.setPosition(g1)
#      self.jointmotor_proxy.setPosition(g2)

        return True


    #
    # sendData
    #
    def sendData(self, data):
        self.x = float(data.axes[0].value)
        self.y = float(data.axes[1].value)
        
    def dk(self, alpha, beta):
        l1 = 1
        l2 = 1
        x = l1 * cos(alpha) + l2 * cos(alpha + beta)
        y = l1 * sin(alpha) + l2 * sin(alpha + beta)
        return x, y

    def ik(self, x, y):
        try:
            r2 = pow(x, 2) + pow(y, 2)
            r = sqrt(r2)
            if r <= self.l1 + self.l2:
                numerador = -r2 + pow(self.l1, 2) + pow(self.l2, 2)
                cosBetaP = numerador / (2 * self.l1 * self.l2)
                radicando = 1 - pow(cosBetaP, 2)
                senoBetaP = sqrt(radicando)
                betaP = 0.0
                if cosBetaP != 0:
                    betaP = atan(senoBetaP / cosBetaP)
                else:
                    betaP = asin(senoBetaP)

                senoAlphaP = self.l2 * senoBetaP / r
                radicando = 1 - pow(senoAlphaP, 2)
                cosAlphaP = sqrt(radicando)
                alphaP = 0.0
                if cosAlphaP != 0:
                    alphaP = atan(senoAlphaP / cosAlphaP)
                else:
                    alpha = asin(senoAlphaP)

                gama = 0.0
                # if r2 != 0: TODO
                #     gama = asin(y / sqrt(r2))
                if x > 0:
                    gama = atan(y / x)
                elif x == 0:
                    gama = asin(y / r)
                elif y >= 0:
                    gama = (pi / 2) - atan(y / x)
                else:
                    gama = -pi + atan(y / x)

                alpha = gama + alphaP
                beta = 0.0
                # TODO esta parte es problematica: cuando betaP deberia ser 0 pero como hay cierta imprecision no lo es
                if betaP <= 0:
                    beta = betaP
                else:
                    beta = betaP - pi

                return alpha, beta
            return 0.0, 0.0
        except Exception, e:
            traceback.print_exc()
            print e
            print '+x={0}'.format(x)
            print '+y={0}'.format(y)
            print '+r={0}'.format(r)
            return 0.0, 0.0 
            
    def invJacobian(self, A, L):
        dHxdA1 = -L[0]*sin(A[0]) - L[1]*sin(A[0]+A[1])
        dHxdA2 = -L[1]*sin(A[0]+A[1])
        dHydA1 = L[0]*cos(A[0]) + L[1]*cos(A[0]+A[1])
        dHydA2 = L[1]*cos(A[0]+A[1])
        J = np.matrix([[dHxdA1,dHxdA2],[dHydA1,dHydA2]])
        iJ= np.linalg.inv(J)
        return iJ
        
        
