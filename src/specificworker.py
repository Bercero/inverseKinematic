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
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.l0 = self.l1 = self.l2 = 0
        self.shoulder = self.elbow = self.wrist = 0
        self.max_speed = 0

    def setParams(self, params):
        self.shoulder = 'arm{0}motor1'.format(params['leg'])
        self.elbow = 'arm{0}motor2'.format(params['leg'])
        self.wrist = 'arm{0}motor3'.format(params['leg'])

        self.l0 = int(params['l0'])
        self.l1 = int(params['l1'])
        self.l2 = int(params['l2'])

        self.max_speed = int(params['max_speed'])

        print 'Iniciado con exito'
        return True
        
    @QtCore.Slot()
    def compute(self):
        a0= self.jointmotor_proxy.getMotorState(self.shoulder).pos
        a1= self.jointmotor_proxy.getMotorState(self.elbow).pos
        a2= self.jointmotor_proxy.getMotorState(self.wrist).pos

        # if abs(a0) <= pi /2 and abs(a1) <= pi /2 and abs(a2) <= pi /2:
        vAngle = self.invJacobian(a0, a1, a2)*np.array([[self.x], [self.y], [self.z]])
        # print vAngle[0].item()
        # print vAngle[1].item()
        # print vAngle[2].item()

        g0 = MotorGoalVelocity()
        g0.name = self.shoulder
        g0.velocity = vAngle[0].item()

        g1 = MotorGoalVelocity()
        g1.name = self.elbow
        g1.velocity = vAngle[1].item()

        g2 = MotorGoalVelocity()
        g2.name = self.wrist
        g2.velocity = vAngle[2].item()

        self.jointmotor_proxy.setVelocity(g0)
        self.jointmotor_proxy.setVelocity(g1)
        self.jointmotor_proxy.setVelocity(g2)

        return True

    # @QtCore.Slot()
    # def compute(self):
    #
    #     a1 = self.jointmotor_proxy.getMotorState(self.elbow).pos
    #     a2 = self.jointmotor_proxy.getMotorState(self.wrist).pos
    #
    #
    #     vAngle = self.OldinvJacobian((a1, a2), (self.l1, self.l2)) * np.array([[self.x], [self.y]])
    #     # print vAngle[0].item()
    #     # print vAngle[1].item()
    #
    #     g1 = MotorGoalVelocity()
    #     g1.name = self.elbow
    #     g1.velocity = vAngle[0].item()
    #
    #     g2 = MotorGoalVelocity()
    #     g2.name = self.wrist
    #     g2.velocity = vAngle[1].item()
    #
    #     self.jointmotor_proxy.setVelocity(g1)
    #     self.jointmotor_proxy.setVelocity(g2)

        return True
    #
    # sendData
    #
    def sendData(self, data):
        self.x = 0
        #self.x = float(data.axes[0].value)*10
        self.y = float(data.axes[1].value)*10
        self.y = 0
        self.z = float(data.axes[2].value)*10

        for b in data.buttons:
            if b.clicked:
                a0 = self.jointmotor_proxy.getMotorState(self.shoulder).pos
                a1 = self.jointmotor_proxy.getMotorState(self.elbow).pos
                a2 = self.jointmotor_proxy.getMotorState(self.wrist).pos
                self.dk(a0, a1,a2)
                break


    def dk(self, a0, a1, a2):
        x = (self.l0 + self.l1 * cos(a1) + self.l2 * cos(a1 + a2)) * cos(a0)
        y = (self.l0 + self.l1 * cos(a1) + self.l2 * cos(a1 + a2)) * sin(a0)
        z = self.l1 * sin(a1) + self.l2 * sin(a1 + a2)

        print "x = {0}".format(x)
        print "y = {0}".format(y)
        print "z = {0}".format(z)
        print "......................."

            
    def OldinvJacobian(self, A, L): # todo borrar
        dHxdA1 = -L[0]*sin(A[0]) - L[1]*sin(A[0]+A[1])
        dHxdA2 = -L[1]*sin(A[0]+A[1])
        dHydA1 = L[0]*cos(A[0]) + L[1]*cos(A[0]+A[1])
        dHydA2 = L[1]*cos(A[0]+A[1])
        J = np.matrix([[dHxdA1,dHxdA2],[dHydA1,dHydA2]])
        iJ= np.linalg.inv(J)
        return iJ
    def invJacobian(self, a0, a1, a2):

        dXdA0 = -(self.l0 + self.l1*cos(a1) + self.l2*cos(a1 + a2))*sin(a0)
        dXdA1 = (-self.l1*sin(a1) - self.l2*sin(a1 + a2))*cos(a0)
        dXdA2 = -self.l2*sin(a1 + a2)*cos(a0)

        dYdA0 = (self.l0 + self.l1*cos(a1) + self.l2*cos(a1 + a2))*cos(a0)
        dYdA1 = (-self.l1*sin(a1) - self.l2*sin(a1 + a2))*sin(a0)
        dYdA2 = -self.l2*sin(a0)*sin(a1 + a2)

        dZdA0 = 0
        dZdA1 = self.l1*cos(a1) + self.l2*cos(a1 + a2)
        dZdA2 = self.l2*cos(a1 + a2)

        J = np.matrix([[dXdA0, dXdA1, dXdA2], [dYdA0, dYdA1, dYdA2],[dZdA0, dZdA1, dZdA2]])
        return np.linalg.inv(J)
