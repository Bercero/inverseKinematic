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

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 2000
        #self.timer.start(self.Period)

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
        # print 'SpecificWorker.compute...'
        #computeCODE
        #try:
        #	self.differentialrobot_proxy.setSpeedBase(100, 0)
        #except Ice.Exception, e:
        #	traceback.print_exc()
        #	print e

        # The API of python-innermodel is not exactly the same as the C++ version
        # self.innermodel.updateTransformValues("head_rot_tilt_pose", 0, 0, 0, 1.3, 0, 0)
        # z = librobocomp_qmat.QVec(3,0)
        # r = self.innermodel.transform("rgbd", z, "laser")
        # r.printvector("d")
        # print r[0], r[1], r[2]

        return True


    #
    # sendData
    #
    def sendData(self, data):
        x = float(data.axes[1].value)
        y = float(data.axes[2].value)
        alpha,beta = self.ik(x, y)
        
        print 'x={0}'.format(x)
        print 'y={0}'.format(y)
        print 'alpha={0}'.format(alpha)
        print 'beta={0}'.format(beta)
        
        g1 = MotorGoalPosition()
        g1.name=self.elbow
        g1.position=alpha
        g1.maxSpeed=self.max_speed
        
        g2 = MotorGoalPosition()
        g2.name=self.wrist
        g2.position=beta
        g2.maxSpeed=self.max_speed
        
        self.jointmotor_proxy.setPosition(g1)
        self.jointmotor_proxy.setPosition(g2)
        pos = self.jointmotor_proxy.getMotorState(self.elbow).pos
        print '{0}={1}'.format(self.elbow, pos)
        pos = self.jointmotor_proxy.getMotorState(self.wrist).pos
        print '{0}={1}'.format(self.wrist, pos)

    def ik(self, x, y):
        l1 = 1.0  # l1 y l2 tienen que ser float
        l2 = 1.0
        r2 = pow(x, 2) + pow(y, 2)
        r = sqrt(r2)
        numerador = -r2 + pow(l1, 2) + pow(l2, 2)
        cosBetaP = numerador / (2 * l1 * l2)
        radicando = 1 - pow(cosBetaP, 2)
        senoBetaP = sqrt(radicando)
        betaP = 0.0
        if cosBetaP != 0:
            betaP = atan(senoBetaP / cosBetaP)
        else:
            betaP = asin(senoBetaP)

        senoAlphaP = l2 * senoBetaP / r
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

