#! /usr/bin/python
from math import *


def ik(x, y):
    l1 = 1.0 #l1 y l2 tienen que ser float
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

    senoAlphaP = l2 * senoBetaP/r
    radicando = 1 - pow(senoAlphaP, 2)
    cosAlphaP = sqrt(radicando)
    alphaP = 0.0
    if cosAlphaP != 0:
        alphaP = atan(senoAlphaP / cosAlphaP)
    else:
        alpha = asin(senoAlphaP)

    gama = 0.0
    if r2 != 0:
        gama = asin(y / sqrt(r2))

    alpha = gama + alphaP
    beta=0.0
    # TODO esta parte es problematica: cuando betaP deberia ser 0 pero como hay cierta imprecision no lo es
    if betaP <= 0:
        beta=betaP
    else:
        beta = betaP - pi

    print(alpha, beta)
