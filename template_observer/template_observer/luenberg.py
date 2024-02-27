#!/usr/bin/env python3

import numpy as np
from template_observer.wrap import wrap

def luenberg(eta, tau, L1, L2, L3):

    eta_hat =   np.array([0, 0, 0])
    nu_hat  =   np.array([0, 0, 0])
    bias_hat   =   np.array([0, 0, 0])

    # Log the incoming information
    print("=== Luenberg Observer ===")
    print("eta: ", eta)
    print("tau: ", tau)
    print("L1: ", L1)
    print("L2: ", L2)
    print("L3: ", L3)


    # Enter your code here



    return eta_hat, nu_hat, bias_hat