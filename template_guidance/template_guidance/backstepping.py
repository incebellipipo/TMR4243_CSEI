#!/usr/bin/env python3

import numpy as np

def backstepping(observer, reference, K1_gain, K2_gain):

    # Getting the states from the observer
    eta_hat = observer.eta
    nu_hat = observer.nu
    bias_hat = observer.bias

    # Getting the states from the reference
    eta_d = reference.eta_d
    eta_ds = reference.eta_ds
    eta_ds2 = reference.eta_ds2

    w = reference.w
    v_s = reference.v_s
    v_ss = reference.v_ss

    # Replace the following line
    tau = [0, 0, 0]
    
    return tau