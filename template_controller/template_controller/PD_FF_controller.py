#!/usr/bin/env python3

def PD_FF_controller(observer, reference, P_gain, D_gain):

    # Getting the states from the observer
    eta_hat = observer.eta
    nu_hat = observer.nu
    bias_hat = observer.bias

    # Getting the states from the refernce
    eta_d = reference.eta_d
    eta_ds = reference.eta_ds
    eta_ds2 = reference.eta_ds2
    w = reference.w
    v_s = reference.v_s
    v_ss = reference.v_ss

    # Replace the following line
    tau = [0, 0, 0]

    #
    ## Write your code below
    #

    return tau