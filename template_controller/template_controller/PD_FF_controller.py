
import numpy as np
import tmr4243_interfaces.msg


def PD_FF_controller(
        observer: tmr4243_interfaces.msg.Observer,
        reference: tmr4243_interfaces.msg.Reference,
        P_gain: float,
        D_gain: float) -> np.ndarray:

    tau = np.zeros((3, 1), dtype=float)

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

    return tau