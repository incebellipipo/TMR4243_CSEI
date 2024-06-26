#!/usr/bin/env python3

import tmr4243_interfaces.msg
import numpy as np

def straight_line() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    eta_d = np.array([0.0, 0.0, 0.0], dtype=float)
    eta_ds = np.array([0.0, 0.0, 0.0], dtype=float)
    eta_ds2 = np.array([0.0, 0.0, 0.0], dtype=float)

    return eta_d, eta_ds, eta_ds2


def update_law() -> tuple[float, float, float]:
    w = 0.0
    v_s  = 0.0
    v_ss = 0.0

    return w, v_s, v_ss