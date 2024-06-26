#!/usr/bin/env python3

import tmr4243_interfaces.msg
import numpy as np
import typing

def stationkeeping() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    eta_d = np.array([0.0, 0.0, 0.0], dtype=float)
    eta_ds = np.array([0.0, 0.0, 0.0], dtype=float)
    eta_ds2 = np.array([0.0, 0.0, 0.0], dtype=float)

    return eta_d, eta_ds, eta_ds2