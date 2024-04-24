#!/usr/bin/env python3

import numpy as np
from template_observer.wrap import wrap


def luenberger(
        eta: np.ndarray,
        tau: np.ndarray,
        L1: np.ndarray,
        L2: np.ndarray,
        L3: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:

    eta_hat = np.zeros((3, 1), dtype=float)
    nu_hat = np.zeros((3, 1), dtype=float)
    bias_hat = np.zeros((3, 1), dtype=float)


    # Enter your code here

    return eta_hat, nu_hat, bias_hat
