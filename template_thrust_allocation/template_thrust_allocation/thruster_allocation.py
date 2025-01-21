#!/usr/bin/env python3

import numpy as np


def thruster_allocation(tau: np.ndarray) -> np.ndarray:
    # tau is 3x1 vector

    # return 5x1 vector
    u = np.array((5, 1), dtype=float).T
    return u
