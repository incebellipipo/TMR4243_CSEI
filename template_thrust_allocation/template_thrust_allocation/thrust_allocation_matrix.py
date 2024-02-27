#!/usr/bin/env python3

import numpy as np
import typing

def polar_configuration_matrix(a1: float, a2: float):
    """
    This function returns the thrust configuration matrix B for the given angles
    of the port and starboard thrusters.

    Hint: This function should return a 3x3 matrix.

    :param a1: float Angle of the port thruster
    :param a2: float Angle of the starboard thruster
    :return: np.ndarray The thrust configuration matrix B
    """

    # Replace the following line and write your code below
    B = np.zeros((3, 3))


    return B

def extended_configuration_matrix():
    """
    This function returns the extended thrust configuration matrix B

    :return: np.ndarray The extended thrust configuration matrix B
    """

    # Replace the following line and write your code below
    B = np.zeros((3, 5))


    return B
