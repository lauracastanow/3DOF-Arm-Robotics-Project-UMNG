import numpy as np
from config import L1, L2, L3

# Cinemática directa
def forward_kinematics(theta1, theta2, d3):

    x = L1*np.cos(theta1) + L2*np.cos(theta1 + theta2)
    y = L1*np.sin(theta1) + L2*np.sin(theta1 + theta2)
    z = L3 - d3

    return x, y, z


# Cinemática inversa
def inverse_kinematics(x, y, z):

    r = np.sqrt(x**2 + y**2)

    cos_theta2 = (r**2 - L1**2 - L2**2)/(2*L1*L2)
    theta2 = np.arccos(cos_theta2)

    theta1 = np.arctan2(y, x) - np.arctan2(L2*np.sin(theta2), L1 + L2*np.cos(theta2))

    d3 = L3 - z

    return theta1, theta2, d3