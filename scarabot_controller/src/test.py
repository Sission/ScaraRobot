import numpy as np


def ik_calcu(x, y, z):
    cosq2 = round((np.square(x) + np.square(y) - 0.6344) * 50 / 31, 6)
    q2 = np.arctan2(np.sqrt(1 - np.square(cosq2)), cosq2)
    q3 = z - 1.68
    cosq1 = ((cosq2 / 2 + 0.62) * x + np.sin(q2) / 2 * y) / (np.square(cosq2 / 2 + 0.62) + np.square(np.sin(q2) / 2))
    q1 = np.arctan2(np.sqrt(1 - np.square(cosq1)), cosq1)
    return q1, q2, q3


# initial position
q = ik_calcu(0, 0.2, 1.68)
print q[0]