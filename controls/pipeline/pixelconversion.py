import numpy as np


def conversion(pixel, altitude):
    K = np.array([[1, 0, 1], [0, 1, 1], [0, 0, 10]])

    u, v = pixel
    Z = altitude

    X_c = (u - K[0, 2]) * Z / K[0, 0]
    Y_c = (v - K[1, 2]) * Z / K[1, 1]
    Z_c = Z

    # Define extrinsic parameters for a downward-facing camera
    R = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])  # Rotation matrix

    # Transform to world coordinates
    coords = np.array([X_c, Y_c, Z_c])
    coords = np.dot(R, coords)

    return coords
