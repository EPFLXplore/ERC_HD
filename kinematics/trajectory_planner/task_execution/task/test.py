from scipy.spatial.transform import Rotation
import numpy as np

# Define the basis vectors of the first coordinate system
basis_vectors_ref1 = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

# Define the basis vectors of the second coordinate system
basis_vectors_ref2 = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

# Compute the rotation
rotation_matrix = Rotation.from_matrix(basis_vectors_ref1.T @ basis_vectors_ref2)

print(rotation_matrix.as_matrix())
