import numpy as np

# Function to create the transformation matrix for a given angle and link length
def transformation_matrix(theta, length):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, length * np.cos(theta)],
        [np.sin(theta),  np.cos(theta), 0, length * np.sin(theta)],
        [0,              0,             1, 0],
        [0,              0,             0, 1]
    ])

# Analytical pseudo-inverse function for a 2x3 Jacobian
def analytical_pseudo_inverse(theta1, theta2, theta3, l1, l2, l3):
    # Compute combined angles
    theta12 = theta1 + theta2
    theta123 = theta12 + theta3

    # Compute sine and cosine terms
    sin1 = np.sin(theta1)
    cos1 = np.cos(theta1)
    sin12 = np.sin(theta12)
    cos12 = np.cos(theta12)
    sin123 = np.sin(theta123)
    cos123 = np.cos(theta123)

    # Compute elements of Jp
    J11 = - (l1 * sin1 + l2 * sin12 + l3 * sin123)
    J12 = - (l2 * sin12 + l3 * sin123)
    J13 = - l3 * sin123
    J21 = l1 * cos1 + l2 * cos12 + l3 * cos123
    J22 = l2 * cos12 + l3 * cos123
    J23 = l3 * cos123

    # Compute A, B, C for Jp Jp^T
    A = J11**2 + J12**2 + J13**2
    B = J11 * J21 + J12 * J22 + J13 * J23
    C = J21**2 + J22**2 + J23**2

    # Compute determinant
    det = A * C - B**2

    # Prevent division by zero
    if np.isclose(det, 0):
        raise ValueError("Jp Jp^T is singular; pseudo-inverse cannot be computed analytically.")

    # Compute inverse elements
    inv_A = C / det
    inv_B = -B / det
    inv_C = A / det

    # Compute Jp_pseudo_inverse elements
    J_pinv_11 = J21 * inv_A + (-J11) * inv_B
    J_pinv_12 = J22 * inv_A + (-J12) * inv_B
    J_pinv_13 = J23 * inv_A + (-J13) * inv_B

    J_pinv_21 = (-J21) * inv_B + J11 * inv_A
    J_pinv_22 = (-J22) * inv_B + J12 * inv_A
    J_pinv_23 = (-J23) * inv_B + J13 * inv_A

    # Construct the pseudo-inverse matrix (3x2)
    J_pinv = np.array([
        [J_pinv_11, J_pinv_21],
        [J_pinv_12, J_pinv_22],
        [J_pinv_13, J_pinv_23]
    ])

    return J_pinv

# Damped pseudo-inverse for singular or ill-conditioned Jacobians
def damped_pseudo_inverse(theta1, theta2, theta3, l1, l2, l3, damping=0.01):
    # Compute combined angles
    theta12 = theta1 + theta2
    theta123 = theta12 + theta3

    # Compute sine and cosine terms
    sin1 = np.sin(theta1)
    cos1 = np.cos(theta1)
    sin12 = np.sin(theta12)
    cos12 = np.cos(theta12)
    sin123 = np.sin(theta123)
    cos123 = np.cos(theta123)

    # Compute elements of Jp
    J11 = - (l1 * sin1 + l2 * sin12 + l3 * sin123)
    J12 = - (l2 * sin12 + l3 * sin123)
    J13 = - l3 * sin123
    J21 = l1 * cos1 + l2 * cos12 + l3 * cos123
    J22 = l2 * cos12 + l3 * cos123
    J23 = l3 * cos123

    # Compute Jp Jp^T
    A = J11**2 + J12**2 + J13**2
    B = J11 * J21 + J12 * J22 + J13 * J23
    C = J21**2 + J22**2 + J23**2

    # Compute determinant with damping
    det = A * C - B**2 + damping**2 * A

    # Compute inverse elements with damping
    inv_A = C / det
    inv_B = -B / det
    inv_C = A / det

    # Compute Jp_pseudo_inverse elements with damping
    J_pinv_11 = J21 * inv_A + (-J11) * inv_B
    J_pinv_12 = J22 * inv_A + (-J12) * inv_B
    J_pinv_13 = J23 * inv_A + (-J13) * inv_B

    J_pinv_21 = (-J21) * inv_B + J11 * inv_A
    J_pinv_22 = (-J22) * inv_B + J12 * inv_A
    J_pinv_23 = (-J23) * inv_B + J13 * inv_A

    # Construct the pseudo-inverse matrix (3x2)
    J_pinv = np.array([
        [J_pinv_11, J_pinv_21],
        [J_pinv_12, J_pinv_22],
        [J_pinv_13, J_pinv_23]
    ])

    return J_pinv

# Function to calculate the Jacobian matrix for given joint angles and link lengths
def calculate_jacobian(theta1, theta2, theta3, l1, l2, l3, damping=0.01):
    # Calculate transformation matrices
    T1 = transformation_matrix(theta1, l1)
    T2 = transformation_matrix(theta2, l2)
    T3 = transformation_matrix(theta3, l3)

    # Overall transformation matrix from base to end-effector
    T0_3 = T1 @ T2 @ T3  # Matrix multiplication

    # End-effector position (last column, first three elements)
    p = T0_3[:3, 3]

    # Compute the positional part of the Jacobian (2x3)
    Jp = np.zeros((2, 3))
    Jp[0, 0] = - (p[1])        # ∂x/∂θ1
    Jp[0, 1] = - (p[1] - T1[:3, 3][1])  # ∂x/∂θ2
    Jp[0, 2] = - (p[1] - (T1 @ T2)[:3, 3][1])  # ∂x/∂θ3

    Jp[1, 0] = p[0]            # ∂y/∂θ1
    Jp[1, 1] = p[0] - T1[:3, 3][0]  # ∂y/∂θ2
    Jp[1, 2] = p[0] - (T1 @ T2)[:3, 3][0]  # ∂y/∂θ3

    # Compute the pseudo-inverse analytically
    try:
        Jp_pseudo_inverse = analytical_pseudo_inverse(theta1, theta2, theta3, l1, l2, l3)
    except ValueError:
        # If Jacobian is singular, use damped pseudo-inverse
        print("Jacobian is singular. Using damped pseudo-inverse.")
        Jp_pseudo_inverse = damped_pseudo_inverse(theta1, theta2, theta3, l1, l2, l3, damping)

    return Jp, Jp_pseudo_inverse

# Example numeric values for joint angles (in radians) and link lengths
theta1 = np.radians(45)
theta2 = np.radians(45)
theta3 = np.radians(45)
l1 = 1.0
l2 = 1.0
l3 = 1.0

# Calculate the Jacobian and its pseudo-inverse
Jacobian, Jacobian_pseudo_inverse = calculate_jacobian(theta1, theta2, theta3, l1, l2, l3)

# Display the results
print("Positional Jacobian Matrix (Jp):")
print(Jacobian)
print("\nPseudo-Inverse of the Positional Jacobian Matrix (Jp^+):")
print(Jacobian_pseudo_inverse)
