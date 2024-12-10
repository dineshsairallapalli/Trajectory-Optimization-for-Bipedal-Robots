### Analytical Pseudo-Inverse and Jacobian Matrix Computation in Robotics

This code focuses on implementing and analyzing key robotic computations using the **Eigen C++ Library**, specifically targeting the Jacobian matrix and its pseudo-inverse for a 3-joint planar manipulator. The code demonstrates a practical approach to inverse kinematics by handling singularities and providing accurate results.

#### Key Features of the Code:

1. **Transformation Matrix Calculation**:
   - The function `transformation_matrix` computes a homogeneous transformation matrix for each joint, based on its angle and link length. These matrices are crucial for determining the position and orientation of the end-effector.

2. **Analytical Pseudo-Inverse**:
   - The function `analytical_pseudo_inverse` calculates the pseudo-inverse of a 2x3 Jacobian matrix analytically by solving for matrix elements directly. This approach ensures computational efficiency and precision.
   - Singularities in the Jacobian matrix (e.g., when the determinant approaches zero) are detected, and a fallback damped pseudo-inverse is used to ensure robustness.

3. **Jacobian Matrix Calculation**:
   - The `calculate_jacobian` function constructs the positional Jacobian matrix (`Jp`) from the transformation matrices.
   - It also calculates the pseudo-inverse (`Jp^+`), either analytically or using the damped method as a fallback, to account for possible singularities.

4. **Numerical Example**:
   - The `main` function includes a working example with specified joint angles (`theta1`, `theta2`, `theta3`) and link lengths (`l1`, `l2`, `l3`).
   - Outputs include:
     - The computed positional Jacobian matrix (`Jp`).
     - The pseudo-inverse of the Jacobian matrix (`Jp^+`).

5. **Error Handling**:
   - Singularities in the Jacobian matrix are gracefully managed using runtime checks, ensuring that the program remains stable and reliable even in edge cases.

---

#### Eigen Library in the Code:

The **Eigen Library** plays a central role in the implementation by providing tools for efficient matrix operations and high-level abstractions. Specifically, the code uses Eigen for:

1. **Matrix Operations**:
   - Constructs transformation matrices (`Matrix4d`) and Jacobian matrices (`Matrix<double, 2, 3>`).
   - Performs matrix multiplication and inversion for computing pseudo-inverses and handling transformations.

2. **Numerical Solvers**:
   - Calculates the damped pseudo-inverse when the analytical approach fails due to singularities.
   - Efficiently computes matrix products and decompositions.

3. **Ease of Implementation**:
   - Eigen's high-level syntax simplifies complex mathematical operations, making the code cleaner and easier to understand.

---

#### How the Code Works:

- The code first computes individual transformation matrices for each joint and multiplies them to get the overall transformation to the end-effector.
- Using these transformations, it calculates the positional Jacobian matrix, which maps joint velocities to end-effector velocities.
- The pseudo-inverse of the Jacobian is then computed, allowing for inverse kinematics solutions where joint angles are adjusted to achieve desired end-effector positions.

#### Example Output:
For the given joint angles (45° each) and link lengths (1 unit each), the code computes:
1. The **Positional Jacobian Matrix (`Jp`)**.
2. The **Pseudo-Inverse of `Jp` (`Jp^+`)**, which can be used to solve for joint velocities in inverse kinematics applications.

---

### Why Eigen and This Implementation?

The use of the Eigen library ensures:
- **Performance**: Fast computations for real-time applications.
- **Flexibility**: Easy adaptation for different robot configurations or extensions.
- **Robustness**: Graceful handling of singularities using a fallback pseudo-inverse method.
