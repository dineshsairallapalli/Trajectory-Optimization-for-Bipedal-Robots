### **Description**
This repository provides Python implementations of core algorithms for optimizing and controlling bipedal robot trajectories. The primary focus is on computing Jacobians and their pseudo-inverses to enable precise motion control and stability in dynamic walking scenarios.

---

### **Code Features**
1. **Pseudo-Inverse Jacobian Calculations**:
   - Implements both **analytical** and **damped pseudo-inverse** methods for positional Jacobians.
   - Handles singular or ill-conditioned Jacobians gracefully using damping techniques.

2. **Dynamic Transformations**:
   - Uses transformation matrices to compute joint-space to task-space mappings for multi-link systems.
   - Supports arbitrary configurations for multi-DOF robotic arms or bipedal robots.

3. **Real-Time Feasibility**:
   - The implementation is designed for computational efficiency, making it suitable for real-time robotic control.

4. **Customizability**:
   - Parameters like damping coefficients, joint angles, and link lengths are fully configurable, allowing adaptation to various robotic platforms.

---

### **Key File**
- **`Final_psuedoinv.py`**:
  - **Functions**:
    - `transformation_matrix(theta, length)`: Computes the transformation matrix for a given joint angle and link length.
    - `analytical_pseudo_inverse(theta1, theta2, theta3, l1, l2, l3)`: Calculates the pseudo-inverse of a 2x3 Jacobian analytically.
    - `damped_pseudo_inverse(theta1, theta2, theta3, l1, l2, l3, damping=0.01)`: Computes a damped pseudo-inverse to handle singular configurations.
    - `calculate_jacobian(theta1, theta2, theta3, l1, l2, l3, damping=0.01)`: Combines Jacobian calculation and pseudo-inverse determination.

  - **Highlights**:
    - Supports both analytical and numerical stability techniques.
    - Prevents division-by-zero errors using damping factors.
    - Demonstrates usage with example joint angles and link lengths.

---

### **How to Use**
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/Bipedal-Robot-Trajectory-Optimization.git
   cd Bipedal-Robot-Trajectory-Optimization
   ```
2. Run the main script (`Final_psuedoinv.py`) to calculate the Jacobian and its pseudo-inverse for a 3-DOF system:
   ```bash
   python Final_psuedoinv.py
   ```
3. Modify input parameters (e.g., joint angles, link lengths, damping factor) in the script to adapt to your robotic system.

