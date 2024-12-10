#include <iostream>
#include <Eigen/Dense>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Use namespaces for convenience
using namespace std;
using namespace Eigen;

// Function to create the transformation matrix for a given angle and link length
Matrix4d transformation_matrix(double theta, double length) {
    Matrix4d T;
    T << cos(theta), -sin(theta), 0, length * cos(theta),
         sin(theta),  cos(theta), 0, length * sin(theta),
         0,           0,          1, 0,
         0,           0,          0, 1;
    return T;
}

// Analytical pseudo-inverse for a 2x3 Jacobian
Matrix<double, 3, 2> analytical_pseudo_inverse(
    double theta1, double theta2, double theta3, 
    double l1, double l2, double l3) 
{
    // Compute combined angles
    double theta12 = theta1 + theta2;
    double theta123 = theta12 + theta3;

    // Compute sine and cosine terms
    double sin1 = sin(theta1), cos1 = cos(theta1);
    double sin12 = sin(theta12), cos12 = cos(theta12);
    double sin123 = sin(theta123), cos123 = cos(theta123);

    // Compute elements of Jp
    double J11 = - (l1 * sin1 + l2 * sin12 + l3 * sin123);
    double J12 = - (l2 * sin12 + l3 * sin123);
    double J13 = - l3 * sin123;
    double J21 = l1 * cos1 + l2 * cos12 + l3 * cos123;
    double J22 = l2 * cos12 + l3 * cos123;
    double J23 = l3 * cos123;

    // Compute A, B, C for Jp * Jp^T
    double A = J11 * J11 + J12 * J12 + J13 * J13;
    double B = J11 * J21 + J12 * J22 + J13 * J23;
    double C = J21 * J21 + J22 * J22 + J23 * J23;

    // Compute determinant
    double det = A * C - B * B;

    // Prevent division by zero
    if (abs(det) < 1e-6) {
        throw runtime_error("Jacobian is singular; pseudo-inverse cannot be computed analytically.");
    }

    // Compute inverse elements
    double inv_A = C / det;
    double inv_B = -B / det;
    double inv_C = A / det;

    // Compute Jp_pseudo_inverse elements
    Matrix<double, 3, 2> J_pinv;
    J_pinv(0, 0) = J21 * inv_A + (-J11) * inv_B;
    J_pinv(0, 1) = (-J21) * inv_B + J11 * inv_A;

    J_pinv(1, 0) = J22 * inv_A + (-J12) * inv_B;
    J_pinv(1, 1) = (-J22) * inv_B + J12 * inv_A;

    J_pinv(2, 0) = J23 * inv_A + (-J13) * inv_B;
    J_pinv(2, 1) = (-J23) * inv_B + J13 * inv_A;

    return J_pinv;
}

// Function to calculate the Jacobian matrix and pseudo-inverse
pair<Matrix<double, 2, 3>, Matrix<double, 3, 2>> 
calculate_jacobian(double theta1, double theta2, double theta3, double l1, double l2, double l3, double damping = 0.01) {
    // Calculate transformation matrices
    Matrix4d T1 = transformation_matrix(theta1, l1);
    Matrix4d T2 = transformation_matrix(theta2, l2);
    Matrix4d T3 = transformation_matrix(theta3, l3);

    // Overall transformation matrix from base to end-effector
    Matrix4d T0_3 = T1 * T2 * T3;

    // End-effector position (last column, first three elements)
    Vector3d p = T0_3.block<3, 1>(0, 3);

    // Compute the positional part of the Jacobian (2x3)
    Matrix<double, 2, 3> Jp;
    Jp(0, 0) = -p[1];
    Jp(0, 1) = -(p[1] - T1(1, 3));
    Jp(0, 2) = -(p[1] - (T1 * T2)(1, 3));
    
    Jp(1, 0) = p[0];
    Jp(1, 1) = p[0] - T1(0, 3);
    Jp(1, 2) = p[0] - (T1 * T2)(0, 3);

    // Compute the pseudo-inverse analytically
    Matrix<double, 3, 2> Jp_pseudo_inverse;
    try {
        Jp_pseudo_inverse = analytical_pseudo_inverse(theta1, theta2, theta3, l1, l2, l3);
    } catch (const exception& e) {
        cerr << e.what() << endl;
        // If Jacobian is singular, use damped pseudo-inverse (fallback)
        Jp_pseudo_inverse = Jp.transpose() * 
                            (Jp * Jp.transpose() + damping * Matrix2d::Identity()).inverse();
    }

    return {Jp, Jp_pseudo_inverse};
}

int main() {
    // Example numeric values for joint angles (in radians) and link lengths
    double theta1 = M_PI / 4; // 45 degrees in radians
    double theta2 = M_PI / 4;
    double theta3 = M_PI / 4;
    double l1 = 1.0, l2 = 1.0, l3 = 1.0;

    // Calculate the Jacobian and its pseudo-inverse
    auto [Jacobian, Jacobian_pseudo_inverse] = calculate_jacobian(theta1, theta2, theta3, l1, l2, l3);

    // Display the results
    cout << "Positional Jacobian Matrix (Jp):\n" << Jacobian << "\n\n";
    cout << "Pseudo-Inverse of the Positional Jacobian Matrix (Jp^+):\n" << Jacobian_pseudo_inverse << "\n";

    return 0;
}
