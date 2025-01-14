
#include <iostream>
#include <cmath>
#include <array>
#include "Eigen/Dense"
#include "InverseKinematics.h"

int main()
{
    Eigen::Vector3d initial_pos {X_INIT, Y_INIT, 0};

    //These will update as the end effector follows the trajectory
    double angle1 = 45.0;
    double angle2 = 45.0;

    for (int t = 0; t < 500; t++) {
        Eigen::Vector3d new_pos;
        new_pos = position_func1(t);

        Eigen::Vector3d pos_change;
        pos_change = new_pos = initial_pos;

        std::cout << "New position: " << new_pos;

        calculate_joint_angles(pos_change, angle1, angle2);

        Eigen::Vector3d test = calculate_end_effector_position(deg_to_rad(angle1), deg_to_rad(angle2));
        std::cout << "Calculated position: " << test << "\n\n";

        initial_pos = new_pos;
    }

    return 0;
}

//Calculate the end effector position by using Euclidean transformation matrices to get the position as a function of the angles
Eigen::Vector3d calculate_joint1_position(double theta1) {
    Eigen::Matrix4d t1 {
        {cos(-theta1), -sin(-theta1), 0, 0},
        { sin(-theta1), cos(-theta1), 0, 0 },
        { 0, 0, 1, 0 },
        { 0, 0, 0, 1 }
    };

    Eigen::Vector4d t2 {1, 0, 0, 1};

    Eigen::Vector4d pos;
    pos = t1 * t2;

    //Remove last "dummy" value
    return pos.head<3>();
}

//Calculate the end effector position by using Euclidean transformation matrices to get the position as a function of the angles
Eigen::Vector3d calculate_end_effector_position(double theta1, double theta2) {
    Eigen::Matrix4d t1 {
        {cos(-theta1), -sin(-theta1), 0, 0}, 
        { sin(-theta1), cos(-theta1), 0, 0 }, 
        { 0, 0, 1, 0 }, 
        { 0, 0, 0, 1 }
    };

    Eigen::Matrix4d t2 {
        {cos(theta2), -sin(theta2), 0, LEN1}, 
        {sin(theta2), cos(theta2), 0, 0}, 
        {0, 0, 1, 0}, 
        {0, 0, 0, 1}
    };

    Eigen::Vector4d t3 {LEN2, 0, 0, 1};

    Eigen::Vector4d pos;
    pos = t1 * t2 * t3;

    return pos.head<3>();
}

//Compute Jacobian matrix given the current angles of the joints
Eigen::MatrixXd compute_jacobian_matrix(double theta1, double theta2) {
    Eigen::MatrixXd result(3, 2);
    //Calculate for column 1 (partial derivatives with respect to theta1)
    Eigen::Vector3d r_joint { 0, 0, 0 };
    Eigen::Vector3d r_effector = calculate_end_effector_position(theta1, theta2);

    Eigen::Vector3d axis_of_rotation { 0, 0, 1 };

    Eigen::Vector3d r_result = axis_of_rotation.cross(r_effector - r_joint);

    //Add this as a column vector into the result matrix
    result(Eigen::all, 0) = r_result;

    //Calculate for column 2 (partial derivatives with respect to theta2)
    r_joint = calculate_joint1_position(theta1);

    r_result = axis_of_rotation.cross(r_effector - r_joint);

    //Add this as a column vector into the result matrix
    result(Eigen::all, 1) = r_result;

    return result;
}

Eigen::Vector3d position_func1(double t) {

    Eigen::Vector3d result_pos;
    result_pos(0) = X_INIT + t / 1000;
    result_pos(1) = Y_INIT + t / 1000;
    result_pos(2) = 0;

    return result_pos;
}

//Calculate changes in joint angle needed
void calculate_joint_angles(Eigen::Vector3d pos_change, double& angle1, double& angle2) {
    Eigen::MatrixXd jacobian(3, 2);
    jacobian = compute_jacobian_matrix(deg_to_rad(angle1), deg_to_rad(angle2));

    //Take Moore-Penrose pseudoinverse of matrix
    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(jacobian.rows(), jacobian.cols());
    cod.compute(jacobian);
    Eigen::MatrixXd jacobian_inv = cod.pseudoInverse();

    Eigen::MatrixXd angle_change = jacobian_inv * pos_change;

    //Update angles
    angle1 = angle1 + angle_change(0, 0);
    angle2 = angle2 + angle_change(1, 0);

    std::cout << "Angle deltas: " << angle_change(0, 0) << "  " << angle_change(1, 0) << "\n";
    std::cout << "Angles: " << angle1 << "  " << angle2 << "\n";
}

double deg_to_rad(double deg) {
    return deg * PI / 180.0;
}