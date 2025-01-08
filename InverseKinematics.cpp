
#include <iostream>
#include <cmath>
#include <array>
#include "Eigen/Dense"

#define PI 3.14159
#define LEN1 1
#define LEN2 1

#define X_INIT -0.31
#define Y_INIT -0.707

//2D std::array
template<typename T, std::size_t Row, std::size_t Col>
using Array2d = std::array<std::array<T, Col>, Row>;

using Vector3 = std::array<double, 3>;

//Functions to return the dimensions of 2d array
template<typename T, std::size_t Row, std::size_t Col>
constexpr size_t rowLen(const Array2d<T, Row, Col>&) {
    return Row;
}

template<typename T, std::size_t Row, std::size_t Col>
constexpr size_t colLen(const Array2d<T, Row, Col>&) {
    return Col;
}

Vector3 vector_subtract(const Vector3& vector1, const Vector3& vector2);
Vector3 cross_product(const Vector3& vector1, const Vector3& vector2);

template <typename T1, typename T2, std::size_t resultRows, std::size_t resultCols>
Array2d<double, resultRows, resultCols> matmul(const T1& mat1, const T2& mat2);

Vector3 calculate_end_effector_position(double theta1, double theta2);
Vector3 calculate_joint1_position(double theta1);

Array2d<double, 3, 2> compute_jacobian_matrix(double theta1, double theta2);

template<typename T_in, typename T_out>
T_out compute_psuedoinverse(T_in mat);

void calculate_joint_angles(Array2d<double, 3, 1> pos_change, double& angle1, double& angle2);

Array2d<double, 3, 1> position_func1(double t);

double deg_to_rad(double deg);
void print_vector(Vector3 vec);

template<typename T1>
Vector3 array_to_vector(const T1& arr);

int main()
{
    /*Array2d<double, 3, 2> mat3 = compute_jacobian_matrix(deg_to_rad(45.0), deg_to_rad(45.0));

    for (int i = 0; i < rowLen(mat3); i++) {
        for (int j = 0; j < colLen(mat3); j++) {
            std::cout << mat3[i][j] << "   ";
        }
        std::cout << '\n';
    }

    Array2d<double, 2, 3> pinv = compute_psuedoinverse<Array2d<double, 3, 2>, Array2d<double, 2, 3> >(mat3);

    for (int i = 0; i < rowLen(pinv); i++) {
        for (int j = 0; j < colLen(pinv); j++) {
            std::cout << pinv[i][j] << "   ";
        }
        std::cout << '\n';
    }*/

    Array2d<double, 3, 1> initial_pos = { {X_INIT, Y_INIT, 0} };
    double angle1 = 45.0;
    double angle2 = 45.0;

    for (int t = 0; t < 500; t++) {
        Array2d<double, 3, 1> new_pos = position_func1(t);
        
        Array2d<double, 3, 1> pos_change;
        for (int i = 0; i < 3; i++) {
            pos_change[i][0] = new_pos[i][0] - initial_pos[i][0];
        }

        Vector3 temp = array_to_vector<Array2d<double, 3, 1>>(new_pos);
        std::cout << "New position: ";
        print_vector(temp);
        calculate_joint_angles(pos_change, angle1, angle2);

        Vector3 test = calculate_end_effector_position(deg_to_rad(angle1), deg_to_rad(angle2));
        std::cout << "Calculated position: ";
        print_vector(test);
        std::cout << "\n\n";
    }

    return 0;
}

//Cross product of two 3D vectors
Vector3 cross_product(const Vector3& vector1, const Vector3& vector2) {
    Vector3 result;
    result[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1];
    result[1] = vector1[2] * vector2[0] - vector1[0] * vector2[2];
    result[2] = vector1[0] * vector2[1] - vector1[1] * vector2[0];

    return result;
}

//Subtract two 3D vectors
Vector3 vector_subtract(const Vector3& vector1, const Vector3& vector2) {
    Vector3 result;
    result[0] = vector1[0] - vector2[0];
    result[1] = vector1[1] - vector2[1];
    result[2] = vector1[2] - vector2[2];

    return result;
}

//Convert 3x1 or 4x1 array to a Vector3
template<typename T1>
Vector3 array_to_vector(const T1& arr) {
    if ((rowLen(arr) != 3 && rowLen(arr) != 4) || colLen(arr) != 1) {
        std::throw_with_nested(std::runtime_error("Invalid dimensions to convert to Vector3"));
    }

    Vector3 result;
    result[0] = arr[0][0];
    result[1] = arr[1][0];
    result[2] = arr[2][0];

    return result;
}

//Calculate the end effector position by using Euclidean transformation matrices to get the position as a function of the angles
Vector3 calculate_joint1_position(double theta1) {
    Array2d<double, 4, 4> t1 = { {{cos(-theta1), -sin(-theta1), 0, 0}, {sin(-theta1), cos(-theta1), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}} };

    Array2d<double, 4, 1> t2 = { {1, 0, 0, 1} };

    Array2d<double, 4, 1> pos = matmul<Array2d<double, 4, 4>, Array2d<double, 4, 1>, 4, 1>(t1, t2);

    return array_to_vector<Array2d<double, 4, 1>>(pos);
}

//Calculate the end effector position by using Euclidean transformation matrices to get the position as a function of the angles
Vector3 calculate_end_effector_position(double theta1, double theta2) {
    Array2d<double, 4, 4> t1 = { {{cos(-theta1), -sin(-theta1), 0, 0}, {sin(-theta1), cos(-theta1), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}} };
    Array2d<double, 4, 4> t2 = { {{cos(theta2), -sin(theta2), 0, 1}, {sin(theta2), cos(theta2), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}} };

    Array2d<double, 4, 4> t_comb = matmul<Array2d<double, 4, 4>, Array2d<double, 4, 4>, 4, 4>(t1, t2);

    Array2d<double, 4, 1> t3 = { {-1, 0, 0, 1} };

    Array2d<double, 4, 1> pos = matmul<Array2d<double, 4, 4>, Array2d<double, 4, 1>, 4, 1>(t_comb, t3);

    return array_to_vector<Array2d<double, 4, 1>>(pos);
}

//Multiply two matrices of any size
template <typename T1, typename T2, std::size_t resultRows, std::size_t resultCols>
Array2d<double, resultRows, resultCols> matmul(const T1& mat1, const T2& mat2) {
    //Check that they can be multiplied
    Array2d<double, resultRows, resultCols> result;

    if (colLen(mat1) != rowLen(mat2)) {
        std::throw_with_nested(std::runtime_error("Invalid matrix multiplication"));
    }

    for (int i = 0; i < rowLen(mat1); i++) {
        for (int j = 0; j < colLen(mat2); j++) {
            //Iterate through the ith row of the first matrix and the jth column of the second matrix at the same time
            double sum = 0;
            for (int index = 0; index < colLen(mat1); index++) {
                sum += mat1[i][index] * mat2[index][j];
            }

            result[i][j] = sum;
        }
    }

    return result;
}

//Compute Jacobian matrix given the current angles of the joints
Array2d<double, 3, 2> compute_jacobian_matrix(double theta1, double theta2) {
    Array2d<double, 3, 2> result;
    //Calculate for column 1 (partial derivatives with respect to theta1)
    Vector3 r_joint = { 0, 0, 0 };
    Vector3 r_effector = calculate_end_effector_position(theta1, theta2);

    Vector3 axis_of_rotation = { 0, 0, 1 };

    Vector3 r_result = vector_subtract(r_effector, r_joint);
    r_result = cross_product(axis_of_rotation, r_result);

    result[0][0] = r_result[0];
    result[1][0] = r_result[1];
    result[2][0] = r_result[2];

    //Calculate for column 2 (partial derivatives with respect to theta2)
    r_joint = calculate_joint1_position(theta1);

    r_result = vector_subtract(r_effector, r_joint);
    r_result = cross_product(axis_of_rotation, r_result);

    result[0][1] = r_result[0];
    result[1][1] = r_result[1];
    result[2][1] = r_result[2];

    return result;
}

//Compute psuedoinverse of a matrix
template<typename T_in, typename T_out>
T_out compute_psuedoinverse(T_in mat) {
    Eigen::MatrixXd eigenmat(rowLen(mat), colLen(mat));

    for (int i = 0; i < rowLen(mat); i++) {
        for (int j = 0; j < colLen(mat); j++) {
            eigenmat(i, j) = mat[i][j];
        }
    }

    Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(eigenmat.rows(), eigenmat.cols());
    cod.compute(eigenmat);

    Eigen::MatrixXd pinv = cod.pseudoInverse();

    T_out pinv_return;

    for (int i = 0; i < rowLen(pinv_return); i++) {
        for (int j = 0; j < colLen(pinv_return); j++) {
            pinv_return[i][j] = pinv(i, j);
        }
    }

    return pinv_return;
}

Array2d<double, 3, 1> position_func1(double t) {

    Array2d<double, 3, 1> result_pos;
    result_pos[0][0] = X_INIT + t / 1000;
    result_pos[1][0] = Y_INIT + t / 1000;
    result_pos[2][0] = 0;

    return result_pos;
}

//Calculate changes in joint angle needed
void calculate_joint_angles(Array2d<double, 3, 1> pos_change, double& angle1, double& angle2) {
    Array2d<double, 3, 2> jacobian = compute_jacobian_matrix(deg_to_rad(angle1), deg_to_rad(angle2));

    Array2d<double, 2, 3> jacobian_inv = compute_psuedoinverse<Array2d<double, 3, 2>, Array2d<double, 2, 3>>(jacobian);

    Array2d<double, 2, 1> angle_change = matmul<Array2d<double, 2, 3>, Array2d<double, 3, 1>, 2, 1>(jacobian_inv, pos_change);

    angle1 = angle1 + angle_change[0][0];
    angle2 = angle2 + angle_change[1][0];

    std::cout << "Angle deltas: " << angle_change[0][0] << "  " << angle_change[1][0] << "\n";
    std::cout << "Angles: " << angle1 << "  " << angle2 << "\n";
}

double deg_to_rad(double deg) {
    return deg * PI / 180.0;
}

void print_vector(Vector3 vec) {
    std::cout << "Vector: <" << vec[0] << " " << vec[1] << " " << vec[2] << "> \n";
}