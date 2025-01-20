#ifndef InverseKinematics_h

#define InverseKinematics_h

#define PI 3.14159

//Lengths of the leg parts
#define LEN1 1
#define LEN2 1

#define X_INIT -0.31
#define Y_INIT -0.707

void followTrajectory(Eigen::Vector3d initial_pos, double& angle1, double& angle2, Eigen::Vector3d(*position_function)(double));

Eigen::Vector3d calculate_end_effector_position(double theta1, double theta2);
Eigen::Vector3d calculate_joint1_position(double theta1);

Eigen::MatrixXd compute_jacobian_matrix(double theta1, double theta2);

//Calculates and updates new joint angles for a small change in position
void calculate_joint_angles(Eigen::Vector3d pos_change, double& angle1, double& angle2);

Eigen::Vector3d position_func1(double t);

double deg_to_rad(double deg);

#endif