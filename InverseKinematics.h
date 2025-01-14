#ifndef InverseKinematics_h

#define InverseKinematics_h

#define PI 3.14159

//Lengths of the leg parts
#define LEN1 1
#define LEN2 -1 //Negative because it is effectively a negative distance in the way the frames are set up

#define X_INIT -0.31
#define Y_INIT -0.707

Eigen::Vector3d calculate_end_effector_position(double theta1, double theta2);
Eigen::Vector3d calculate_joint1_position(double theta1);

Eigen::MatrixXd compute_jacobian_matrix(double theta1, double theta2);

void calculate_joint_angles(Eigen::Vector3d pos_change, double& angle1, double& angle2);

Eigen::Vector3d position_func1(double t);

double deg_to_rad(double deg);

#endif