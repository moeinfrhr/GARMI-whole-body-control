#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <ncurses.h>
#include <algorithm>

extern "C" {
#include <arm_control/moein_helpers/openGJK.h>
// #include "dubins.c"
}

#include <arm_control/moein_helpers/redundancy_resolution.h>

using namespace Eigen;
typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<double, 7, 1> Vector7d;

// Cross Product Operator
Matrix3d crossOperator(Vector3d vec) {
    Matrix3d crossMatrix;
    crossMatrix << 0.0, -vec.z(), vec.y(),
                   vec.z(), 0.0, -vec.x(),
                   -vec.y(), vec.x(), 0.0;
    return crossMatrix;
}

// Denavit-Hartenberg Matrix
Matrix4d DHmatrix(const VectorXd& q, int i) {
    double l = 0.1; // Tool length
    VectorXd d(8); d << 0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107 + l;
    VectorXd a(8); a << 0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0;
    VectorXd alpha(8); alpha << 0, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0;
    VectorXd q_ext(8);
    q_ext << q, 0;

    double ct = cos(q_ext(i));
    double st = sin(q_ext(i));
    double ca = cos(alpha(i));
    double sa = sin(alpha(i));

    Matrix4d T;
    T << ct, -st, 0, a(i),
         st * ca, ct * ca, -sa, -sa * d(i),
         st * sa, ct * sa, ca, ca * d(i),
         0, 0, 0, 1;
    return T;
}

Matrix4d FK(const VectorXd& Q, int index, bool isRightArm) {  // forward kinematics
    double x = Q(0);
    double y = Q(1);
    double theta = Q(2);
    VectorXd q_arm = (isRightArm) ? Q.segment(3, 7) : Q.segment(10, 7);

    // Parameters Definition
    double width = 0.75;
    double height = 0.2;
    double depth = 0.75;
    double wheelRadius = height / 2;
    double spineHeight = 1.05;
    double spineOffset = 0.1;
    double shoulderWidth = 0.4;

    double beta = M_PI / 2.0;
    double gamma = isRightArm ? M_PI / 2.0 : -M_PI / 2.0;
    Vector3d shoulderPose = isRightArm ? Vector3d(spineOffset, -shoulderWidth / 2, height + spineHeight) : Vector3d(spineOffset, shoulderWidth / 2, height + spineHeight);

    // Transformation matrix for the base of the robot
    Matrix4d T_mobile_base;
    T_mobile_base << cos(theta), -sin(theta), 0, x,
                     sin(theta), cos(theta), 0, y,
                     0, 0, 1, 0,
                     0, 0, 0, 1;

    // Transformation matrix for the shoulder pose
    Matrix4d Tshoulder;
    Tshoulder << 1, 0, 0, shoulderPose(0),
                 0, 1, 0, shoulderPose(1),
                 0, 0, 1, shoulderPose(2),
                 0, 0, 0, 1;

    // Transformation matrix for the robot's wall-mount arm installation
    Matrix4d Ty, Tz;
    Ty << cos(beta), 0, sin(beta), 0,
          0, 1, 0, 0,
          -sin(beta), 0, cos(beta), 0,
          0, 0, 0, 1;

    Tz << cos(gamma), -sin(gamma), 0, 0,
          sin(gamma), cos(gamma), 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

    // Transformation of the arm_base to the universal frame
    Matrix4d Ti = T_mobile_base * Tshoulder * Ty * Tz;

    for (int joint_id = 0; joint_id <= index; ++joint_id) {
        Ti = Ti * DHmatrix(q_arm, joint_id); // Apply rotation matrix to compensate for arm configuration
    }

    return Ti;
}

VectorXd pose_arm(const VectorXd& Q, int index, bool isRightArm) {
    VectorXd X(6);
    X.setZero();

    // Compute the transformation matrix for the specified configuration
    MatrixXd T_idx = FK(Q, index, isRightArm);

    // Extract the rotation matrix from the transformation matrix
    Matrix3d R = T_idx.block<3, 3>(0, 0);

    // Convert the rotation matrix to an axis-angle representation
    double angle = std::acos((R.trace() - 1) / 2);
    Vector3d axis;
    if (angle != 0) {
        axis << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
        axis /= axis.norm();
    } else {
        axis << 0, 0, 1; // Default axis if no rotation is present
    }

    // Return pose vector
    X.segment(0, 3) = T_idx.block<3, 1>(0, 3);
    X.segment(3, 3) = angle * axis;
    return X;
}

MatrixXd getJointsPositions(const VectorXd& Q, bool isRightArm) {
    MatrixXd positions(3, 8); // Matrix to hold the positions of up to 8 joints

    for (int joint_id = 0; joint_id <=7 ; ++joint_id) {
        Matrix4d T = FK(Q, joint_id, isRightArm); // Use FK to find the transformation matrix
        positions.col(joint_id) = T.block<3, 1>(0, 3); // Extract the translation part (position) and store it in the matrix
    }
    return positions;
}

MatrixXd JacobArm(const VectorXd& Q, int index, bool isRightArm) {
    Matrix<double, 6, 7> jacobian; // Explicitly define Jacobian matrix size
    jacobian.setZero();

    // Compute FK up to the specified joint index
    Matrix4d T_joint_index = FK(Q, index, isRightArm);
    Vector3d P_joint_index = T_joint_index.block<3, 1>(0, 3); // Extract the position vector of the joint at the specified index

    for (int joint_id = 0; joint_id < index; ++joint_id) { // Count joint positions up to one before the target joint
        // Compute the transformation matrix up to the current joint
        Matrix4d Ti = FK(Q, joint_id, isRightArm);

        // Extract the joint axis of the current joint
        Vector3d P_i = Ti.block<3, 1>(0, 3);

        // Calculate the columns of the Jacobian matrix
        Vector3d z_current = Ti.block<3, 1>(0, 2); // z-axis
        Vector3d p_diff = P_joint_index - P_i;

        // Update the Jacobian matrix
        // Linear velocity part
        jacobian.block<3, 1>(0, joint_id) = z_current.cross(p_diff);
        // Angular velocity part
        jacobian.block<3, 1>(3, joint_id) = z_current;
    }

    return jacobian;
}

MatrixXd JacobBody(const VectorXd& Q, int index, bool isRightArm) {

    // Compute FK up to the specified joint index
    Matrix4d T_joint_index = FK(Q, index, isRightArm);

    // Extract the position vector of the joint at the specified index
    // in absolute frame relative to robot base.
    Vector3d P_joint_index = T_joint_index.block<3,1>(0, 3) - Vector3d(Q(0), Q(1), 0); // find the position of base: Q(0:2) = [x,y,theta]'

    // Nonholonomic matrix setup (assuming linear and angular velocity command scenario)
    MatrixXd nonholonomicMatrix(6, 3);
    nonholonomicMatrix << 1, 0, 0,
                          0, 1, 0,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 1;

    // Calculate the Jacobian for the platform
    Matrix3d crossProdMatrix = crossOperator(P_joint_index);
    MatrixXd JacobianPlatform(6, 6);
    JacobianPlatform.setZero(); // Initialize with zeros
    JacobianPlatform.block<3, 3>(0, 0) = Matrix3d::Identity();
    JacobianPlatform.block<3, 3>(0, 3) = -crossProdMatrix;
    JacobianPlatform.block<3, 3>(3, 3) = Matrix3d::Identity();

    // Apply the nonholonomic constraints to the platform Jacobian
    MatrixXd Jacobian = JacobianPlatform * nonholonomicMatrix;

    return Jacobian;
}

MatrixXd JacobianWholeBody(const VectorXd& Q, int index, bool isRightArm) {
    MatrixXd J_wb(6, 17); // Whole-body Jacobian matrix with 6 rows and 14 columns
    J_wb.setZero();

    MatrixXd J_arm = JacobArm(Q, index, isRightArm);
    MatrixXd J_body = JacobBody(Q, index, isRightArm);

    J_wb.leftCols(J_body.cols()) = J_body;
    if (isRightArm) {
        J_wb.block(0, J_body.cols(), 6, J_arm.cols()) = J_arm;
    } else {
        J_wb.block(0, J_body.cols() + J_arm.cols(), 6, J_arm.cols()) = J_arm;
    }

    return J_wb;
}

MatrixXd JacobArm_cg(const VectorXd& Q, int index, bool isRightArm) {
    MatrixXd jacobian(6, 7); // Pre-allocate for speed
    jacobian.setZero(); // initialize jacobian to zero. 
    MatrixXd linkCoMs(3, 7);
    
    // Updated CoM for each link (expressed as offsets from the link frame)
    linkCoMs << 0,   -0.0256,  0.00652, -0.0472, -0.00402,  0.1263,  0.1,
                -5.844e-9, -0.323,   0.0228,   0.000919, 0.0770, -0.0410, 0.00290,
                -0.391,    0.0706, -0.0701,   0.0287,  -0.376,  -0.0109, 0.0272;

    Matrix4d T_mi = FK(Q, index, isRightArm);
    Matrix3d R_mi = T_mi.block<3,3>(0,0);
    Vector3d joint_position_before_mi = T_mi.block<3,1>(0,3);
    Vector3d r_mi_abs = joint_position_before_mi + R_mi * linkCoMs.col(index); // center of mass position of link i in global frame relative to global frame

    for (int joint_id = 0; joint_id <= index; ++joint_id) {
        Matrix4d Ti = FK(Q, joint_id, isRightArm);
        Vector3d joint_position = Ti.block<3,1>(0,3);
        Vector3d joint_axis = Ti.block<3,1>(0,2); // frame z axis is equivalent to the joint axis vector.
        jacobian.block<3,1>(0, joint_id) = joint_axis.cross(r_mi_abs - joint_position);
        jacobian.block<3,1>(3, joint_id) = joint_axis;
    }

    return jacobian;
}

MatrixXd JacobBody_cg(const VectorXd& Q, int index, bool isRightArm) {
    MatrixXd Jacobian(6, 3);

    // Updated CoM for each link (expressed as offsets from the link frame)
    MatrixXd linkCoMs(3, 7);
    linkCoMs << 0,   -0.0256,  0.00652, -0.0472, -0.00402,  0.1263,  0.1,
                -5.844e-9, -0.323,   0.0228,   0.000919, 0.0770, -0.0410, 0.00290,
                -0.391,    0.0706, -0.0701,   0.0287,  -0.376,  -0.0109, 0.0272;

    Matrix4d T_mi = FK(Q, index, isRightArm);
    Matrix3d R_mi = T_mi.block<3,3>(0,0);
    Vector3d joint_position_before_cg = T_mi.block<3,1>(0,3);
    Vector3d r_mi_abs = joint_position_before_cg + R_mi * linkCoMs.col(index); // center of mass position of link i in global frame

    Vector3d r_mi_abs_rel = r_mi_abs - Vector3d(Q(0), Q(1), 0); // center of mass position of link i relative to mobile base frame Q(0:2) = [x,y,theta]'

    // Nonholonomic matrix setup (assuming linear and angular velocity command scenario)
    MatrixXd nonholonomicMatrix(6, 3);
    nonholonomicMatrix << 1, 0, 0,
                          0, 1, 0,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 1;

    // Calculate the Jacobian for the platform
    Matrix3d crossProdMatrix = crossOperator(r_mi_abs_rel);
    MatrixXd JacobianPlatform(6, 6);
    JacobianPlatform.setZero(); // Initialize with zeros
    JacobianPlatform.block<3, 3>(0, 0) = Matrix3d::Identity();
    JacobianPlatform.block<3, 3>(0, 3) = -crossProdMatrix;
    JacobianPlatform.block<3, 3>(3, 3) = Matrix3d::Identity();

    // Apply the nonholonomic constraints to the platform Jacobian
    Jacobian = JacobianPlatform * nonholonomicMatrix;

    return Jacobian;
}

MatrixXd JacobianWholeBody_cg(const VectorXd& Q, int index, bool isRightArm) {
    MatrixXd J_wb(6, 17);
    J_wb.setZero();
    
    MatrixXd J_arm_cg = JacobArm_cg(Q, index, isRightArm);
    MatrixXd J_body_cg = JacobBody_cg(Q, index, isRightArm);

    J_wb.leftCols(J_body_cg.cols()) = J_body_cg;
    if (isRightArm) {
        J_wb.block(0, J_body_cg.cols(), 6, J_arm_cg.cols()) = J_arm_cg;
    } else {
        J_wb.block(0, J_body_cg.cols() + J_arm_cg.cols(), 6, J_arm_cg.cols()) = J_arm_cg;
    }
    return J_wb;
}

void computeDynamics(const VectorXd& Q, const VectorXd& Qdot, MatrixXd& M, MatrixXd& C, VectorXd& G) {
    // Dynamic parameters
    Vector3d g(0, 0, -9.81);
    double payloadMass = 0.77;

    // Updated mass for each link using VectorXd
    VectorXd linkMasses(7);
    linkMasses << 4.07, 5.996, 0.570, 3.083, 3.616, 1.435, 0.453 + payloadMass;

    // Initialize inertia matrices for each link, approximated and written inline
    MatrixXd linkInertia(3, 21);
    
    linkInertia.block<3, 3>(0, 0) << 4.47e-6, 0, 0, 0, 4.47e-6, -5.61e-10, 0, -5.61e-10, 2.00e-6; // Link 1 inertia
    linkInertia.block<3, 3>(0, 3) << 0.0465, -0.00338, 0.000748, -0.00338, 0.00243, 0.00980, 0.000748, 0.00980, 0.0445;  // Link 2 inertia
    linkInertia.block<3, 3>(0, 6) << 0.0504, 0.00248, -0.0118, 0.00248, 0.0525, 0.00747, -0.0118, 0.00747, 0.00817;  // Link 3 inertia
    linkInertia.block<3, 3>(0, 9) << 0.0308, 0.00732, 0.000194, 0.00732, 0.00278, 0.00101, 0.000194, 0.00101, 0.0329;  // Link 4 inertia
    linkInertia.block<3, 3>(0, 12) << 0.0511, -0.000984, -0.00223, -0.000984, 0.0472, 0.00346, -0.00223, 0.00346, 0.00475;  // Link 5 inertia
    linkInertia.block<3, 3>(0, 15) << 0.00823, 0.00463, -0.00127, 0.00463, 0.0187, 0.00110, -0.00127, 0.00110, 0.0144;  // Link 6 inertia
    linkInertia.block<3, 3>(0, 18) << 0.00378, -3.27e-5, -0.000200, -3.27e-5, 0.00368, -0.000602, -0.000200, -0.000602, 0.000122;  // Link 7 inertia

    // Mass and Inertia parameters for mobile base
    MatrixXd baseMass = MatrixXd::Zero(3, 3);
    baseMass.diagonal() << 50, 50, 25; // Set the diagonal elements to 50, 50, and 25

    // Initialize matrices M, C, and G
    M = MatrixXd::Zero(17, 17);
    C = MatrixXd::Zero(17, 17);
    G = VectorXd::Zero(17);

    for (int arm = 1; arm <= 2; ++arm) {
        VectorXd q_arm = (arm == 1) ? Q.segment(3, 7) : Q.segment(10, 7);
        for (int joint_id = 0; joint_id < 7; ++joint_id) {
            MatrixXd Tcg = FK(Q, joint_id, arm == 1);
            Matrix3d Rcg = Tcg.block<3, 3>(0, 0);
            MatrixXd Jwb_cg = JacobianWholeBody_cg(Q, joint_id, arm == 1);
            double linkmass = linkMasses(joint_id);
            Matrix3d InertiaMatrix = linkInertia.block<3, 3>(0, 3 * (joint_id));

            MatrixXd mass_link = MatrixXd::Zero(6, 6);
            mass_link.block<3, 3>(0, 0) = Matrix3d::Identity() * linkmass;
            mass_link.block<3, 3>(3, 3) = Rcg * InertiaMatrix * Rcg.transpose();

            MatrixXd Jwb_cg_v = Jwb_cg.block<3, 17>(0, 0);

            M += Jwb_cg.transpose() * mass_link * Jwb_cg;
            G -= linkmass * Jwb_cg_v.transpose() * g;
        }
    }

    MatrixXd jacobBase(3, 17);
    jacobBase << Matrix3d::Identity(), MatrixXd::Zero(3, 14);
    M += jacobBase.transpose() * baseMass * jacobBase;
    // C and G adjustments if necessary (not modified in this example)

}

double calculateDistance(const MatrixXd& link1, const MatrixXd& link2) {
    gkFloat dd;
    gkSimplex s;
    gkPolytope lk1, lk2;

    // Allocate and convert Eigen matrix to the format expected by openGJK
    auto convertMatrix = [](const MatrixXd& mat) -> gkFloat** {
        gkFloat** arr = new gkFloat*[mat.cols()];
        for (int i = 0; i < mat.cols(); ++i) {
            arr[i] = new gkFloat[3];
            for (int j = 0; j < 3; ++j) {
                arr[i][j] = static_cast<gkFloat>(mat(j, i));
            }
        }
        return arr;
    };

    gkFloat** gjkLink1 = convertMatrix(link1);
    gkFloat** gjkLink2 = convertMatrix(link2);

    lk1.coord = gjkLink1;
    lk1.numpoints = link1.cols();

    lk2.coord = gjkLink2;
    lk2.numpoints = link2.cols();

    s.nvrtx = 0;

    dd = compute_minimum_distance(lk1, lk2, &s);

    // Free allocated memory
    for (int i = 0; i < link1.cols(); ++i) {
        delete[] gjkLink1[i];
    }
    delete[] gjkLink1;
    for (int i = 0; i < link2.cols(); ++i) {
        delete[] gjkLink2[i];
    }
    delete[] gjkLink2;

    return static_cast<double>(dd);
}

VectorXd CollisionAvoidanceDistances(const VectorXd& Q, bool isRightArm) {
    Vector7d minDistances;
    minDistances.fill(std::numeric_limits<double>::max());

    MatrixXd positions = getJointsPositions(Q, isRightArm);
    MatrixXd positions_other = getJointsPositions(Q, !isRightArm);


    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            MatrixXd link(3, 3);
            link.col(0) = positions.col(i);
            link.col(1) = 0.5 * (positions.col(i) + positions.col(i + 1));
            link.col(2) = positions.col(i + 1);

            MatrixXd link_other(3, 3);
            link_other.col(0) = positions_other.col(j);
            link_other.col(1) = 0.5 * (positions_other.col(j) + positions_other.col(j + 1));
            link_other.col(2) = positions_other.col(j + 1);

            double dist = calculateDistance(link, link_other);
            minDistances[i] = std::min(minDistances[i], dist);
        }
    }

    return minDistances;
}

// Function to compute the gradient of manipulability 
VectorXd SingularityAvoidanceGradient(const VectorXd& Q, bool isRightArm) {
    double epsilon = 1e-6; 
    double k_manipulability = 20;    // gain for the manipulability torque --- avoid high values
    VectorXd gradient(7);

    // Compute the Jacobian(end-effector) and manipulability at q and q_0
    MatrixXd J = JacobArm(Q, 7, isRightArm);

    double manipulability_at_Q = std::sqrt((J * J.transpose()).determinant());
    double manipulability_zero = 1.2;   // the manipulability should be greater than this value to avoid singularity

    k_manipulability = (k_manipulability > 50) ? 50 : k_manipulability; // saturate  gain manipulability to 50
    
    // Compute the potential manipulability at q
    double potential_manipulability_at_Q = manipulability_at_Q <= manipulability_zero
                                           ? k_manipulability*std::pow(manipulability_at_Q - manipulability_zero, 2)
                                           : 0;

    double starting_index = (isRightArm == 1) ? 3 : 10;  // starting index for left or right arm elements

    for (int i = 0; i < 7; ++i) {   // count for all joint number of the right or left arm. 
        // Perturb joint angle by epsilon
        VectorXd Q_epsilon = Q;
        Q_epsilon(i + starting_index) += epsilon;

        // Compute manipulability at perturbed joint state
        MatrixXd J_epsilon = JacobArm(Q_epsilon,7,isRightArm);
        double manipulability_at_Q_epsilon = std::sqrt((J_epsilon * J_epsilon.transpose()).determinant());

        // Compute the potential manipulability at Q_epsilon
        double potential_manipulability_at_Q_epsilon = manipulability_at_Q_epsilon <= manipulability_zero
                                                       ? k_manipulability*std::pow(manipulability_at_Q_epsilon - manipulability_zero, 2)
                                                       : 0;

        // Compute gradient component
        gradient(i) = (potential_manipulability_at_Q_epsilon - potential_manipulability_at_Q) / epsilon;
    }
    return gradient;
}

VectorXd JointLimitPotentialGradient(const VectorXd& Q, bool isRightArm) {
    // Internal parameters
    double delta = 0.1;
    double epsilon = 1e-6;
    double k_qlim = 500;

    // Hard-coded qmax and qmin values for a 7-joint robotic arm
    VectorXd qmax(7);
    qmax << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
    VectorXd qmin(7);
    qmin << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

    VectorXd gradient = VectorXd::Zero(7);
    VectorXd qtilda = VectorXd::Zero(7);

    double starting_index = (isRightArm == 1) ? 3 : 10;  // starting index for left or right arm elements

    // Calculate qtilda for the current q
    for (int i = 0; i <  7; ++i) {
        if (Q(i + starting_index) > qmax(i) - delta) {
            qtilda(i) = Q(i + starting_index) - (qmax(i) - delta);
        } else if (Q(i + starting_index) < qmin(i) + delta) {
            qtilda(i) = Q(i + starting_index) - (qmin(i) + delta);
        }
        // qtilda(i) is 0 when not within delta range of limits
    }

    // Calculate the potential at the current q
    double potential_jointlimit_at_Q = 0.5 * qtilda.transpose() * k_qlim * qtilda;

    // Numerically derive the gradient
    for (int i = 0; i < 7 ; ++i) {
        VectorXd Q_epsilon = Q;
        Q_epsilon(i + starting_index) += epsilon;

        // Calculate qtilda for Q_epsilon
        VectorXd qtilda_epsilon = VectorXd::Zero(7);
        for (int j = 0; j < 7; ++j) {
            if (Q_epsilon(j + starting_index) > qmax(j) - delta) {
                qtilda_epsilon(j) = Q_epsilon(j + starting_index) - (qmax(j) - delta);
            } else if (Q_epsilon(j + starting_index) < qmin(j) + delta) {
                qtilda_epsilon(j) = Q_epsilon(j + starting_index) - (qmin(j) + delta);
            }
             // is 0 when not within delta range of limits
        }

        // Calculate the potential at Q_epsilon
        double potential_jointlimit_at_Q_epsilon = 0.5 * qtilda_epsilon.transpose() * k_qlim * qtilda_epsilon;

        // Compute the gradient component
        gradient(i) = (potential_jointlimit_at_Q_epsilon - potential_jointlimit_at_Q) / epsilon;
    }

    return gradient;
}

VectorXd CollisionAvoidanceGradient(const VectorXd& Q, const bool isRightArm) {
    // Internal parameters
    double epsilon = 1e-6;
    double Fmax = 2500;
    double collisionRadius = 0.23; // Radius for collision avoidance

    VectorXd gradient = VectorXd::Zero(7);

    // Calculate the minimum distances of the current arm links to the other arm links
    VectorXd minDistances = CollisionAvoidanceDistances(Q, isRightArm);

    // Calculate the potential at the current q
    double potential_collision_at_Q = 0;
    for (int i = 0; i < minDistances.size(); ++i) {
        if (minDistances(i) < collisionRadius) {
            potential_collision_at_Q -= Fmax / pow(collisionRadius, 2) / 3 * pow(minDistances(i) - collisionRadius, 3);
        }
    }

    double starting_index = (isRightArm == 1) ? 3 : 10;  // starting index for left or right arm elements
    // Numerically derive the gradient
    for (int i = 0; i < 7; ++i) {
        VectorXd Q_epsilon = Q;
        Q_epsilon(i + starting_index) += epsilon;

        // Calculate minimum distances for Q_epsilon
        VectorXd minDistances_epsilon = CollisionAvoidanceDistances(Q_epsilon, isRightArm);

        double potential_collision_at_Q_epsilon = 0;
        for (int j = 0; j < minDistances_epsilon.size(); ++j) {
            if (minDistances_epsilon(j) < collisionRadius) {
                potential_collision_at_Q_epsilon -= Fmax / pow(collisionRadius, 2) / 3 * pow(minDistances_epsilon(j) - collisionRadius, 3);
            }
        }

        // Compute the gradient component
        gradient(i) = (potential_collision_at_Q_epsilon - potential_collision_at_Q) / epsilon;
    }

    return gradient;
}

VectorXd refAdmitanceCalc(const VectorXd& wrench_base, const VectorXd& pose_base_admitance, const VectorXd& twist_base) {
    double mass = 30.0;
    double inertia = 30.0;
    double linear_stiffness = 5.0;
    double angular_stiffness = 5.0;
    double linear_damping = 3 * std::sqrt(mass * linear_stiffness);
    double angular_damping = 3 * std::sqrt(inertia * angular_stiffness);

    MatrixXd massInertiaMatrix = MatrixXd::Zero(3, 3);
    massInertiaMatrix.diagonal() << mass, mass, inertia;

    MatrixXd stiffnessMatrix = MatrixXd::Zero(3, 3);
    stiffnessMatrix.diagonal() << linear_stiffness, linear_stiffness, angular_stiffness;

    MatrixXd dampingMatrix = MatrixXd::Zero(3, 3);
    dampingMatrix.diagonal() << linear_damping, linear_damping, angular_damping;

    VectorXd p_home(3);
    p_home << 0, 0, 0;
    VectorXd error = p_home - pose_base_admitance;

    VectorXd twist_base_dot = massInertiaMatrix.inverse() * (wrench_base + stiffnessMatrix * error - dampingMatrix * twist_base);
    return twist_base_dot;
}

VectorXd poseNonHolonomic( const VectorXd& pose_base, const VectorXd& twist_base) {
    double orientation = 0;
    if (!(twist_base(0) == 0 && twist_base(1) == 0)) {
        orientation = std::atan2(twist_base(1), twist_base(0));
    }
    VectorXd pose_nonholonomic(3);
    pose_nonholonomic << pose_base(0), pose_base(1), orientation;
    return pose_nonholonomic;
}

VectorXd poseErrorWithRotation(const VectorXd& pose_base_ref, const VectorXd& pose_base) {
    VectorXd e = pose_base_ref - pose_base;
    double theta = pose_base(2);

    MatrixXd R_z(3, 3);
    R_z << std::cos(theta), -std::sin(theta), 0,
           std::sin(theta),  std::cos(theta), 0,
           0,                0,                1;

    VectorXd e_rotated = R_z.transpose() * e;
    return e_rotated;
}

VectorXd baseController(const VectorXd& twist_base_ref, const VectorXd& e_rotated) {
    double kx = 10.0;
    double ky = 10.0;
    double k_theta = 2 * std::sqrt(ky);

    double x_e = e_rotated(0);
    double y_e = e_rotated(1);
    double theta_e = e_rotated(2);

    double v_r = twist_base_ref.head(2).norm(); // Linear velocity command
    double w_r = 0; // Angular velocity command, not directly calculated from twist_base in this example

    double v = v_r * std::cos(theta_e) + kx * x_e;
    double w = w_r + v_r * (ky * y_e + k_theta * std::sin(theta_e));

    VectorXd u(2);
    u << v, w;
    return u;
}

ControlOutput wholeBodyController(const VectorXd& X_task, const VectorXd& Q_null, const VectorXd& F_d, const VectorXd& delta_frc, const VectorXd& Q, const VectorXd& Qdot, const VectorXd& tau_mes, double currentTime, double ratioStiffnessTask, double ratioDampingTask, double ratioStiffnessNull, double ratioDampingNull)
{
    // Define control parameters for task space
    // Right arm control parameters
    double K_right_arm_translation = ratioStiffnessTask * 400; // Stiffness for translation components of right arm (default: 625)
    double K_right_arm_orientation = 16.0; // Stiffness for orientation components of right arm (default: 9.0)
    double D_right_arm_translation = ratioDampingTask * 50.0; // Damping for translation components of right arm (default: 50.0)
    double D_right_arm_orientation = 8.0; // Damping for orientation components of right arm (default: 6.0)

    // Left arm control parameters
    double K_left_arm_translation = ratioStiffnessTask * 400; // Stiffness for translation components of left arm (default: 625)
    double K_left_arm_orientation = 16.0; // Stiffness for orientation components of left arm (default: 9.0)
    double D_left_arm_translation = ratioDampingTask * 50.0; // Damping for translation components of left arm (default: 50)
    double D_left_arm_orientation = 8.0; // Damping for orientation components of left arm (default: 6.0)

    // Combine stiffness and damping for right and left arms into one matrix
    MatrixXd K_task = MatrixXd::Zero(12, 12);
    K_task.block<3, 3>(0, 0) = Matrix3d::Identity() * K_right_arm_translation;
    K_task.block<3, 3>(3, 3) = Matrix3d::Identity() * K_right_arm_orientation;
    K_task.block<3, 3>(6, 6) = Matrix3d::Identity() * K_left_arm_translation;
    K_task.block<3, 3>(9, 9) = Matrix3d::Identity() * K_left_arm_orientation;

    MatrixXd D_task = MatrixXd::Zero(12, 12);
    D_task.block<3, 3>(0, 0) = Matrix3d::Identity() * D_right_arm_translation;
    D_task.block<3, 3>(3, 3) = Matrix3d::Identity() * D_right_arm_orientation;
    D_task.block<3, 3>(6, 6) = Matrix3d::Identity() * D_left_arm_translation;
    D_task.block<3, 3>(9, 9) = Matrix3d::Identity() * D_left_arm_orientation;

    // Base, right arm, and left arm gains
    double K_null_base = 5.0; // Gain for the base (default: 3.0)
    double D_null_base = 80.0; // Damping for the base (default: 60.0)
    double I_null_base = 0.0; // Integrator for the base (default: 2.0)
    double K_null_right_arm = ratioStiffnessNull * 4.0; // Gain for the right arm (default: 1.0)
    double K_null_left_arm = ratioStiffnessNull * 4.0; // Gain for the left arm (default: 1.0)
    double D_null_right_arm = ratioDampingNull * 4.0; // Damping for the right arm (default: 1.0)
    double D_null_left_arm = ratioDampingNull * 4.0; // Damping for the left arm (default: 1.0)
    double I_null_right_arm = 0.0; // Integrator for the right arm (default: 0.0)
    double I_null_left_arm = 0.0; // Integrator for the left arm (default: 0.0)

    // Diagonal matrices for K_null and D_null
    MatrixXd K_null = MatrixXd::Zero(17, 17);
    K_null.diagonal() << VectorXd::Constant(3, K_null_base), VectorXd::Constant(7, K_null_right_arm), VectorXd::Constant(7, K_null_left_arm);
    MatrixXd D_null = MatrixXd::Zero(17, 17);
    D_null.diagonal() << VectorXd::Constant(3, D_null_base), VectorXd::Constant(7, D_null_right_arm), VectorXd::Constant(7, D_null_left_arm);
    MatrixXd I_null = MatrixXd::Zero(17, 17);
    I_null.diagonal() << VectorXd::Constant(3, I_null_base), VectorXd::Constant(7, I_null_right_arm), VectorXd::Constant(7, I_null_left_arm);

    // Compute whole body task-space pose/twist and null-space vecotrs
    VectorXd X_right = pose_arm(Q, 7, true);
    VectorXd X_left = pose_arm(Q, 7, false);
    VectorXd X(12);
    X.segment(0,6) = X_right;
    X.segment(6,6) = X_left;
    // Compute whole body Jacobian for right and left arm
    MatrixXd Jwb_right = JacobianWholeBody(Q, 7, true);
    MatrixXd Jwb_left = JacobianWholeBody(Q, 7, false);
    // Compute Xdot for both arms and concatenate
    VectorXd Xdot_right = Jwb_right * Qdot;
    VectorXd Xdot_left = Jwb_left * Qdot;
    VectorXd Xdot(12), Xdot_task(12);
    Xdot.segment(0,6) = Xdot_right;
    Xdot.segment(6,6) = Xdot_left;
    Xdot_task.setZero();

    // Compute whole body dynamics  model for the model based control.
    MatrixXd M(17, 17);
    MatrixXd C(17, 17);
    VectorXd G(17);
    // Compute whole body dynamics for mobile system without nonholonomic constraint. 
    computeDynamics(Q, Qdot, M, C, G); // Compute dynamics

    // Compute reduced order dynamics
    MatrixXd A(1, 17); // Nonholonomic constraint matrix
    A << -sin(Q(2)), cos(Q(2)), 0, MatrixXd::Zero(1, 14);
    MatrixXd Adot(1, 17); // Derivative of A
    Adot << -Qdot(2) * cos(Q(2)), -Qdot(2) * sin(Q(2)), 0, MatrixXd::Zero(1, 14);

    // Computations for reduced order dynamics with the nonholonimic constraint
    MatrixXd D = MatrixXd::Zero(16, 17);
    D.block(0, 0, 1, 3) << cos(Q(2)), sin(Q(2)), 0;
    D.block(1, 2, 15, 15) = MatrixXd::Identity(15, 15);
    MatrixXd Ddot = MatrixXd::Zero(16, 17);
    Ddot.block(0, 0, 1, 3) << -Qdot(2) * sin(Q(2)), Qdot(2) * cos(Q(2)), 0;

    // Combine A and D into a square matrix
    MatrixXd A_D(17, 17);
    A_D.block(0, 0, 1, 17) << A;
    A_D.block(1, 0, 16, 17) << D;

    // Inverse of A_D
    MatrixXd inverse_A_D = MatrixXd::Zero(17, 17);
    inverse_A_D.setZero();
    inverse_A_D.block(0, 0, 2, 2) << -sin(Q(2)), cos(Q(2)), cos(Q(2)), sin(Q(2));
    inverse_A_D.block(2, 2, 15, 15) = MatrixXd::Identity(15, 15);
    VectorXd E = inverse_A_D.col(0);  // Extract E and F from the inverse of A_D
    MatrixXd F = inverse_A_D.rightCols(16);
    VectorXd v = D * Qdot;  // Pseudo velocities
    // watch this video to learn how to drive these equations:
    //  https://www.youtube.com/watch?v=C1cb03fYkxk

    // Compute reduced order Jacobians and Mass
    MatrixXd Jreduced = MatrixXd::Zero(12, 16);
    Jreduced.block(0, 0, 6, 16) = Jwb_right * F;
    Jreduced.block(6, 0, 6, 16) = Jwb_left * F;
    MatrixXd Jdot_reduced = MatrixXd::Zero(12, 16); // Variable to store the computed rate of change of Jreduced

    // Reduced mass matrix and its inverse
    MatrixXd M_reduced = F.transpose() * M * F;
    MatrixXd M_reduced_inv = M_reduced.inverse();
    MatrixXd JreducedT = Jreduced.transpose();
    MatrixXd lambda = Jreduced * M_reduced_inv * JreducedT;  // auxilary parameter that stors the inverse of Mc
    MatrixXd M_task = (lambda).inverse(); // Compute cartesian mass that decouples the dynamics of null space with task space.
    MatrixXd Jr_plus_M = M_reduced_inv * JreducedT * M_task;  // Compute weighted pseudo-inverse
    // MatrixXd Jr_plus = JreducedT * (Jreduced * JreducedT).inverse(); // Compute pseudo-inverse of Jreduced

    // Define variables
    double deltaTime;
    static double prev_time = -1; // Static variable to store the time of the previous call
    static VectorXd v_wb = v;
    static MatrixXd prev_Jreduced = Jreduced; // Static variable to store Jreduced from the previous call
    static VectorXd tauR_ext = VectorXd::Zero(16);
    static VectorXd F_ext = VectorXd::Zero(12);
    static VectorXd moIntegral = VectorXd::Zero(16);
    static VectorXd residual = VectorXd::Zero(16);
    static MatrixXd prev_M = M;

    if (prev_time >= 0) { // Check if prev_time is initialized
        deltaTime = currentTime - prev_time;
        Jdot_reduced = (Jreduced - prev_Jreduced) / deltaTime;
    }
    else
    {
        Jdot_reduced = MatrixXd::Zero(12, 16);
    }
    prev_Jreduced = Jreduced;  // Update the previous variables for the next call

    // Compute Force Controller in Task Space    
    // Compute Force Control
    VectorXd F_frc_ff(12);
    VectorXd F_pid(12);
    double Kf_p = 0.5;
    double Kf_d = 0.0;
    double Kf_i = 0.0;
    
    F_pid << Kf_p * (F_d - F_ext);
    F_frc_ff << F_pid + F_d + K_task*(X_task - X);
    
    // Zero out elements not in force_dir (assuming setdiff functionality)
    for (int i = 0; i < F_frc_ff.size(); ++i) {
        if (delta_frc(i) <= 0.001)  // for small and negative delta values, force control is disabled. 
        F_frc_ff(i) = 0.0;
    }

    // Contact-loss stabilization
    VectorXd X_tilde = X_task - X; // Compute the error vector
    VectorXd rho_frc = VectorXd::Ones(F_frc_ff.size()); // Initialize rho_frc as a vector of ones (default case)
    
    for (int i = 0; i < F_frc_ff.size(); ++i) {
        if (abs(X_tilde(i)) >= 2 * delta_frc(i)) { // Condition 1: Set rho_frc to 0 where abs(X_tilde) > 2 * delta_frc
            rho_frc(i) = 0;
        }
        else if (delta_frc(i) <= abs(X_tilde(i)) && abs(X_tilde(i)) <= 2 * delta_frc(i)) { // Condition 2: Apply cosine function where 0 < X_tilde <= 2 * delta_frc
            rho_frc(i) = 0.5 * (1 + cos(M_PI * (X_tilde(i) / delta_frc(i) - 1)));
        }
        else
        {
            rho_frc(i) = 1.0;
        }
    }
    
    // Convert to diagonal matrix
    MatrixXd rho_frc_diag = rho_frc.asDiagonal();
    
    // Compute final force control output
    VectorXd F_frc_rho = rho_frc_diag *F_frc_ff;
    VectorXd a_frc_rho = -lambda * (F_frc_rho);
    VectorXd u_frc_rho = M_reduced*Jr_plus_M*(a_frc_rho);

    // Compute null space control torque command
    MatrixXd Nr = MatrixXd::Identity(16, 16) - Jr_plus_M * Jreduced;
    // Compute null space torques:
    VectorXd tau_pid = K_null * (Q_null - Q) - D_null * (Qdot);
    VectorXd tau_qlim(17);
    tau_qlim << 0.0,0.0,0.0,-JointLimitPotentialGradient(Q,true),-JointLimitPotentialGradient(Q,false);  // apply on right and left arm. no influence on the base
    VectorXd tau_sa(17);
    //tau_sa << 0.0,0.0,0.0,-SingularityAvoidanceGradient(Q,true),-SingularityAvoidanceGradient(Q,false);  // apply on right and left arm. no influence on the base
    tau_sa.setZero();
    VectorXd tau_ca(17);
    //tau_ca << 0.0,0.0,0.0,-CollisionAvoidanceGradient(Q,true),-CollisionAvoidanceGradient(Q,false);  // apply on right and left arm. no influence on the base
    tau_ca.setZero();
    VectorXd tau_null = tau_pid + tau_qlim + tau_sa + tau_ca;
    VectorXd u_null = M_reduced * Nr * F.transpose() * ((M).inverse() * tau_null);

    // Compute task space control torque command
    VectorXd F_imp = K_task * (X_task - X) + D_task * (Xdot_task - Xdot);
    VectorXd a_task = lambda * (F_imp);
    VectorXd a_tauExt = -Jreduced*M_reduced_inv*tauR_ext;  // add this to u_task if you want to have disturbance rejection. Note: It doesn't work fine and is close to unstability!
    VectorXd u_task = M_reduced*Jr_plus_M*(a_task - Jdot_reduced*v);

    // Simulating disturbances in x and theta directions
    double gain_x = -0.0; // -18.0 Gain for disturbance in the x direction
    double gain_theta = 0.0; // 14.0 Gain for disturbance in the theta direction
    double now_start = 0.0;  // 7.0

    VectorXd tauR_Ext_virtual = VectorXd::Zero(16); // Initialize tauR_Ext_virtual with zeros
    // Apply disturbance torques in the x and theta directions based on current time and adjustable gains
    tauR_Ext_virtual(0) = (currentTime > now_start && currentTime <= now_start + 2.5) ? gain_x : // Apply +gain_x from 20 to 22.5 seconds as disturbance in x
                (currentTime > now_start + 2.5 && currentTime <= now_start + 5.0) ? -gain_x : 0; // Apply -gain_x from 22.5 to 25 seconds as disturbance in x

    tauR_Ext_virtual(1) = (currentTime > now_start + 7.5 && currentTime <= now_start + 10.0) ? gain_theta : // Apply +gain_theta from 30 to 32.5 seconds as disturbance in theta
    (currentTime > now_start + 10.0 && currentTime <= now_start + 12.5) ? -gain_theta : 0; // Apply -gain_theta from 32.5 to 35 seconds as disturbance in theta

    // Compute whole body torque control command
    VectorXd u = u_task + u_null + u_frc_rho + tauR_Ext_virtual; // tauR_Ext_virtual for demo
    VectorXd tauR_wb = F.transpose() * (C * Qdot - M * (E * Adot + F * Ddot) * F * v) + u;  //tauR_Ext for demo // the robot arm has its own gravity compensation and thus no need to add as feedforward term here.

    // VectorXd vdot = M_reduced_inv * (u); 
    VectorXd vdot = M_reduced_inv * (u); 
    v_wb = v_wb + deltaTime*vdot + 0.005*(v-v_wb);
    
    // Momentum Observer
    double KI = 5.0;
    VectorXd tauR_mes(16);
    tauR_mes << tauR_wb(0), tauR_wb(1), tau_mes.segment(3,14);

    VectorXd tauR_ext_bias(16);
    tauR_ext_bias << 0.0, 0.0, 0.0, -1.5535, -2.1778, 7.9396, 0.2965, -0.7246, -0.5442, 11.2854, -4.5143, 5.4729, 4.8297, 2.1539, -1.1199, 0.2910;
    if (prev_time >= 0.0) { // Check if prev_time is initialized
        MatrixXd Mdot = (M - prev_M) / deltaTime;
        MatrixXd Momentum = M_reduced * v;
        residual = KI * (Momentum - moIntegral);
        moIntegral += deltaTime*(tauR_mes + F.transpose()*(Mdot - (E*Adot + F*Ddot).transpose()*M.transpose())*F*v -F.transpose()*G + residual);
        tauR_ext = residual - tauR_ext_bias;
    }    
    prev_M = M;  // update the previous mass matrix for the next cycle. 

    // Map external toquese to external forces
    MatrixXd J_right = JacobArm(Q,7,true);
    MatrixXd J_left = JacobArm(Q,7,false);
    MatrixXd J_plus_right = J_right.transpose() * (J_right * J_right.transpose()).inverse();
    MatrixXd J_plus_left = J_left.transpose() * (J_left * J_left.transpose()).inverse();
    F_ext <<  J_plus_right.transpose() * tauR_ext.segment(2, 7) , J_plus_left.transpose() * tauR_ext.segment(9, 7);  // F_ext for right/left arm
    
    ControlOutput output;
    output.tauR_wb = tauR_wb;  // for torque based control
    output.v_wb = v_wb; // for velocity based control
    output.tauR_ext = tauR_ext;
    output.F_ext = F_ext;
    prev_time = currentTime;
    
    return output;
}

