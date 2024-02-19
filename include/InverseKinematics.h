#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

// References: 
// Modern Robotics by Kevin M. Lynch and Frank C. Park [R1]
// Robot Modeling and Control by Mark W. Spong, Seth Hutchinson, and M. Vidyasagar [R2]

#include "Manipulator.h"

namespace planarMainpulator
{
    class InverseKinematics
    {
    public:
        InverseKinematics();
        ~InverseKinematics();
        
        // Compute the analytical inverse kinematics solution for the robot
        // Analytical solution is computed using the geometric method by hand [R1]  
        Eigen::VectorXd computeAnalyticalIK(const Eigen::Vector3d& x_desired, Manipulator& manipulator);

        // Compute the numerical inverse kinematics solution for the robot
        // Numerical solution is computed using Newton-Raphson method [R1] [Page 196]
        Eigen::VectorXd computeNumericalIK(const Eigen::Vector3d& x_desired, const Eigen::VectorXd& initialGuess , Manipulator& manipulator);

    private:

    Eigen::MatrixXd computeJacobian(const Eigen::VectorXd& q, Manipulator& manipulator);


    };
} // namespace manipulator

#endif // INVERSEKINEMATICS_H