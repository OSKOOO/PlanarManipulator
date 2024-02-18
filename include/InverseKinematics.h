#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

// Reference: Modern Robotics by Kevin M. Lynch and Frank C. Park [R1]

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

    private:

    };
} // namespace manipulator

#endif // INVERSEKINEMATICS_H