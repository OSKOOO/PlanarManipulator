#include "InverseKinematics.h"
#include "Transformation.h"

namespace planarMainpulator
{
    InverseKinematics::InverseKinematics() {}
    InverseKinematics::~InverseKinematics() {}

    Eigen::VectorXd InverseKinematics::computeAnalyticalIK(const Eigen::Vector3d& x_desired, Manipulator& manipulator){
       
        // Analytical solution is computed using the geometric method by hand [R1]  
        if (manipulator.getNumJoints() == 3) {
            // End-effector position
            double Xe = x_desired[0];
            double Ye = x_desired[1];
            double si = x_desired[2]; 

           
            double l1 = manipulator.getLinkLength(0);
            double l2 = manipulator.getLinkLength(1);
            double l3 = manipulator.getLinkLength(2);

            // Compute emd-effector to link 2 distance
            double P2x = Xe - l3 * cos(si);
            double P2y = Ye - l3 * sin(si);

            // Compute th2
            double numerator = pow(P2x, 2) + pow(P2y, 2) - (pow(l1, 2) + pow(l2, 2));
            double denominator = 2 * l1 * l2;
            if (abs(denominator) < 1e-6) {
                throw std::runtime_error("IK: Division by zero or near zero in computing th2");
            }
            double cos_th2 = numerator / denominator;

            double th2 = acos(cos_th2);
            double th1 = atan2(P2y, P2x) - atan2(l2 * sin(th2), l1 + l2 * cos(th2));
            double th3 = si - th1 - th2;

            Eigen::VectorXd result(3);
            result << th1, th2, th3;
            return result;            

        }else{std::cerr << "Analytical solution is not implemented for the given manipulator" << std::endl;}
        
        return Eigen::VectorXd::Zero(3);
    }


} // namespace manipulator