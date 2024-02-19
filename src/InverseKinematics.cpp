#include "InverseKinematics.h"
#include "Transformation.h"

namespace planarMainpulator
{
    InverseKinematics::InverseKinematics() {}
    InverseKinematics::~InverseKinematics() {}

    /***************************************************************************************************/
    /***************************************************************************************************/    

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
            
            if (abs(denominator) < 1e-6) {throw std::runtime_error("IK: Division by zero or near zero in computing th2");}
            
            double cos_th2 = numerator / denominator;

            double th2 = acos(cos_th2);
            double th1 = atan2(P2y, P2x) - atan2(l2 * sin(th2), l1 + l2 * cos(th2));
            double th3 = si - th1 - th2;

            Eigen::VectorXd result(3);
            result << th1, th2, th3;
            return result;            

        }else{throw std::runtime_error( "IK: Analytical solution is not implemented for the given manipulator" );}
        
        return Eigen::VectorXd::Zero(3);
    }
   
    /***************************************************************************************************/
    /***************************************************************************************************/    
   
    Eigen::VectorXd InverseKinematics::computeNumericalIK(const Eigen::Vector3d& x_desired, const Eigen::VectorXd& initialGuess, Manipulator& manipulator){
        if (manipulator.getNumJoints() == 3) {
        // Numerical solution is computed using Newton-Raphson iterative algorithm [R1] [Page 196]
        // Initial guess for the joint angles
        Eigen::VectorXd Qi = initialGuess;

        for (int i = 0; i < maxIterations_; i++) {
            Eigen::Vector3d x_current = manipulator.getEndEffectorPosition();
            Eigen::Vector3d error = x_desired - x_current;
        
        if (error.norm() < tolerance_) {
            break; // Solution found
            }

        //Compute the Jacobian
        Eigen::MatrixXd J = computeJacobian(Qi, manipulator);
        Eigen::MatrixXd J_pseudo_inverse = J.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::VectorXd delta_Q = J_pseudo_inverse * error;

        // Update the joint angles
        Qi += delta_Q;

        // Update the manipulator configuration
        for (int i = 0; i < Qi.size(); ++i) {
            manipulator.setJointPosition(i, Qi(i));
            }
        if (i == maxIterations_ - 1) {
            std::cerr << "ERROR: IK solver max iteration performed" << std::endl;
            }

        }

    return Qi;
    }else{
        throw std::runtime_error( "IK: Numerical solution is not implemented for the given manipulator" );
    }
    }

    /***************************************************************************************************/
    /***************************************************************************************************/   

    Eigen::MatrixXd InverseKinematics::computeJacobian(const Eigen::VectorXd& q, Manipulator& manipulator){
        // Compute the Jacobian matrix
        // Jacobian matrix is computed using the geometric method [R2] [Page 127]  
        if (q.size() == 3) {
        //Spong implmention
        Eigen::Vector3d L; // Link lengths
        L << manipulator.getLinkLength(0), manipulator.getLinkLength(1), manipulator.getLinkLength(2);
        
        Eigen::MatrixXd J(3, 3);
        Eigen::MatrixXd T_0 = Transformation::transformationMatrix(q(0),0,0);
        Eigen::MatrixXd T_1 = Transformation::transformationMatrix(q(1), L(0), 0);
        Eigen::MatrixXd T_2 = Transformation::transformationMatrix(q(2), L(1), 0);
        Eigen::MatrixXd T_3 = Transformation::transformationMatrix(0.0,  L(2), 0);

        Eigen::Vector3d O_0 = T_0.block(0,2,3,1);
        Eigen::Vector3d O_1 = (T_0 * T_1).block(0,2,3,1);
        Eigen::Vector3d O_2 = (T_0 * T_1 * T_2).block(0,2,3,1);
        Eigen::Vector3d O_3 = (T_0 * T_1 * T_2 * T_3).block(0,2,3,1);        
        
        
        Eigen::Vector3d z;
        z << 0, 0, 1; //Rotation axis
        J.col(0) = z.cross(O_3 - O_0);
        J.col(1) = z.cross(O_3 - O_1);
        J.col(2) = z.cross(O_3 - O_2);
        J.row(2) << 1, 1, 1; //Account for the orientation of the end effector
        
        return J;

        }
        else {
            throw std::runtime_error( "IK: Jacobian computation is not implemented for the given manipulator" );
        }
        return Eigen::MatrixXd::Zero(3, 3);
    }

    /***************************************************************************************************/
    /***************************************************************************************************/             


} // namespace planarMainpulator