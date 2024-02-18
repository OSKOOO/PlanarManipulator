#include "Manipulator.h"

namespace planarMainpulator {
    
    Manipulator::Manipulator() 
        : endEffectorLinkIndex_(-1), numJoints_(0), numLinks_(0) {
    }
   
    /***************************************************************************************************/
    /***************************************************************************************************/
    
    Manipulator::~Manipulator() {
    }
   
    /***************************************************************************************************/
    /***************************************************************************************************/
    
    void Manipulator::addJoint(shared_ptr<Joint> joint) {
        joints_.push_back(joint);
        numJoints_++;
    }
    
    /***************************************************************************************************/
    /***************************************************************************************************/
   
    void Manipulator::addLink(shared_ptr<Link> link) {
        links_.push_back(link);
        numLinks_++;
        endEffectorLinkIndex_ = numLinks_ - 1;
    }
    
    /***************************************************************************************************/   
    /***************************************************************************************************/
    
    void Manipulator::removeLastJointAndLink() {
        if (!joints_.empty()) {
            joints_.pop_back();
        }
        if(!links_.empty()){
            links_.pop_back();
            endEffectorLinkIndex_--;
        }else{
            endEffectorLinkIndex_ = -1;
        }
    }

    /***************************************************************************************************/
    /***************************************************************************************************/ 

    Eigen::VectorXd Manipulator::getEndEffectorPosition() const {
        
        Eigen::VectorXd endEffectorPosition = Eigen::VectorXd::Zero(3);
        double totalTheta = 0.0;
        if (endEffectorLinkIndex_ >= 0) {
            // Compute the forward kinematics of the robot
            Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
            for (int i = 0; i <= endEffectorLinkIndex_+1; i++) {
                if(i == 0){
                    T = T * Transformation::transformationMatrix(joints_[i]->getPosition(), 0.0, 0.0);
                }else if(i > 0 && i <= endEffectorLinkIndex_){
                    T = T * Transformation::transformationMatrix(joints_[i]->getPosition(), links_[i-1]->length, 0.0); //TODO: consider making length and array for x and y coordinates
                }else{
                    T = T * Transformation::transformationMatrix(0.0, links_[i-1]->length, 0.0);
                }
            }
            for (int i = 0; i <= endEffectorLinkIndex_; i++) {
                totalTheta += joints_[i]->getPosition();
            }
            endEffectorPosition << T(0, 2), T(1, 2), totalTheta;
        }
        return endEffectorPosition;
    }

    /***************************************************************************************************/
    /***************************************************************************************************/ 

    void Manipulator::displayEndEffectorPosition() const {
        Eigen::VectorXd endEffectorPosition = getEndEffectorPosition();
        std::cout << "End effector position (x, y, theta_p) = \n" << endEffectorPosition << std::endl;
    }

} // namespace planarMainpulator