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

} // namespace planarMainpulator