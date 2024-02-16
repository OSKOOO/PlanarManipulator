#include "Joint.h"
#include <limits>    // std::numeric_limits
#include <algorithm> // std::clamp


namespace planarMainpulator {

    Joint::Joint (Type type, double initPosition, double minLimit, double maxLimit) 
        : type_(type), position_(initPosition), minLimit_(minLimit), maxLimit_(maxLimit) {
    }
    /***************************************************************************************************/
    /***************************************************************************************************/   
    Joint::Joint (Type type, double initPosition) 
        : type_(type), position_(initPosition), minLimit_(0), maxLimit_(0) {

            //Check if limit exceeds at the time of construction
            if (type_ == REVOLUTE || type_ == PRISMATIC) {
                
                std::cerr << "ERROR: Joint limits are not defined for this joint type" << std::endl;    
                exit(1);
            }
            if(type_ == CONTINUOUS){
                minLimit_ = - std::numeric_limits<double>::infinity();
                maxLimit_ =   std::numeric_limits<double>::infinity();
            }
    }
    /***************************************************************************************************/
    /***************************************************************************************************/
    void Joint::setPosition(double position) {
        if(type_ == CONTINUOUS){
            position_ = position;
        }else{
            position_ = std::clamp(position, minLimit_, maxLimit_);
        }
    }
    /***************************************************************************************************/
    /***************************************************************************************************/
    double Joint::getPosition() const {
        return position_;
    }
    /***************************************************************************************************/
    /***************************************************************************************************/
    Joint::Type Joint::getType() const {
        return type_;
    }
    /***************************************************************************************************/
    /***************************************************************************************************/
    

} // namespace planarMainpulator
