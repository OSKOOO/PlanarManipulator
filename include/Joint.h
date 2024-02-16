#ifndef JOINT_H
#define JOINT_H

#include <iostream>

namespace planarMainpulator {
class Joint {
    public:
        enum Type {
            REVOLUTE, 
            CONTINUOUS,
            PRISMATIC,
        };

    // Constructor for revolute and prismatic joint types
    Joint (Type type, double initPosition, double minLimit, double maxLimit);

    // Constructor for continuous joint type
    Joint (Type type, double initPosition);

    // Setters
    void setPosition(double position);

    // Getters
    double getPosition() const;
    Type getType() const;

    private:
        Type type_;
        double position_;
        double minLimit_;
        double maxLimit_;

}; // class Joint

} // namespace planarMainpulator

#endif // JOINT_H