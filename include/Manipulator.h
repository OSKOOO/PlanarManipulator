#ifndef MANIPULATOR_H   
#define MANIPULATOR_H

#include "Joint.h"
#include "Link.h"
#include <vector>
#include <memory>

using namespace std;

namespace planarMainpulator {

    class Manipulator {
        public:
            Manipulator();
            ~Manipulator();

            void addJoint(shared_ptr<Joint> joint);
            void removeLastJointAndLink();

            void addLink(shared_ptr<Link> link);

            // Getters
            int getEndEffectorLinkIndex() const { return endEffectorLinkIndex_; };
            size_t getNumJoints() const { return joints_.size(); };
            size_t getNumLinks() const { return links_.size(); };

        private:
            vector<shared_ptr<Joint>> joints_;
            vector<shared_ptr<Link>> links_;
            int endEffectorLinkIndex_;
            //TODO: Consider removing these two variables
            int numJoints_;
            int numLinks_;
    }; // class Manipulator
};
#endif // MANIPULATOR_H