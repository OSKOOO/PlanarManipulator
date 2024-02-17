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
            void removeJoint(shared_ptr<Joint> joint);

            void addLink(shared_ptr<Link> link);
            void removeLink(shared_ptr<Link> link);

            // Getters
            int getEndEffectorLinkIndex() const { return endEffectorLinkIndex_; };
            int getNumJoints() const { return numJoints_; };
            int getNumLinks() const { return numLinks_; };

        private:
            vector<shared_ptr<Joint>> joints_;
            vector<shared_ptr<Link>> links_;
            int endEffectorLinkIndex_;
            int numJoints_;
            int numLinks_;
    }; // class Manipulator
};
#endif // MANIPULATOR_H