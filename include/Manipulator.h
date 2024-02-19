#ifndef MANIPULATOR_H   
#define MANIPULATOR_H

#include "Joint.h"
#include "Link.h"
#include "Transformation.h"
#include <vector>
#include <memory>

using namespace std;

namespace planarMainpulator {

    class Manipulator {
        public:
            Manipulator();
            ~Manipulator();

            /**
             * Add a joint to the manipulator
             * @param joint Joint to be added
             */
            void addJoint(shared_ptr<Joint> joint);

            /**
             * Remove the last joint and link from the manipulator
             */
            void removeLastJointAndLink();

            /**
             * Add a link to the manipulator
             * @param link Link to be added
             */
            void addLink(shared_ptr<Link> link);

            // Getters
            int getEndEffectorLinkIndex() const { return endEffectorLinkIndex_; };
            size_t getNumJoints() const { return joints_.size(); };
            size_t getNumLinks() const { return links_.size(); };
            double getJointPosition(int jointIndex) const { return joints_[jointIndex]->getPosition(); };
            double getLinkLength(int linkIndex) const { return links_[linkIndex]->length; };
            shared_ptr<Joint> getJoint(int jointIndex) const { return joints_[jointIndex]; };
            shared_ptr<Link> getLink(int linkIndex) const { return links_[linkIndex]; };            

            // Computes the end effector position based on the forward kinematics of the robot
            Eigen::VectorXd getEndEffectorPosition() const;
            bool isInsideCircle (const Eigen::VectorXd& center, double radius) const;
            
            // Setters
            void setJointPosition(int jointIndex, double position) { joints_[jointIndex]->setPosition(position); };
            void setLinkLength(int linkIndex, double length) { links_[linkIndex]->length = length; };

            // Display end-effector position in the console
            void displayEndEffectorPosition() const;

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

