#include <gtest/gtest.h>
#include "Manipulator.h"
#include "InverseKinematics.h"
#include <cmath>

using namespace planarMainpulator;

class ManipulatorTest : public ::testing::Test {
    protected:
        void SetUp() override {
            manipulator = std::make_shared<Manipulator>();
        }
        std::shared_ptr<Manipulator> manipulator;
};

TEST_F(ManipulatorTest, addJointTest) {

    // Simple test to add joints
    auto joint1 = std::make_shared<Joint>(Joint::Type::REVOLUTE, 0.0, -M_PI, M_PI);
    auto joint2 = std::make_shared<Joint>(Joint::Type::PRISMATIC, 0.0, 0.0, 1.0);
    auto joint3 = std::make_shared<Joint>(Joint::Type::CONTINUOUS, 0.0);
    manipulator->addJoint(joint1);
    manipulator->addJoint(joint2);
    manipulator->addJoint(joint3);
    EXPECT_EQ(manipulator->getNumJoints(), 3);
}

TEST_F(ManipulatorTest, addLinkTest) {

    // Simple test to add links
    auto link1 = std::make_shared<Link>(1.0);
    auto link2 = std::make_shared<Link>(1.0);
    auto link3 = std::make_shared<Link>(1.0);
    manipulator->addLink(link1);
    manipulator->addLink(link2);
    manipulator->addLink(link3);
    EXPECT_EQ(manipulator->getNumLinks(), 3);
}

TEST_F(ManipulatorTest, removeLastJointAndLinkTest) {

    // Simple test to remove the last joint and link
    auto joint1 = std::make_shared<Joint>(Joint::Type::REVOLUTE, 0.0, -M_PI, M_PI);
    auto joint2 = std::make_shared<Joint>(Joint::Type::PRISMATIC, 0.0, 0.0, 1.0);
    auto joint3 = std::make_shared<Joint>(Joint::Type::CONTINUOUS, 0.0);
    auto link1 = std::make_shared<Link>(1.0);
    auto link2 = std::make_shared<Link>(1.0);
    auto link3 = std::make_shared<Link>(1.0);
    manipulator->addJoint(joint1);
    manipulator->addJoint(joint2);
    manipulator->addJoint(joint3);
    manipulator->addLink(link1);
    manipulator->addLink(link2);
    manipulator->addLink(link3);
    manipulator->removeLastJointAndLink();
    EXPECT_EQ(manipulator->getNumJoints(), 2);
    EXPECT_EQ(manipulator->getNumLinks(), 2);
    manipulator->removeLastJointAndLink();
    manipulator->removeLastJointAndLink();
    EXPECT_EQ(manipulator->getNumJoints(), 0);
    EXPECT_EQ(manipulator->getNumLinks(), 0);
}






