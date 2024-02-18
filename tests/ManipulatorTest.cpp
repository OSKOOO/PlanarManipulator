#include <gtest/gtest.h>
#include "Manipulator.h"
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

TEST_F(ManipulatorTest, getEndEffectorPositionTest ) {
    Manipulator manipulator;

    // Set up an inverted C shape kinematic chain; easy to visualize
    manipulator.addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, 0.0)); 
    manipulator.addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, M_PI/2.0));
    manipulator.addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, M_PI/2.0)); 
    manipulator.addLink(std::make_shared<Link>(1.0));
    manipulator.addLink(std::make_shared<Link>(1.0));
    manipulator.addLink(std::make_shared<Link>(1.0));

    // We know if the robot is in the shape of an inverted C, the end effector position should be (0, 1, pi)
    Eigen::VectorXd expectedPosition(3);
    expectedPosition << 0.0, 1.0, M_PI; 

    Eigen::VectorXd actualPosition = manipulator.getEndEffectorPosition();

    // Use EXPECT_NEAR due to floating-point arithmetic
    EXPECT_NEAR(expectedPosition(0), actualPosition(0), 1e-5);
    EXPECT_NEAR(expectedPosition(1), actualPosition(1), 1e-5);
    EXPECT_NEAR(expectedPosition(2), actualPosition(2), 1e-5);    
}

TEST_F(ManipulatorTest, getEndEffectorPositionTest_EdgeCase ) {
    Manipulator manipulator;

    
    manipulator.addJoint(std::make_shared<Joint>(Joint::CONTINUOUS,  2*M_PI)); 
    manipulator.addJoint(std::make_shared<Joint>(Joint::CONTINUOUS,  4*M_PI));
    manipulator.addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, -6*M_PI)); 
    manipulator.addLink(std::make_shared<Link>(1.0));
    manipulator.addLink(std::make_shared<Link>(2.0));
    manipulator.addLink(std::make_shared<Link>(3.0));

    Eigen::VectorXd expectedPosition(3);
    expectedPosition << 1+2+3, 0.0, 0.0; 

    Eigen::VectorXd actualPosition = manipulator.getEndEffectorPosition();

    // Use EXPECT_NEAR due to floating-point arithmetic
    EXPECT_NEAR(expectedPosition(0), actualPosition(0), 1e-5);
    EXPECT_NEAR(expectedPosition(1), actualPosition(1), 1e-5);
    EXPECT_NEAR(expectedPosition(2), actualPosition(2), 1e-5);    
}