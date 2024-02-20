#include <gtest/gtest.h>
#include "Manipulator.h" 
#include <cmath>

using namespace planarMainpulator;

class fkTest : public ::testing::Test {
    protected:
        void SetUp() override {
            manipulator_ = std::make_shared<Manipulator>();
        }
        std::shared_ptr<Manipulator> manipulator_;
};


TEST_F(fkTest, getEndEffectorPositionTest ) {

    // Set up an inverted C shape kinematic chain; easy to visualize
    manipulator_->addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, 0.0)); 
    manipulator_->addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, M_PI/2.0));
    manipulator_->addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, M_PI/2.0)); 
    manipulator_->addLink(std::make_shared<Link>(1.0));
    manipulator_->addLink(std::make_shared<Link>(1.0));
    manipulator_->addLink(std::make_shared<Link>(1.0));

    // We know if the robot is in the shape of an inverted C, the end effector position should be (0, 1, pi)
    Eigen::VectorXd expectedPosition(3);
    expectedPosition << 0.0, 1.0, M_PI; 

    Eigen::VectorXd actualPosition = manipulator_->getEndEffectorPosition();

    // Use EXPECT_NEAR due to floating-point arithmetic
    EXPECT_NEAR(expectedPosition(0), actualPosition(0), 1e-5);
    EXPECT_NEAR(expectedPosition(1), actualPosition(1), 1e-5);
    EXPECT_NEAR(expectedPosition(2), actualPosition(2), 1e-5);    
}

TEST_F(fkTest, getEndEffectorPositionTest_EdgeCase ) {
    
    manipulator_->addJoint(std::make_shared<Joint>(Joint::CONTINUOUS,  2*M_PI)); 
    manipulator_->addJoint(std::make_shared<Joint>(Joint::CONTINUOUS,  4*M_PI));
    manipulator_->addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, -6*M_PI)); 
    manipulator_->addLink(std::make_shared<Link>(1.0));
    manipulator_->addLink(std::make_shared<Link>(2.0));
    manipulator_->addLink(std::make_shared<Link>(3.0));

    Eigen::VectorXd expectedPosition(3);
    expectedPosition << 1+2+3, 0.0, 0.0; 

    Eigen::VectorXd actualPosition = manipulator_->getEndEffectorPosition();

    // Use EXPECT_NEAR due to floating-point arithmetic
    EXPECT_NEAR(expectedPosition(0), actualPosition(0), 1e-5);
    EXPECT_NEAR(expectedPosition(1), actualPosition(1), 1e-5);
    EXPECT_NEAR(expectedPosition(2), actualPosition(2), 1e-5);    
}

// TODO: Add another test case of forward kinematics using hand calculations
