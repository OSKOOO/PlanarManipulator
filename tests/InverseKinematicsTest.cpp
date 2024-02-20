#include <gtest/gtest.h>
#include "Manipulator.h"
#include "InverseKinematics.h"
#include <cmath>

using namespace planarMainpulator;

class ikTest : public ::testing::Test {
    protected:
        void SetUp() override {
            manipulator_ = std::make_shared<Manipulator>();
        }
        std::shared_ptr<Manipulator> manipulator_;
        InverseKinematics ik;
};

TEST_F(ikTest, analyticalIK_Test){

    // Test the inverse kinematics solution
    manipulator_->addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, 0.0));
    manipulator_->addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, 0.0));
    manipulator_->addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, 0.0));
    manipulator_->addLink(std::make_shared<Link>(1.0));
    manipulator_->addLink(std::make_shared<Link>(1.0));
    manipulator_->addLink(std::make_shared<Link>(1.0));

    Eigen::Vector3d x_desired(0.0, 1.0, M_PI);
    Eigen::VectorXd actualResult = ik.computeAnalyticalIK(x_desired, *manipulator_);
    Eigen::VectorXd expectedResult(3);
    expectedResult << 0.0, M_PI/2.0, M_PI/2.0;
    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(expectedResult(i), actualResult(i), 1e-5);
    }

}

TEST_F(ikTest, numericalIK_Test){
    
        // Test the numerical inverse kinematics solution

        manipulator_->addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, 0.0));
        manipulator_->addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, 0.0));
        manipulator_->addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, 0.0));
        manipulator_->addLink(std::make_shared<Link>(1.0));
        manipulator_->addLink(std::make_shared<Link>(1.0));
        manipulator_->addLink(std::make_shared<Link>(1.0));
    
        Eigen::Vector3d x_desired(0.0, 1.0, M_PI);
        Eigen::VectorXd initialGuess(3);
        initialGuess << 0.0, 0.0, 0.0;
        Eigen::VectorXd actualResult = ik.computeNumericalIK(x_desired, initialGuess, *manipulator_);
        for (int i = 0; i < 3; i++) {
            manipulator_->setJointPosition(i, actualResult(i));
        }
        Eigen::VectorXd result = manipulator_->getEndEffectorPosition();

        for (int i = 0; i < 3; i++) {
            EXPECT_NEAR(x_desired(i), result(i), 1e-5);
        }
}