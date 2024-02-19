#include <gtest/gtest.h>
#include "Manipulator.h"
#include <cmath>

using namespace planarMainpulator;

class intersectionTest : public ::testing::Test {
    protected:
        void SetUp() override {
            manipulator = std::make_shared<Manipulator>();
        }
        std::shared_ptr<Manipulator> manipulator;
};


TEST_F(intersectionTest, isInCircleTest ){

    // Test if the end effector is in a circle
    Manipulator manipulator;
    manipulator.addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, 0.0));
    manipulator.addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, M_PI/2.0));
    manipulator.addJoint(std::make_shared<Joint>(Joint::CONTINUOUS, M_PI/2.0));
    manipulator.addLink(std::make_shared<Link>(1.0));
    manipulator.addLink(std::make_shared<Link>(1.5));
    manipulator.addLink(std::make_shared<Link>(1.0));

    // We know if the robot is in the shape of an inverted C, the end effector position should be (0, 1, pi)
    Eigen::VectorXd expectedPosition(3);
    expectedPosition << 0.0, 1.0, M_PI;

    Eigen::Vector2d circleCenter(0, 0);
    double radius = 1.5;
    bool actualResult = manipulator.isInsideCircle(circleCenter, radius);
    EXPECT_FALSE(actualResult);

    radius = 1.5001;
    actualResult = manipulator.isInsideCircle(circleCenter, radius);
    EXPECT_TRUE(actualResult);

}