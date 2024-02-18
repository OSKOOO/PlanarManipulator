#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <Eigen/Dense>

class Transformation {
public:
    // 2D transformation matrix for computing forward kinematics
    static Eigen::Matrix3d transformationMatrix(double theta, double dx, double dy) {
        Eigen::Matrix3d T;
        T << cos(theta), -sin(theta), dx,
             sin(theta),  cos(theta), dy,
             0,            0,          1;
        return T;
    }
};

#endif // TRANSFORMATION_H