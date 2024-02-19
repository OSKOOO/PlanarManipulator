#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <Eigen/Dense>

class Transformation {
public:
    /**
     * @brief 2D transformation matrix for computing forward kinematic
     * @param theta Angle of rotation
     * @param dx Translation in the x direction
     * @param dy Translation in the y direction
     * @return Eigen::Matrix3d 2D transformation matrix
    */
    static Eigen::Matrix3d transformationMatrix(double theta, double dx, double dy) {
        Eigen::Matrix3d T;
        T << cos(theta), -sin(theta), dx,
             sin(theta),  cos(theta), dy,
             0,            0,          1;
        return T;
    }
};

#endif // TRANSFORMATION_H