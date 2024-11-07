# PlanarManipulator

## Overview
**PlanarManipulator** is a C++ kinematics library for simulating a 3-link planar robotic arm with an object-oriented design. It provides tools for forward and inverse kinematics, workspace analysis, and end-effector positioning, with flexibility to expand to higher-degree-of-freedom manipulators or 3D models.

## Project Structure
```
PlanarManipulator/
├── CMakeLists.txt
├── docs/
│   ├── Instructions.txt           # Build instructions
│   └── Report.txt                 # Project report
├── include/
│   ├── Link.h                     # Defines link properties
│   ├── Joint.h                    # Defines joint properties
│   ├── Transformation.h           # 2D transformation matrix
│   ├── Manipulator.h              # Core robot class
│   └── InverseKinematics.h        # Analytical & numerical IK solvers
├── src/
│   ├── Joint.cpp                  # Joint implementation
│   ├── Manipulator.cpp            # Robot methods & forward kinematics
│   └── InverseKinematics.cpp      # IK solvers
└── tests/
    ├── ManipulatorTest.cpp        # Tests for manipulator methods
    ├── IntersectionTest.cpp       # Intersection tests
    ├── ForwardKinematicsTest.cpp  # Forward kinematics tests
    └── InverseKinematicsTest.cpp  # Inverse kinematics tests
```

## Main functionalities
  PlanarManipulator namespace includes a suite of tools designed for planar robotics 
  manipulator kinematic calculations. The class Manipulator serves as a foundational 
  building block for constructing and interacting with the robot. Users can include
  Manipulator.h to instantiate a robot object. To add links and joints, users can use 
  std::shared_ptr for both Joint and Link types; this is to ensure memory is handled 
  efficiently.

  In addition, the Manipulator class provides forward kinematics calculations using 
  getEndEffectorPosition(), which calculates the end-effector position based on 
  current joint angles. This implementation of forward kinematics is extendable to 
  planar robots with a higher degree-of-freedom. This extendability is handled by efficient 
  calculation of frames using Transformation.h [R2]. The Manipulator can also calculate if a 
  given circle encompasses the end-effector using isInsideCircle().

  The inverse kinematics solution is implemented using the geometric analytical approach, and 
  users can access is by creating an InverseKinematics object and calling computeAnalyticalIK().
  A numerical solver is also implmented using the Newton-Raphson method from [R1]. The method 
  requires computation of the robot jacobian and is calculated from [R2]. 

  Finally, test cases for each of these functionalities are implemented using Googletest. 

## Current limitations:
  Inverse kinematics are only supported for the 3-link planar manipulator arm. That is mainly 
  due to analytical jacobian calculations. For the general case, the developer must consider 
  constructing a jacobian based on the given robot object or using a numerical jacobian. 

## References: 
   - Modern Robotics by Kevin M. Lynch and Frank C. Park [R1]
   - Robot Modeling and Control by Mark W. Spong, Seth Hutchinson, and M. Vidyasagar [R2]

## Contact author:
   - Omar Kolt: omarkolt@hotmail.com
