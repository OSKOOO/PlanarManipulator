#ifndef LINK_H
#define LINK_H

namespace planarMainpulator{
    struct Link{
        double length;
        double mass;
        // constructor
        Link(double length) : length(length), mass(1) {}
    };
}; // namespace manipulator

#endif // LINK_H