#ifndef LINK_H
#define LINK_H

namespace planarMainpulator{
    struct Link{
        double length;
        double mass;
      
        /**
         * @brief Construct a new Link object
         * @param length Length of the link
        */
        Link(double length) : length(length), mass(1) {}
    };
}; // namespace manipulator

#endif // LINK_H