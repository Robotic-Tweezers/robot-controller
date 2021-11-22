#ifndef _CONTROLLER_HPP_
#define _CONTROLLER_HPP_

#include <kinematic.hpp>

namespace robot_tweezers
{
    class TaskSpaceController
    {
        private:

        robot_tweezers::Kinematic robot_kin;

        public:

        TaskSpaceController(float k_p[], float h_v[], float theta[]);
    };
}

#endif // _CONTROLLER_HPP_