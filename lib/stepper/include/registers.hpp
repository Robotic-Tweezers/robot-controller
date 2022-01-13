#ifndef _REGISTERS_HPP_
#define _REGISTERS_HPP_

#include <stdint.h>

namespace RobotTweezers
{
    union global_config_s
    {
        uint8_t bytes[2];
        struct flags
        {
            bool test_mode;
            bool multistep_filt;
            bool mstep_reg_select;
            bool pdn_disable;
            bool index_step;
            bool index_otpw;
            bool shaft;
            bool en_SpreadCycle;
            bool internal_Rsense;
            bool I_scale_analog;
        } flags;
    } global_config_s;
}

#endif // _REGISTERS_HPP_