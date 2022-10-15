#include <xcontrol_v2/epos4_extended.h>

using namespace ethercatcpp;

namespace xcontrol {

Epos4Extended::Epos4Extended(bool has_motor) :
    Epos4(),
    has_motor_(has_motor) { }

void Epos4Extended::switch_to_enable_op() {
    if(has_motor_) {
        if (get_Device_State_In_String() == "Switch on disable") {
            set_Device_State_Control_Word(Epos4::shutdown);
        }
        if (get_Device_State_In_String() == "Ready to switch ON") {
            set_Device_State_Control_Word(Epos4::switch_on_and_enable_op);
        }
    }
}

bool Epos4Extended::get_has_motor() {
    return has_motor_;
}

}