#include <xcontrol_v2/three_axis_slot.h>

using namespace ethercatcpp;

namespace xcontrol {

ThreeAxisSlot::ThreeAxisSlot(bool has_motor, unsigned int manufacturer_id, unsigned int device_model_id) : Epos4Extended(has_motor) {
    set_Id("EPOS4", manufacturer_id, device_model_id);
}

}