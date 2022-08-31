#pragma once
#include "epos4_extended.h"

namespace xcontrol {

/** @brief This class is used to add a arm motor on EtherCAT buss
 */
class OneAxisSlot: public Epos4Extended {
public:
    /**
     * @brief Constructor of ArmMotor class
     */
    OneAxisSlot(bool has_motor, unsigned int manufacturer_id, unsigned int device_model_id);
};

}