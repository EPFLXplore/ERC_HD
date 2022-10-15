#pragma once
#include <ethercatcpp/epos4.h>

#include <string>
#include <vector>

namespace xcontrol {

class Epos4Extended: public ethercatcpp::Epos4 {
public:

    Epos4Extended(bool has_motor);

    void switch_to_enable_op();
    bool get_has_motor();
    int id_;

private:

    bool has_motor_;
};

}