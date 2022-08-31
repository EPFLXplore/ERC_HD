#ifndef ETHERCATCPP_SOEM_MASTER_PIMPL_H
#define ETHERCATCPP_SOEM_MASTER_PIMPL_H

#define EC_BIG_ENDIAN

#include <ethercatcpp/master.h>
#include <soem/ethercat.h>

namespace ethercatcpp {

class soem_master_impl { // PImpl to use ethercat soem
public:
    // Master contex structure from SOEM lib
    ecx_contextt context_;

    // Construction of soem_master_impl
    soem_master_impl();
    ~soem_master_impl();
};

} // namespace ethercatcpp

#endif // ETHERCATCPP_SOEM_MASTER_PIMPL_H
