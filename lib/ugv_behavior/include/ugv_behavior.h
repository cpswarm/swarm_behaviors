#ifndef UGV_BEHAVIOR_H
#define UGV_BEHAVIOR_H

#include "cps_behavior.h"

/**
 * @brief An abstract base class for all behaviors of unmanned ground vehicle (UGV) cyber physical systems (CPSs).
 */
class ugv_behavior : public cps_behavior
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    ugv_behavior ();
};

#endif // UGV_BEHAVIOR_H
