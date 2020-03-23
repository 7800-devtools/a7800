// license:BSD-3-Clause
// copyright-holders:smf
/**********************************************************************

    Commodore 64 User Port emulation

**********************************************************************

                    GND       1      A       GND
                    +5V       2      B       /FLAG2
                 /RESET       3      C       PB0
                   CNT1       4      D       PB1
                    SP1       5      E       PB2
                   CNT2       6      F       PB3
                    SP2       7      H       PB4
                   /PC2       8      J       PB5
                    ATN       9      K       PB6
                  +9VAC      10      L       PB7
                  +9VAC      11      M       PA2
                    GND      12      N       GND

**********************************************************************/

#ifndef MAME_BUS_C64_USER_H
#define MAME_BUS_C64_USER_H

#pragma once

#include "bus/pet/user.h"

SLOT_INTERFACE_EXTERN( c64_user_port_cards );

#endif // MAME_BUS_C64_USER_H
