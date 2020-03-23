// license:GPL-2.0+
// copyright-holders:Dirk Best
/***************************************************************************

    SVI 318/328 Expander Bus Modules

***************************************************************************/

#include "emu.h"
#include "modules.h"

#include "sv601.h"
#include "sv602.h"
#include "sv603.h"

SLOT_INTERFACE_START( svi_expander_modules )
	SLOT_INTERFACE("sv601", SV601)
	SLOT_INTERFACE("sv602", SV602)
	SLOT_INTERFACE("sv603", SV603)
SLOT_INTERFACE_END
