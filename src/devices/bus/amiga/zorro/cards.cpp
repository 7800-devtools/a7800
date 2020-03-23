// license:GPL-2.0+
// copyright-holders:Dirk Best
/***************************************************************************

    Amiga Zorro Cards

***************************************************************************/

#include "emu.h"
#include "cards.h"

#include "a2052.h"
#include "a2232.h"
#include "a590.h"
#include "action_replay.h"
#include "buddha.h"


SLOT_INTERFACE_START( a1000_expansion_cards )
SLOT_INTERFACE_END

SLOT_INTERFACE_START( a500_expansion_cards )
	SLOT_INTERFACE("ar1", ACTION_REPLAY_MK1)
	SLOT_INTERFACE("ar2", ACTION_REPLAY_MK2)
	SLOT_INTERFACE("ar3", ACTION_REPLAY_MK3)
	SLOT_INTERFACE("a590", A590)
SLOT_INTERFACE_END

SLOT_INTERFACE_START( a2000_expansion_cards )
	SLOT_INTERFACE("ar1", ACTION_REPLAY_MK1)
	SLOT_INTERFACE("ar2", ACTION_REPLAY_MK2)
	SLOT_INTERFACE("ar3", ACTION_REPLAY_MK3)
SLOT_INTERFACE_END

SLOT_INTERFACE_START( zorro2_cards )
	SLOT_INTERFACE("a2052", A2052)
	SLOT_INTERFACE("a2091", A2091)
	SLOT_INTERFACE("a2232", A2232)
	SLOT_INTERFACE("buddha", BUDDHA)
SLOT_INTERFACE_END

SLOT_INTERFACE_START( zorro3_cards )
	SLOT_INTERFACE("a2052", A2052)
	SLOT_INTERFACE("a2091", A2091)
	SLOT_INTERFACE("a2232", A2232)
	SLOT_INTERFACE("buddha", BUDDHA)
SLOT_INTERFACE_END
