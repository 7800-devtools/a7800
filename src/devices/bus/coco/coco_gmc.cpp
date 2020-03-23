// license:BSD-3-Clause
// copyright-holders:tim lindner
/***************************************************************************

    coco_gmc.cpp

    Code for emulating the Games Master Cartridge. A banked switched ROM
    cartridge with a SN76489AN programmable sound generator.

    The ROM bank switching is exactly like the circuit developed for RoboCop
    and Predator.

    The SN76489AN is tied to address $FF41.

    Cartridge by John Linville.

***************************************************************************/

#include "emu.h"
#include "coco_pak.h"
#include "sound/sn76496.h"
#include "speaker.h"

#define SN76489AN_TAG   "gmc_psg"

//**************************************************************************
//  TYPE DECLARATIONS
//**************************************************************************

namespace
{
	// ======================> coco_pak_banked_device

	class coco_pak_gmc_device :
		public coco_pak_banked_device
	{
	public:
		// construction/destruction
		coco_pak_gmc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
		virtual void device_add_mconfig(machine_config &config) override;

	protected:
		// device-level overrides
		virtual DECLARE_WRITE8_MEMBER(scs_write) override;

	private:
		required_device<sn76489a_device> m_psg;
	};
};


MACHINE_CONFIG_MEMBER(coco_pak_gmc_device::device_add_mconfig)
	MCFG_SPEAKER_STANDARD_MONO("gmc_speaker")
	MCFG_SOUND_ADD(SN76489AN_TAG, SN76489A, XTAL_4MHz)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "gmc_speaker", 1.0)
MACHINE_CONFIG_END


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(COCO_PAK_GMC, coco_pak_gmc_device, "cocopakgmc", "CoCo Games Master Cartridge")

//-------------------------------------------------
//  coco_pak_device - constructor
//-------------------------------------------------

coco_pak_gmc_device::coco_pak_gmc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: coco_pak_banked_device(mconfig, COCO_PAK_GMC, tag, owner, clock),
		m_psg(*this, SN76489AN_TAG)
{
}


//-------------------------------------------------
//    scs_write
//-------------------------------------------------

WRITE8_MEMBER(coco_pak_gmc_device::scs_write)
{
	switch(offset)
	{
		case 0:
			/* set the bank */
			coco_pak_banked_device::scs_write(space, offset, data, mem_mask);
			break;
		case 1:
			m_psg->write(data);
			break;
	}
}
