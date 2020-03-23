// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*
    Flash ROM emulation

    Explicitly supports:
    Intel 28F016S5 (byte-wide)
    AMD/Fujitsu 29F016 (byte-wide)
    Sharp LH28F400 (word-wide)

    Flash ROMs use a standardized command set across manufacturers,
    so this emulation should work even for non-Intel and non-Sharp chips
    as long as the game doesn't query the maker ID.
*/

#include "emu.h"
#include "intelfsh.h"


//**************************************************************************
//  CONSTANTS
//**************************************************************************

enum
{
	FM_NORMAL,  // normal read/write
	FM_READID,  // read ID
	FM_READSTATUS,  // read status
	FM_WRITEPART1,  // first half of programming, awaiting second
	FM_CLEARPART1,  // first half of clear, awaiting second
	FM_SETMASTER,   // first half of set master lock, awaiting on/off
	FM_READAMDID1,  // part 1 of alt ID sequence
	FM_READAMDID2,  // part 2 of alt ID sequence
	FM_READAMDID3,  // part 3 of alt ID sequence
	FM_ERASEAMD1,   // part 1 of AMD erase sequence
	FM_ERASEAMD2,   // part 2 of AMD erase sequence
	FM_ERASEAMD3,   // part 3 of AMD erase sequence
	FM_ERASEAMD4,   // part 4 of AMD erase sequence
	FM_BYTEPROGRAM,
	FM_BANKSELECT,
	FM_WRITEPAGEATMEL
};


enum
{
	MFG_ALLIANCE = 0x52,
	MFG_AMD = 0x01,
	MFG_AMIC = 0x37,
	MFG_ATMEL = 0x1f,
	MFG_BRIGHT = 0xad,
	MFG_CATALYST = 0x31,
	MFG_EON = 0x1c,
	MFG_FUJITSU = 0x04,
	MFG_GIGADEVICE = 0xc8,
	MFG_HYUNDAI = 0xad,
	MFG_INTEL = 0x89,
	MFG_ISSI = 0xd5,
	MFG_MACRONIX = 0xc2,
	MFG_PANASONIC = 0x32,
	MFG_PMC = 0x9d,
	MFG_SANYO = 0x62,
	MFG_SHARP = 0xb0,
	MFG_SPANSION = 0x01,
	MFG_SST = 0xbf,
	MFG_ST = 0x20,
	MFG_SYNCMOS = 0x40,
	MFG_TI = 0x97,
	MFG_TI_OLD = 0x01,
	MFG_WINBOND_NEX = 0xef,
	MFG_WINBOND = 0xda
};



//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

// device type definition
DEFINE_DEVICE_TYPE(INTEL_28F016S5,        intel_28f016s5_device,        "intel_28f016s5",        "Intel 28F016S5 Flash")
DEFINE_DEVICE_TYPE(SHARP_LH28F016S,       sharp_lh28f016s_device,       "sharp_lh28f016s",       "Sharp LH28F016S Flash")
DEFINE_DEVICE_TYPE(SHARP_LH28F016S_16BIT, sharp_lh28f016s_16bit_device, "sharp_lh28f016s_16bit", "Sharp LH28F016S Flash (16-bit)")
DEFINE_DEVICE_TYPE(ATMEL_29C010,          atmel_29c010_device,          "atmel_29c010",          "Atmel 29C010 Flash")
DEFINE_DEVICE_TYPE(AMD_29F010,            amd_29f010_device,            "amd_29f010",            "AMD 29F010 Flash")
DEFINE_DEVICE_TYPE(AMD_29F040,            amd_29f040_device,            "amd_29f040",            "AMD 29F040 Flash")
DEFINE_DEVICE_TYPE(AMD_29F080,            amd_29f080_device,            "amd_29f080",            "AMD 29F080 Flash")
DEFINE_DEVICE_TYPE(AMD_29F400T,           amd_29f400t_device,           "amd_29f400t",           "AMD 29F400T Flash")
DEFINE_DEVICE_TYPE(AMD_29F800T,           amd_29f800t_device,           "amd_29f800t",           "AMD 29F800T Flash")
DEFINE_DEVICE_TYPE(AMD_29LV200T,          amd_29lv200t_device,          "amd_29lv200t",          "AMD 29LV200T Flash")
DEFINE_DEVICE_TYPE(FUJITSU_29F160T,       fujitsu_29f160t_device,       "fujitsu_29f160t",       "Fujitsu 29F160T Flash")
DEFINE_DEVICE_TYPE(FUJITSU_29F016A,       fujitsu_29f016a_device,       "fujitsu_29f016a",       "Fujitsu 29F016A Flash")
DEFINE_DEVICE_TYPE(FUJITSU_29DL16X,       fujitsu_29dl16x_device,       "fujitsu_29dl16x",       "Fujitsu 29DL16X Flash")
DEFINE_DEVICE_TYPE(INTEL_E28F400B,        intel_e28f400b_device,        "intel_e28f400b",        "Intel E28F400B Flash")
DEFINE_DEVICE_TYPE(MACRONIX_29L001MC,     macronix_29l001mc_device,     "macronix_29l001mc",     "Macronix 29L001MC Flash")
DEFINE_DEVICE_TYPE(MACRONIX_29LV160TMC,   macronix_29lv160tmc_device,   "macronix_29lv160tmc",   "Macronix 29LV160TMC Flash")
DEFINE_DEVICE_TYPE(TMS_29F040,            tms_29f040_device,            "tms_29f040",            "Texas Instruments 29F040 Flash")

DEFINE_DEVICE_TYPE(PANASONIC_MN63F805MNP, panasonic_mn63f805mnp_device, "panasonic_mn63f805mnp", "Panasonic MN63F805MNP Flash")
DEFINE_DEVICE_TYPE(SANYO_LE26FV10N1TS,    sanyo_le26fv10n1ts_device,    "sanyo_le26fv10n1ts",    "Sanyo LE26FV10N1TS Flash")
DEFINE_DEVICE_TYPE(SST_28SF040,           sst_28sf040_device,           "sst_28sf040",           "SST 28SF040 Flash")
DEFINE_DEVICE_TYPE(SST_39VF020,           sst_39vf020_device,           "sst_39vf020",           "SST 39VF020 Flash")

DEFINE_DEVICE_TYPE(SHARP_LH28F400,        sharp_lh28f400_device,        "sharp_lh28f400",        "Sharp LH28F400 Flash")
DEFINE_DEVICE_TYPE(INTEL_E28F008SA,       intel_e28f008sa_device,       "intel_e28f008sa",       "Intel E28F008SA Flash")
DEFINE_DEVICE_TYPE(INTEL_TE28F160,        intel_te28f160_device,        "intel_te28f160",        "Intel TE28F160 Flash")
DEFINE_DEVICE_TYPE(INTEL_TE28F320,        intel_te28f320_device,        "intel_te28f320",        "Intel TE28F320 Flash")
DEFINE_DEVICE_TYPE(SHARP_UNK128MBIT,      sharp_unk128mbit_device,      "sharp_unk128mbit",      "Sharp Unknown 128Mbit Flash")
DEFINE_DEVICE_TYPE(INTEL_28F320J3D,       intel_28f320j3d_device,       "intel_28f320j3d",       "Intel 28F320J3D Flash")
DEFINE_DEVICE_TYPE(INTEL_28F320J5,        intel_28f320j5_device,        "intel_28f320j5",        "Intel 28F320J5 Flash")

DEFINE_DEVICE_TYPE(SST_39VF400A,          sst_39vf400a_device,          "sst_39vf400a",          "SST 39VF400A Flash")

DEFINE_DEVICE_TYPE(ATMEL_49F4096,         atmel_49f4096_device,         "atmel_49f4096",         "Atmel AT49F4096 Flash")


//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

//-------------------------------------------------
//  intelfsh_device - constructor
//-------------------------------------------------

intelfsh_device::intelfsh_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, uint32_t variant)
	: device_t(mconfig, type, tag, owner, clock),
		device_nvram_interface(mconfig, *this),
		m_region(*this, DEVICE_SELF),
		m_type(variant),
		m_size(0),
		m_bits(8),
		m_addrmask(0),
		m_device_id(0),
		m_maker_id(0),
		m_sector_is_4k(false),
		m_sector_is_16k(false),
		m_top_boot_sector(false),
		m_status(0x80),
		m_erase_sector(0),
		m_flash_mode(FM_NORMAL),
		m_flash_master_lock(false),
		m_timer(nullptr),
		m_bank(0)
{
	switch( variant )
	{
	case FLASH_INTEL_28F016S5:
	case FLASH_SHARP_LH28F016S:
		m_bits = 8;
		m_size = 0x200000;
		m_maker_id = MFG_INTEL;
		m_device_id = 0xaa;
		break;
	case FLASH_SHARP_LH28F016S_16BIT:
		m_bits = 16;
		m_size = 0x200000;
		m_maker_id = MFG_INTEL;
		m_device_id = 0xaa;
		break;
	case FLASH_ATMEL_29C010:
		m_bits = 8;
		m_size = 0x20000;
		m_page_size = 0x80;
		m_maker_id = MFG_ATMEL;
		m_device_id = 0xd5;
		break;
	case FLASH_ATMEL_49F4096:
		m_bits = 16;
		m_size = 0x80000;
		m_maker_id = MFG_ATMEL;
		m_device_id = 0x92;
		m_sector_is_16k = true;
		break;
	case FLASH_AMD_29F010:
		m_bits = 8;
		m_size = 0x20000;
		m_maker_id = MFG_AMD;
		m_device_id = 0x20;
		break;
	case FLASH_AMD_29F040:
		m_bits = 8;
		m_size = 0x80000;
		m_maker_id = MFG_AMD;
		m_device_id = 0xa4;
		break;
	case FLASH_AMD_29F080:
		m_bits = 8;
		m_size = 0x100000;
		m_addrmask = 0x7ff;
		m_maker_id = MFG_AMD;
		m_device_id = 0xd5;
		break;
	case FLASH_AMD_29F400T:
		m_bits = 8;
		m_size = 0x80000;
		m_maker_id = MFG_AMD;
		m_device_id = 0x23;
		m_top_boot_sector = true;
		break;
	case FLASH_AMD_29F800T:
		m_bits = 8;
		m_size = 0x100000;
		m_maker_id = MFG_AMD;
		m_device_id = 0xda;
		m_top_boot_sector = true;
		break;
	case FLASH_AMD_29LV200T:
		m_bits = 8;
		m_size = 0x40000;
		m_maker_id = MFG_AMD;
		m_device_id = 0x3b;
		break;
	case FLASH_INTEL_28F320J3D:
		m_bits = 16;
		m_size = 0x400000;
		m_maker_id = MFG_INTEL;
		m_device_id = 0x16;
		m_sector_is_4k = true;
		break;
	case FLASH_INTEL_28F320J5: // funkball
		m_bits = 16;
		m_size = 0x400000;
		m_maker_id = MFG_INTEL;
		m_device_id = 0x14;
//      m_sector_is_4k = true; 128kb?
		break;
	case FLASH_SST_39VF020:
		m_bits = 8;
		m_size = 0x40000;
		m_maker_id = MFG_SST;
		m_device_id = 0xd6;
		m_sector_is_4k = true;
		break;
	case FLASH_SST_39VF400A:
		m_bits = 16;
		m_size = 0x80000;
		m_maker_id = MFG_SST;
		m_device_id = 0xd6;
		m_sector_is_4k = true;
		break;
	case FLASH_SHARP_LH28F400:
		m_bits = 16;
		m_size = 0x80000;
		m_maker_id = MFG_SHARP;
		m_device_id = 0xed;
		break;
	case FLASH_INTEL_E28F400B:
		m_bits = 16;
		m_size = 0x80000;
		m_maker_id = MFG_INTEL;
		m_device_id = 0x4471;
		break;
	case FLASH_FUJITSU_29F160T:
		m_bits = 8;
		m_size = 0x200000;
		m_maker_id = MFG_FUJITSU;
		m_device_id = 0xad;
		m_top_boot_sector = true;
		break;
	case FLASH_FUJITSU_29F016A:
		m_bits = 8;
		m_size = 0x200000;
		m_maker_id = MFG_FUJITSU;
		m_device_id = 0xad;
		break;
	case FLASH_FUJITSU_29DL16X:
		m_bits = 8;
		m_size = 0x200000;
		m_maker_id = MFG_FUJITSU;
		m_device_id = 0x35;
		break;
	case FLASH_INTEL_E28F008SA:
		m_bits = 8;
		m_size = 0x100000;
		m_maker_id = MFG_INTEL;
		m_device_id = 0xa2;
		break;
	case FLASH_INTEL_TE28F160:
		m_bits = 16;
		m_size = 0x200000;
		m_maker_id = MFG_SHARP;
		m_device_id = 0xd0;
		break;
	case FLASH_INTEL_TE28F320:
		m_bits = 16;
		m_size = 0x400000;
		m_maker_id = MFG_INTEL;
		m_device_id = 0x8896;
		break;
	case FLASH_SHARP_UNK128MBIT:
		m_bits = 16;
		m_size = 0x800000;
		m_maker_id = MFG_SHARP;
		m_device_id = 0xb0;
		break;
	case FLASH_MACRONIX_29L001MC:
		m_bits = 8;
		m_size = 0x20000;
		m_maker_id = MFG_MACRONIX;
		m_device_id = 0x51;
		break;
	case FLASH_MACRONIX_29LV160TMC:
		m_bits = 8;
		m_size = 0x20000;
		m_maker_id = MFG_MACRONIX;
		m_device_id = 0x49;
		m_sector_is_16k = true;
		break;
	case FLASH_PANASONIC_MN63F805MNP:
		m_bits = 8;
		m_size = 0x10000;
		m_maker_id = MFG_PANASONIC;
		m_device_id = 0x1b;
		m_sector_is_4k = true;
		break;
	case FLASH_SANYO_LE26FV10N1TS:
		m_bits = 8;
		m_size = 0x20000;
		m_maker_id = MFG_SANYO;
		m_device_id = 0x13;
		m_sector_is_4k = true;
		break;
	case FLASH_SST_28SF040:
		m_bits = 8;
		m_size = 0x80000;
		m_maker_id = MFG_SST;
		m_device_id = 0x04;
		break;
	case FLASH_TMS_29F040:
		m_bits = 8;
		m_addrmask = 0x7fff;
		m_size = 0x80000;
		m_maker_id = MFG_AMD;
		m_device_id = 0xa4;
		break;
	}

	int addrbits;
	for (addrbits = 24; addrbits > 0; addrbits--)
		if ((m_size & (1 << addrbits)) != 0)
			break;
}

intelfsh8_device::intelfsh8_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, uint32_t variant)
	: intelfsh_device(mconfig, type, tag, owner, clock, variant) { }

intelfsh16_device::intelfsh16_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, uint32_t variant)
	: intelfsh_device(mconfig, type, tag, owner, clock, variant) { }


intel_28f016s5_device::intel_28f016s5_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, INTEL_28F016S5, tag, owner, clock, FLASH_INTEL_28F016S5) { }

fujitsu_29f160t_device::fujitsu_29f160t_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, FUJITSU_29F160T, tag, owner, clock, FLASH_FUJITSU_29F160T) { }

fujitsu_29f016a_device::fujitsu_29f016a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, FUJITSU_29F016A, tag, owner, clock, FLASH_FUJITSU_29F016A) { }

fujitsu_29dl16x_device::fujitsu_29dl16x_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, FUJITSU_29DL16X, tag, owner, clock, FLASH_FUJITSU_29DL16X) { }

sharp_lh28f016s_device::sharp_lh28f016s_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, SHARP_LH28F016S, tag, owner, clock, FLASH_SHARP_LH28F016S) { }

sharp_lh28f016s_16bit_device::sharp_lh28f016s_16bit_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh16_device(mconfig, SHARP_LH28F016S_16BIT, tag, owner, clock, FLASH_SHARP_LH28F016S_16BIT) { }

atmel_29c010_device::atmel_29c010_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, ATMEL_29C010, tag, owner, clock, FLASH_ATMEL_29C010) { }

atmel_49f4096_device::atmel_49f4096_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh16_device(mconfig, ATMEL_49F4096, tag, owner, clock, FLASH_ATMEL_49F4096) { }

amd_29f010_device::amd_29f010_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, AMD_29F010, tag, owner, clock, FLASH_AMD_29F010) { }

amd_29f040_device::amd_29f040_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, AMD_29F040, tag, owner, clock, FLASH_AMD_29F040) { }

amd_29f080_device::amd_29f080_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, AMD_29F080, tag, owner, clock, FLASH_AMD_29F080) { }

amd_29f400t_device::amd_29f400t_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, AMD_29F400T, tag, owner, clock, FLASH_AMD_29F400T) { }

amd_29f800t_device::amd_29f800t_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, AMD_29F800T, tag, owner, clock, FLASH_AMD_29F800T) { }

amd_29lv200t_device::amd_29lv200t_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, AMD_29LV200T, tag, owner, clock, FLASH_AMD_29LV200T) { }

intel_e28f008sa_device::intel_e28f008sa_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, INTEL_E28F008SA, tag, owner, clock, FLASH_INTEL_E28F008SA) { }

macronix_29l001mc_device::macronix_29l001mc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, MACRONIX_29L001MC, tag, owner, clock, FLASH_MACRONIX_29L001MC) { }

macronix_29lv160tmc_device::macronix_29lv160tmc_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, MACRONIX_29LV160TMC, tag, owner, clock, FLASH_MACRONIX_29LV160TMC) { }

panasonic_mn63f805mnp_device::panasonic_mn63f805mnp_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, PANASONIC_MN63F805MNP, tag, owner, clock, FLASH_PANASONIC_MN63F805MNP) { }

sanyo_le26fv10n1ts_device::sanyo_le26fv10n1ts_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, SANYO_LE26FV10N1TS, tag, owner, clock, FLASH_SANYO_LE26FV10N1TS) { }

sst_28sf040_device::sst_28sf040_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, SST_28SF040, tag, owner, clock, FLASH_SST_28SF040) { }

sst_39vf020_device::sst_39vf020_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, SST_39VF020, tag, owner, clock, FLASH_SST_39VF020) { }

sharp_lh28f400_device::sharp_lh28f400_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh16_device(mconfig, SHARP_LH28F400, tag, owner, clock, FLASH_SHARP_LH28F400) { }

intel_te28f160_device::intel_te28f160_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh16_device(mconfig, INTEL_TE28F160, tag, owner, clock, FLASH_INTEL_TE28F160) { }

intel_te28f320_device::intel_te28f320_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh16_device(mconfig, INTEL_TE28F320, tag, owner, clock, FLASH_INTEL_TE28F320) { }

intel_e28f400b_device::intel_e28f400b_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh16_device(mconfig, INTEL_E28F400B, tag, owner, clock, FLASH_INTEL_E28F400B) { }

sharp_unk128mbit_device::sharp_unk128mbit_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh16_device(mconfig, SHARP_UNK128MBIT, tag, owner, clock, FLASH_SHARP_UNK128MBIT) { }

intel_28f320j3d_device::intel_28f320j3d_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh16_device(mconfig, INTEL_28F320J3D, tag, owner, clock, FLASH_INTEL_28F320J3D) { }

intel_28f320j5_device::intel_28f320j5_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh16_device(mconfig, INTEL_28F320J5, tag, owner, clock, FLASH_INTEL_28F320J5) { }


sst_39vf400a_device::sst_39vf400a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh16_device(mconfig, SST_39VF400A, tag, owner, clock, FLASH_SST_39VF400A) { }


tms_29f040_device::tms_29f040_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: intelfsh8_device(mconfig, TMS_29F040, tag, owner, clock, FLASH_TMS_29F040) { }

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void intelfsh_device::device_start()
{
	m_data = std::make_unique<uint8_t []>(m_size);
	m_timer = timer_alloc();

	save_item( NAME(m_status) );
	save_item( NAME(m_flash_mode) );
	save_item( NAME(m_flash_master_lock) );
	save_pointer( &m_data[0], "m_data", m_size);
}


//-------------------------------------------------
//  device_timer - handler timer events
//-------------------------------------------------

void intelfsh_device::device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr)
{
	switch( m_flash_mode )
	{
	case FM_READSTATUS:
		m_status = 0x80;
		break;

	case FM_ERASEAMD4:
		m_flash_mode = FM_NORMAL;
		break;
	}
}


//-------------------------------------------------
//  nvram_default - called to initialize NVRAM to
//  its default state
//-------------------------------------------------

void intelfsh_device::nvram_default()
{
	// region always wins
	if (m_region.found())
	{
		uint32_t bytes = m_region->bytes();
		if (bytes > m_size)
			bytes = m_size;

		if (m_bits == 8)
		{
			for (offs_t offs = 0; offs < bytes; offs++)
				m_data[offs] = m_region->as_u8(offs);
		}
		else
		{
			for (offs_t offs = 0; offs < bytes; offs += 2) {
				uint16_t v = m_region->as_u16(offs / 2);
				m_data[offs] = v >> 8;
				m_data[offs+1] = v;
			}
		}
		return;
	}

	// otherwise, default to 0xff
	memset(&m_data[0], 0xff, m_size);
}


//-------------------------------------------------
//  nvram_read - called to read NVRAM from the
//  .nv file
//-------------------------------------------------

void intelfsh_device::nvram_read(emu_file &file)
{
	file.read(&m_data[0], m_size);
}


//-------------------------------------------------
//  nvram_write - called to write NVRAM to the
//  .nv file
//-------------------------------------------------

void intelfsh_device::nvram_write(emu_file &file)
{
	file.write(&m_data[0], m_size);
}


//-------------------------------------------------
//  read_full - generic read, called by the
//  bit-width-specific readers
//-------------------------------------------------

uint32_t intelfsh_device::read_full(uint32_t address)
{
	uint32_t data = 0;
	address += m_bank << 16;
	switch( m_flash_mode )
	{
	default:
	case FM_NORMAL:
		switch( m_bits )
		{
		case 8:
			data = m_data[address];
			break;
		case 16:
			data = (m_data[address*2] << 8) | m_data[address*2+1];
			break;
		}
		break;
	case FM_READSTATUS:
		data = m_status;
		break;
	case FM_READAMDID3:
		if ((m_maker_id == MFG_FUJITSU && m_device_id == 0x35) || (m_maker_id == MFG_AMD && m_device_id == 0x3b))
		{
			// used in Fujitsu 29DL16X 8bits mode
			// used in AMD 29LV200 8bits mode
			switch (address)
			{
				case 0: data = m_maker_id; break;
				case 2: data = m_device_id; break;
				case 4: data = 0; break;
			}
		}
		else
		{
			switch (address)
			{
				case 0: data = m_maker_id; break;
				case 1: data = m_device_id; break;
				case 2: data = 0; break;
			}
		}
		break;
	case FM_READID:
		if (m_maker_id == MFG_INTEL && m_device_id == 0x16)
		{
			switch (address)
			{
				case 0: data = m_maker_id; break;
				case 2: data = m_device_id; break;
				case 4: data = 0; break;
			}
		}
		else
		{
			switch (address)
			{
			case 0: // maker ID
				data = m_maker_id;
				break;
			case 1: // chip ID
				data = m_device_id;
				break;
			case 2: // block lock config
				data = 0; // we don't support this yet
				break;
			case 3: // master lock config
				if (m_flash_master_lock)
				{
					data = 1;
				}
				else
				{
					data = 0;
				}
				break;
			}
		}
		break;
	case FM_ERASEAMD4:
		// reads outside of the erasing sector return normal data
		if ((address < m_erase_sector) || (address >= m_erase_sector+(64*1024)))
		{
			switch( m_bits )
			{
			case 8:
				data = m_data[address];
				break;
			case 16:
				data = (m_data[address*2] << 8) | m_data[address*2+1];
				break;
			}
		}
		else
		{
			m_status ^= ( 1 << 6 ) | ( 1 << 2 );
			data = m_status;
		}
		break;
	}

	//logerror( "intelflash_read( %08x ) %08x\n", address, data );

	return data;
}


//-------------------------------------------------
//  write_full - generic write, called by the
//  bit-width-specific writers
//-------------------------------------------------

void intelfsh_device::write_full(uint32_t address, uint32_t data)
{
	//logerror( "intelflash_write( %u : %08x, %08x )\n", m_flash_mode, address, data );

	address += m_bank << 16;

	switch( m_flash_mode )
	{
	case FM_NORMAL:
	case FM_READSTATUS:
	case FM_READID:
	case FM_READAMDID3:
		switch( data & 0xff )
		{
		case 0xf0:
		case 0xff:  // reset chip mode
			m_flash_mode = FM_NORMAL;
			break;
		case 0x90:  // read ID
			m_flash_mode = FM_READID;
			break;
		case 0x40:
		case 0x10:  // program
			m_flash_mode = FM_WRITEPART1;
			break;
		case 0x50:  // clear status reg
			m_status = 0x80;
			m_flash_mode = FM_READSTATUS;
			break;
		case 0x20:  // block erase
			m_flash_mode = FM_CLEARPART1;
			break;
		case 0x60:  // set master lock
			m_flash_mode = FM_SETMASTER;
			break;
		case 0x70:  // read status
			m_flash_mode = FM_READSTATUS;
			break;
		case 0xaa:  // AMD ID select part 1
			if( ( address & 0xfff ) == 0x555 )
			{
				m_flash_mode = FM_READAMDID1;
			}
			else if( ( address & 0xfff ) == 0xaaa )
			{
				m_flash_mode = FM_READAMDID1;
			}
			break;
		default:
			logerror( "Unknown flash mode byte %x\n", data & 0xff );
			break;
		}
		break;
	case FM_READAMDID1:
		if( ( address & 0xffff ) == 0x2aa && ( data & 0xff ) == 0x55 )
		{
			m_flash_mode = FM_READAMDID2;
		}
		else if( ( address & 0xffff ) == 0x2aaa && ( data & 0xff ) == 0x55 )
		{
			m_flash_mode = FM_READAMDID2;
		}
		else if( ( address & 0xfff ) == 0x555 && ( data & 0xff ) == 0x55 )
		{
			m_flash_mode = FM_READAMDID2;
		}
		// for AMD 29F080 address bits A11-A19 don't care, for TMS 29F040 address bits A15-A18 don't care
		else if( ( address & m_addrmask ) == ( 0xaaaa & m_addrmask ) && ( data & 0xff ) == 0x55 && m_addrmask )
		{
			m_flash_mode = FM_READAMDID2;
		}
		else
		{
			logerror( "unexpected %08x=%02x in FM_READAMDID1\n", address, data & 0xff );
			m_flash_mode = FM_NORMAL;
		}
		break;
	case FM_READAMDID2:
		if( ( address & 0xffff ) == 0x555 && ( data & 0xff ) == 0x90 )
		{
			m_flash_mode = FM_READAMDID3;
		}
		else if( ( address & 0xffff ) == 0x5555 && ( data & 0xff ) == 0x90 )
		{
			m_flash_mode = FM_READAMDID3;
		}
		else if( ( address & 0xfff ) == 0xaaa && ( data & 0xff ) == 0x90 )
		{
			m_flash_mode = FM_READAMDID3;
		}
		else if( ( address & 0xffff ) == 0x555 && ( data & 0xff ) == 0x80 )
		{
			m_flash_mode = FM_ERASEAMD1;
		}
		else if( ( address & 0xffff ) == 0x5555 && ( data & 0xff ) == 0x80 )
		{
			m_flash_mode = FM_ERASEAMD1;
		}
		else if( ( address & 0xfff ) == 0xaaa && ( data & 0xff ) == 0x80 )
		{
			m_flash_mode = FM_ERASEAMD1;
		}
		else if( ( address & 0xffff ) == 0x555 && ( data & 0xff ) == 0xa0 )
		{
			m_flash_mode = FM_BYTEPROGRAM;
		}
		else if( ( address & 0xffff ) == 0x5555 && ( data & 0xff ) == 0xa0 )
		{
			if (m_type == FLASH_ATMEL_29C010)
			{
				m_flash_mode = FM_WRITEPAGEATMEL;
				m_byte_count = 0;
			}
			else
			{
				m_flash_mode = FM_BYTEPROGRAM;
			}
		}
		else if( ( address & 0xfff ) == 0xaaa && ( data & 0xff ) == 0xa0 )
		{
			m_flash_mode = FM_BYTEPROGRAM;
		}
		else if( ( address & 0xffff ) == 0x555 && ( data & 0xff ) == 0xf0 )
		{
			m_flash_mode = FM_NORMAL;
		}
		else if( ( address & 0xffff ) == 0x5555 && ( data & 0xff ) == 0xf0 )
		{
			m_flash_mode = FM_NORMAL;
		}
		else if( ( address & 0xfff ) == 0xaaa && ( data & 0xff ) == 0xf0 )
		{
			m_flash_mode = FM_NORMAL;
		}
		else if( ( address & 0xffff ) == 0x5555 && ( data & 0xff ) == 0xb0 && m_maker_id == 0x62 && m_device_id == 0x13 )
		{
			m_flash_mode = FM_BANKSELECT;
		}

		// for AMD 29F080 address bits A11-A19 don't care, for TMS 29F040 address bits A15-A18 don't care
		else if(( address & m_addrmask ) == ( 0x5555 & m_addrmask ) && ( data & 0xff ) == 0x80 && m_addrmask )
		{
			m_flash_mode = FM_ERASEAMD1;
		}
		else if(( address & m_addrmask ) == ( 0x5555 & m_addrmask ) && ( data & 0xff ) == 0x90 && m_addrmask )
		{
			m_flash_mode = FM_READAMDID3;
		}
		else if(( address & m_addrmask ) == ( 0x5555 & m_addrmask ) && ( data & 0xff ) == 0xa0 && m_addrmask )
		{
			m_flash_mode = FM_BYTEPROGRAM;
		}
		else if(( address & m_addrmask ) == ( 0x5555 & m_addrmask ) && ( data & 0xff ) == 0xf0 && m_addrmask )
		{
			m_flash_mode = FM_NORMAL;
		}
		else
		{
			logerror( "unexpected %08x=%02x in FM_READAMDID2\n", address, data & 0xff );
			m_flash_mode = FM_NORMAL;
		}
		break;
	case FM_ERASEAMD1:
		if( ( address & 0xfff ) == 0x555 && ( data & 0xff ) == 0xaa )
		{
			m_flash_mode = FM_ERASEAMD2;
		}
		else if( ( address & 0xfff ) == 0xaaa && ( data & 0xff ) == 0xaa )
		{
			m_flash_mode = FM_ERASEAMD2;
		}
		else
		{
			logerror( "unexpected %08x=%02x in FM_ERASEAMD1\n", address, data & 0xff );
		}
		break;
	case FM_ERASEAMD2:
		if( ( address & 0xffff ) == 0x2aa && ( data & 0xff ) == 0x55 )
		{
			m_flash_mode = FM_ERASEAMD3;
		}
		else if( ( address & 0xffff ) == 0x2aaa && ( data & 0xff ) == 0x55 )
		{
			m_flash_mode = FM_ERASEAMD3;
		}
		else if( ( address & 0xfff ) == 0x555 && ( data & 0xff ) == 0x55 )
		{
			m_flash_mode = FM_ERASEAMD3;
		}
		else
		{
			logerror( "unexpected %08x=%02x in FM_ERASEAMD2\n", address, data & 0xff );
		}
		break;
	case FM_ERASEAMD3:
		if( (( address & 0xfff ) == 0x555 && ( data & 0xff ) == 0x10 ) ||
			(( address & 0xfff ) == 0xaaa && ( data & 0xff ) == 0x10 ) )
		{
			// chip erase
			memset(&m_data[0], 0xff, m_size);

			m_status = 1 << 3;
			m_flash_mode = FM_ERASEAMD4;

			if (m_sector_is_4k)
			{
				m_timer->adjust( attotime::from_seconds( 1 ) );
			}
			else if(m_sector_is_16k)
			{
				m_timer->adjust( attotime::from_seconds( 4 ) );
			}
			else
			{
				m_timer->adjust( attotime::from_seconds( 16 ) );
			}
		}
		else if( ( data & 0xff ) == 0x30 )
		{
			// sector erase
			// clear the 4k/64k block containing the current address to all 0xffs
			uint32_t base = address * ((m_bits == 16) ? 2 : 1);
			if (m_sector_is_4k)
			{
				memset(&m_data[base & ~0xfff], 0xff, 4 * 1024);
				m_erase_sector = address & ((m_bits == 16) ? ~0x7ff : ~0xfff);
				m_timer->adjust( attotime::from_msec( 125 ) );
			}
			else if(m_sector_is_16k)
			{
				memset(&m_data[base & ~0x3fff], 0xff, 16 * 1024);
				m_erase_sector = address & ((m_bits == 16) ? ~0x1fff : ~0x3fff);
				m_timer->adjust( attotime::from_msec( 500 ) );
			}
			else if(m_top_boot_sector && address >= (m_size - 64*1024))
			{
				if (address >= (m_size - (16*1024)))
				{
					memset(&m_data[base & ~0x3fff], 0xff, 16 * 1024);
					m_erase_sector = address & ((m_bits == 16) ? ~0x1fff : ~0x3fff);
					m_timer->adjust( attotime::from_msec( 500 ) );
				}
				else if (address >= (m_size - (32*1024)))
				{
					memset(&m_data[base & ~0x1fff], 0xff, 8 * 1024);
					m_erase_sector = address & ((m_bits == 16) ? ~0xfff : ~0x1fff);
					m_timer->adjust( attotime::from_msec( 250 ) );
				}
				else
				{
					memset(&m_data[base & ~0x7fff], 0xff, 32 * 1024);
					m_erase_sector = address & ((m_bits == 16) ? ~0x3fff : ~0x7fff);
					m_timer->adjust( attotime::from_msec( 500 ) );
				}
			}
			else
			{
				memset(&m_data[base & ~0xffff], 0xff, 64 * 1024);
				m_erase_sector = address & ((m_bits == 16) ? ~0x7fff : ~0xffff);
				m_timer->adjust( attotime::from_seconds( 1 ) );
			}

			m_status = 1 << 3;
			m_flash_mode = FM_ERASEAMD4;
		}
		else
		{
			logerror( "unexpected %08x=%02x in FM_ERASEAMD3\n", address, data & 0xff );
		}
		break;
	case FM_BYTEPROGRAM:
		switch( m_bits )
		{
		case 8:
			{
				m_data[address] = data;
			}
			break;
		default:
			logerror( "FM_BYTEPROGRAM not supported when m_bits == %d\n", m_bits );
			break;
		}
		m_flash_mode = FM_NORMAL;
		break;
	case FM_WRITEPART1:
		switch( m_bits )
		{
		case 8:
			m_data[address] = data;
			break;
		case 16:
			m_data[address*2] = data >> 8;
			m_data[address*2+1] = data;
			break;
		default:
			logerror( "FM_WRITEPART1 not supported when m_bits == %d\n", m_bits );
			break;
		}
		m_status = 0x80;
		if (m_type == FLASH_SST_28SF040)
			m_flash_mode = FM_NORMAL;
		else
			m_flash_mode = FM_READSTATUS;
		break;
	case FM_WRITEPAGEATMEL:
		switch( m_bits )
		{
		case 8:
			m_data[address] = data;
			break;
		case 16:
			m_data[address*2] = data >> 8;
			m_data[address*2+1] = data;
			break;
		default:
			logerror( "FM_WRITEPAGEATMEL not supported when m_bits == %d\n", m_bits );
			break;
		}

		m_byte_count++;

		if (m_byte_count == m_page_size)
		{
			m_flash_mode = FM_NORMAL;
		}
		break;
	case FM_CLEARPART1:
		if( ( data & 0xff ) == 0xd0 )
		{
			if (m_type == FLASH_SST_28SF040)
			{
				// clear the 256 bytes block containing the current address to all 0xffs
				uint32_t base = address * ((m_bits == 16) ? 2 : 1);
				memset(&m_data[base & ~0xff], 0xff, 256);

				m_timer->adjust( attotime::from_msec( 4 ) );
			}
			else if (m_type == FLASH_INTEL_E28F400B)
			{
				// 00000-03fff -  16KB boot block (may be write protected via external pins)
				// 04000-05fff -   8KB parameter block
				// 06000-07fff -   8KB parameter block
				// 08000-1ffff -  96KB main block
				// 20000-3ffff - 128KB main block
				// 40000-5ffff - 128KB main block
				// 60000-7ffff - 128KB main block
				// erase duration is 0.3s for boot and parameter blocks, and 0.6s for main blocks
				uint32_t base = (address & 0x3ffff) * 2;
				int size, duration;
				if (base < 0x4000)
				{
					base = 0;
					size = 0x4000;
					duration = 300;
				}
				else if (base < 0x8000)
				{
					base &= 0x6000;
					size = 0x2000;
					duration = 300;
				}
				else if (base < 0x20000)
				{
					base = 0x8000;
					size = 0x18000;
					duration = 600;
				}
				else
				{
					base &= 0x60000;
					size = 0x20000;
					duration = 600;
				}

				// clear the block containing the current address to all 0xffffs
				memset(&m_data[base], 0xff, size);

				m_timer->adjust( attotime::from_msec( duration ) );
			}
			else
			{
				// clear the 64k block containing the current address to all 0xffs
				uint32_t base = address * ((m_bits == 16) ? 2 : 1);
				memset(&m_data[base & ~0xffff], 0xff, 64 * 1024);

				m_timer->adjust( attotime::from_seconds( 1 ) );
			}

			m_status = 0x00;
			m_flash_mode = FM_READSTATUS;
			break;
		}
		else
		{
			logerror( "unexpected %02x in FM_CLEARPART1\n", data & 0xff );
		}
		break;
	case FM_SETMASTER:
		switch( data & 0xff )
		{
		case 0xf1:
			m_flash_master_lock = true;
			break;
		case 0xd0:
			m_flash_master_lock = false;
			break;
		default:
			logerror( "unexpected %08x=%02x in FM_SETMASTER:\n", address, data & 0xff );
			break;
		}
		m_flash_mode = FM_NORMAL;
		break;
	case FM_BANKSELECT:
		m_bank = data & 0xff;
		m_flash_mode = FM_NORMAL;
		break;
	}
}
