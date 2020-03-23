// license:BSD-3-Clause
// copyright-holders:Fabio Priuli
/**********************************************************************

    Megadrive carts

**********************************************************************/

#include "emu.h"
#include "md_carts.h"

#include "rom.h"
#include "svp.h"
#include "sk.h"
#include "ggenie.h"
#include "eeprom.h"
#include "jcart.h"
#include "stm95.h"


SLOT_INTERFACE_START(md_cart)
	SLOT_INTERFACE_INTERNAL("rom",  MD_STD_ROM)
	SLOT_INTERFACE_INTERNAL("rom_svp",  MD_ROM_SVP)
	SLOT_INTERFACE_INTERNAL("rom_sk",  MD_ROM_SK)
// NVRAM handling
	SLOT_INTERFACE_INTERNAL("rom_sram",  MD_ROM_SRAM)
	SLOT_INTERFACE_INTERNAL("rom_sramsafe",  MD_ROM_SRAM)
	SLOT_INTERFACE_INTERNAL("rom_fram",  MD_ROM_FRAM)
	SLOT_INTERFACE_INTERNAL("rom_hardbl95", MD_ROM_SRAM)
	SLOT_INTERFACE_INTERNAL("rom_xinqig",  MD_ROM_SRAM)
	SLOT_INTERFACE_INTERNAL("rom_sf001",  MD_ROM_BEGGARP)
	SLOT_INTERFACE_INTERNAL("rom_sf002",  MD_ROM_WUKONG)
	SLOT_INTERFACE_INTERNAL("rom_sf004",  MD_ROM_STARODYS)
// EEPROM handling (not supported fully yet)
	SLOT_INTERFACE_INTERNAL("rom_eeprom",  MD_STD_EEPROM)
	SLOT_INTERFACE_INTERNAL("rom_nbajam",  MD_EEPROM_NBAJAM)
	SLOT_INTERFACE_INTERNAL("rom_nbajamte",  MD_EEPROM_NBAJAMTE)
	SLOT_INTERFACE_INTERNAL("rom_nflqb96",  MD_EEPROM_NFLQB)
	SLOT_INTERFACE_INTERNAL("rom_cslam",  MD_EEPROM_CSLAM)
	SLOT_INTERFACE_INTERNAL("rom_nhlpa",  MD_EEPROM_NHLPA)
	SLOT_INTERFACE_INTERNAL("rom_blara",  MD_EEPROM_BLARA)
// J-Cart controller (Sampras Tennis)
	SLOT_INTERFACE_INTERNAL("rom_jcart",  MD_JCART)
// J-Cart controller + EEPROM handling (not supported fully yet)
	SLOT_INTERFACE_INTERNAL("rom_codemast",  MD_SEPROM_CODEMAST)
	SLOT_INTERFACE_INTERNAL("rom_mm96",  MD_SEPROM_MM96)
// STM95 EEPROM
	SLOT_INTERFACE_INTERNAL("rom_stm95",  MD_EEPROM_STM95)
// CodeMasters 2-in-1 (reset based)
	SLOT_INTERFACE_INTERNAL("rom_cm2in1",  MD_ROM_CM2IN1)
// Game Genie
	SLOT_INTERFACE_INTERNAL("rom_ggenie",  MD_ROM_GAMEGENIE)
// unique bankswitch
	SLOT_INTERFACE_INTERNAL("rom_ssf2",  MD_ROM_SSF2)
	SLOT_INTERFACE_INTERNAL("rom_radica",  MD_ROM_RADICA)
// pirate mappers (protection and/or bankswitch)
	SLOT_INTERFACE_INTERNAL("rom_16mj2",  MD_ROM_16MJ2)
	SLOT_INTERFACE_INTERNAL("rom_bugs",  MD_ROM_BUGSLIFE)
	SLOT_INTERFACE_INTERNAL("rom_chinf3",  MD_ROM_CHINF3)
	SLOT_INTERFACE_INTERNAL("rom_elfwor",  MD_ROM_ELFWOR)
	SLOT_INTERFACE_INTERNAL("rom_yasech",  MD_ROM_YASECH)
	SLOT_INTERFACE_INTERNAL("rom_kof98",  MD_ROM_KOF98)
	SLOT_INTERFACE_INTERNAL("rom_kof99",  MD_ROM_KOF99)
	SLOT_INTERFACE_INTERNAL("rom_lion2",  MD_ROM_LION2)
	SLOT_INTERFACE_INTERNAL("rom_lion3",  MD_ROM_LION3)
	SLOT_INTERFACE_INTERNAL("rom_mcpir",  MD_ROM_MCPIR)
	SLOT_INTERFACE_INTERNAL("rom_mjlov",  MD_ROM_MJLOV)
	SLOT_INTERFACE_INTERNAL("rom_cjmjclub",  MD_ROM_CJMJCLUB)
	SLOT_INTERFACE_INTERNAL("rom_pokea",  MD_ROM_POKEA)
	SLOT_INTERFACE_INTERNAL("rom_pokestad",  MD_ROM_POKESTAD)
	SLOT_INTERFACE_INTERNAL("rom_realtec",  MD_ROM_REALTEC)
	SLOT_INTERFACE_INTERNAL("rom_redcl",  MD_ROM_REDCL)
	SLOT_INTERFACE_INTERNAL("rom_rx3",  MD_ROM_RX3)
	SLOT_INTERFACE_INTERNAL("rom_sbubl",  MD_ROM_SBUBL)
	SLOT_INTERFACE_INTERNAL("rom_smb",  MD_ROM_SMB)
	SLOT_INTERFACE_INTERNAL("rom_smb2",  MD_ROM_SMB2)
	SLOT_INTERFACE_INTERNAL("rom_smw64",  MD_ROM_SMW64)
	SLOT_INTERFACE_INTERNAL("rom_smouse",  MD_ROM_SMOUSE)
	SLOT_INTERFACE_INTERNAL("rom_soulb",  MD_ROM_SOULB)
	SLOT_INTERFACE_INTERNAL("rom_squir",  MD_ROM_SQUIR)
	SLOT_INTERFACE_INTERNAL("rom_tekkensp",  MD_ROM_TEKKENSP)
	SLOT_INTERFACE_INTERNAL("rom_topf",  MD_ROM_TOPF)


SLOT_INTERFACE_INTERNAL("rom_nbajam_alt",  MD_EEPROM_NBAJAM_ALT)
SLOT_INTERFACE_END
