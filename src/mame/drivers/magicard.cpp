// license:BSD-3-Clause
// copyright-holders:Angelo Salese, Roberto Fresca, David Haywood
/******************************************************************************

  MAGIC CARD - IMPERA
  -------------------

  Preliminary driver by Roberto Fresca, David Haywood & Angelo Salese


  Games running on this hardware:

  * Magic Card (set 1),        Impera, 199?.
  * Magic Card (set 2),        Impera, 199?.
  * Magic Card (set 3),        Impera, 199?.
  * Magic Card Export 94,      Impera, 1994.
  * Magic Card Jackpot (4.01), Impera, 1998.
  * Magic Lotto Export (5.03), Impera, 2001.
  * Hot Slots (6.00),          Impera, 2002.
  * Quingo Export (5.00),      Impera, 1999.
  * Bel Slots Export (5.01),   Impera, 1999.
  * Big Deal Belgien (5.04),   Impera, 2001.

*******************************************************************************


  *** Hardware Notes ***

  These are actually the specs of the Philips CD-i console.

  Identified:

  - CPU:  1x Philips SCC 68070 CCA84 (16 bits Microprocessor, PLCC) @ 15 MHz
  - VSC:  1x Philips SCC 66470 CAB (Video and System Controller, QFP)

  - Protection: 1x Dallas TimeKey DS1207-1 (for book-keeping protection)

  - Crystals:   1x 30.0000 MHz.
                1x 19.6608 MHz.

  - PLDs:       1x PAL16L8ACN
                1x PALCE18V8H-25


*******************************************************************************


  *** General Notes ***

  Impera released "Magic Card" in a custom 16-bits PCB.
  The hardware was so expensive and they never have reached the expected sales,
  so... they ported the game to Impera/Funworld 8bits boards, losing part of
  graphics and sound/music quality. The new product was named "Magic Card II".


*******************************************************************************

   Magic Card Jackpot 4.01
  (Also Magic Lotto Export)
  -------------------------

  PCB Layout:
   __________________________________________________________________________________________________
  |                                                                                                  |
  |                       SERIAL NUMBER                                                              |___
  |                                                                                                  (A)_|
  |  __     _____             ___________        _________                                            ___|
  | |  |   /     \           |9524 GNN   |      |YMZ284-D |                                           ___|
  | |  |  |BATTERY|          |HM514270AJ8|      |_________|                                           ___|
  | |A |  |  +3V  |          |___________|                                                            ___|
  | |  |  |       |                                       __________________         ________     __  ___|
  | |  |   \_____/            ___________                |                  |       |ULN2803A|   |..| ___|
  | |__|                     |9524 GNN   |               |  MUSIC           |       |________|   |..| ___|
  |                          |HM514270AJ8|               |  TR9C1710-11PCA  |                    |..|(J)_|
  |                          |___________|               |  SA119X/9612     |       _________    |..||
  |    ___    ___                                        |__________________|      |74HC273N |   |..||
  |   |   |  |   |                                                                 |_________|   |..||
  |   |   |  |   |                                                                               |..||
  |   | B |  | B |                                     _________                    _________    |..||
  |   |   |  |   |                                    | 74HC04N |                  |74HC245N |   |__||
  |   |   |  |   |                                    |_________|                  |_________|       |___
  |   |___|  |___|                                                                                   A___|
  |                                                                                                   ___|
  |    _____                  ______________                                                          ___|
  |   |     |                |              |                                        ________         ___|
  |   |     |                |  PHILIPS     |     ___    ___________    ________    |ULN2803A|        ___|
  |   |     | EI79465--A/02  |  SCC66470CAB |    | D |  |PIC16F84-10|  |   E    |   |________|        ___|
  |   |  C  | LPL-CPU V4.0   |  172632=1/2  |    |___|  |___________|  |________|                     ___|
  |   |     | MULTI GAME     |  DfD0032I3   |                                       _________         ___|
  |   |     | 8603186        |              |                                      |74HC273N |        ___|
  |   |     |                |              |                                      |_________|        ___|
  |   |     |                |              |                                                         ___|
  |   |_____|                |______________|                                 _     _________         ___|
  |                                                                          |G|   |74HC245N |        ___|
  |                                                                          |_|   |_________|        ___|
  |   _______   _______                                                                               ___|
  |  |]     [| |IC21   |                                 ______             ___                       ___|
  |  |]  E  [| |       |                                |ALTERA|           |___|                __    ___|
  |  |]  M  [| |       |                                | MAX  |             F      _________  |..|   ___|
  |  |]  P  [| | MAGIC |     ________________           |      |                   |74HC245N | |..|   ___|
  |  |]  T  [| | CARD  |    |                |          |EPM712|                   |_________| |..|   ___|
  |  |]  Y  [| |JACKPOT|    |   PHILIPS      |          |8SQC10|                               |..|   ___|
  |  |]     [| |       |    |  SCC68070CCA84 |          |0-15  |                               |..|   ___|
  |  |]  S  [| |Version|    |  213140-1      |          |______|      X_TAL's                  |__|   ___|
  |  |]  O  [| |   4.01|    |  DfD0103V3     |                      _   _   _                         ___|
  |  |]  C  [| |       |    |                |    ALL RIGHTS       | | | | | |                        ___|
  |  |]  K  [| |Vnr.:  |    |                |    BY  IMPERA       |1| |2| |3|                        ___|
  |  |]  E  [| |11.7.98|    |                |                     |_| |_| |_|                        ___|
  |  |]  T  [| |       |    |                |                                                        ___|
  |  |]     [| |       |    |________________|                                                        ___|
  |  |]_____[| |27C4002|                                                                              ___|
  |  |_______| |_______|                               ___________                                    ___|
  |    ___________________                            |RTC2421 A  |                                   ___|
  |   |   :::::::::::::   |                           |___________|                                  Z___|
  |   |___________________|                                                                          |
  |__________________________________________________________________________________________________|

  Xtal 1: 30.000 MHz.
  Xtal 2:  8.000 MHz.
  Xtal 3: 19.660 MHz.

  A = LT 0030 / LTC695CN / U18708
  B = NEC Japan / D43256BGU-70LL / 0008XD041
  C = MX B9819 / 29F1610MC-12C3 / M25685 / TAIWAN
  D = 24C02C / 24C04 (Serial I2C Bus EEPROM with User-Defined Block Write Protection).
  E = P0030SG / CD40106BCN
  F = 74HCU04D
  G = 74HC74D

  Silkscreened on the solder side:

  LEOTS.
  2800
  AT&S-F0 ML 94V-0

  IMPERA AUSTRIA          -------
  TEL: 0043/7242/27116     V 4.0
  FAX: 0043/7242/27053    -------

*******************************************************************************

  QUINGO EXPORT Version 5.00

  Hardware PCB informations : E179465--A/02 LPL-CPU V4.0/MULTI GAME 8603186

  Eprom type ST M27c4002
  Version 5.00
  vnr 27.07.99 Cksum (provided) 79C5

  Eeprom : 24c04A

  PCB Layout:
   __________________________________________________________________________________________________
  |                                                                                                  |
  |                       SERIAL NUMBER                                                              |___
  |                                                                                                  (A)_|
  |  __     _____             ___________        _________                                            ___|
  | |  |   /     \           |9524 GNN   |      |YMZ284-D |                                           ___|
  | |  |  |BATTERY|          |HM514270AJ8|      |_________|                                           ___|
  | |A |  |  +3V  |          |___________|                                                            ___|
  | |  |  |       |                                       __________________         ________     __  ___|
  | |  |   \_____/            ___________                |                  |       |ULN2803A|   |..| ___|
  | |__|                     |9524 GNN   |               |  MUSIC           |       |________|   |..| ___|
  |                          |HM514270AJ8|               |  TR9C1710-11PCA  |                    |..|(J)_|
  |                          |___________|               |  SA119X/9612     |       _________    |..||
  |    ___    ___                                        |__________________|      |74HC273N |   |..||
  |   |   |  |   |                                                                 |_________|   |..||
  |   |   |  |   |                                                                               |..||
  |   | B |  | B |                                     _________                    _________    |..||
  |   |   |  |   |                                    | 74HC04N |                  |74HC245N |   |__||
  |   |   |  |   |                                    |_________|                  |_________|       |___
  |   |___|  |___|                                                                                   A___|
  |                                                                                                   ___|
  |    _____                  ______________                                                          ___|
  |   |     |                |              |                                        ________         ___|
  |   |     |                |  PHILIPS     |     ___    ___________    ________    |ULN2803A|        ___|
  |   |     | EI79465--A/02  |  SCC66470CAB |    | D |  |PIC16F84-10|  |   E    |   |________|        ___|
  |   |  C  | LPL-CPU V4.0   |  172632=1/2  |    |___|  |___________|  |________|                     ___|
  |   |     | MULTI GAME     |  DfD0032I3   |                                       _________         ___|
  |   |     | 8603186        |              |                                      |74HC273N |        ___|
  |   |     |                |              |                                      |_________|        ___|
  |   |     |                |              |                                                         ___|
  |   |_____|                |______________|                                 _     _________         ___|
  |                                                                          |G|   |74HC245N |        ___|
  |                                                                          |_|   |_________|        ___|
  |   _______   _______                                                                               ___|
  |  |]     [| |IC21   |                                 ______             ___                       ___|
  |  |]  E  [| |       |                                |ALTERA|           |___|                __    ___|
  |  |]  M  [| |       |                                | MAX  |             F      _________  |..|   ___|
  |  |]  P  [| |QUINGO |     ________________           |      |                   |74HC245N | |..|   ___|
  |  |]  T  [| | EXPORT|    |                |          |EPM712|                   |_________| |..|   ___|
  |  |]  Y  [| |       |    |   PHILIPS      |          |8SQC10|                               |..|   ___|
  |  |]     [| |       |    |  SCC68070CCA84 |          |0-15  |                               |..|   ___|
  |  |]  S  [| |Version|    |  213140-1      |          |______|      X_TAL's                  |__|   ___|
  |  |]  O  [| |   5.00|    |  DfD0103V3     |                      _   _   _                         ___|
  |  |]  C  [| |       |    |                |    ALL RIGHTS       | | | | | |                        ___|
  |  |]  K  [| |Vnr.:  |    |                |    BY  IMPERA       |1| |2| |3|                        ___|
  |  |]  E  [| |270799 |    |                |                     |_| |_| |_|                        ___|
  |  |]  T  [| |       |    |                |                                                        ___|
  |  |]     [| |       |    |________________|                                                        ___|
  |  |]_____[| |27C4002|                                                                              ___|
  |  |_______| |_______|                             ___________                                      ___|
  |    ___________________                          |RTC2421 A  |   °°°°°                             ___|
  |   |   :::::::::::::   |                         |___________|    CON5                            Z___|
  |   |___________________|                                                                          |
  |__________________________________________________________________________________________________|

  Xtal 1: 30.000 MHz.
  Xtal 2:  8.000 MHz.
  Xtal 3: 19.660 MHz.

  A = LT 0030 / LTC695CN / U18708
  B = NEC Japan / D43256BGU-70LL / 0008XD041
  C = MX B9819 / 29F1610MC-12C3 / M25685 / TAIWAN
  D = 24C02C / 24C04 (Serial I2C Bus EEPROM with User-Defined Block Write Protection).
  E = P0030SG / CD40106BCN
  F = 74HCU04D
  G = 74HC74D

*******************************************************************************

  BIG DEAL BELGIEN Version 5.04

  Hardware PCB informations : E179465--A/02 LPL-CPU V4.0/MULTI GAME 8603186
  Eprom type ST M27c4002
  Version 5.04
  vnr 21.05.01 Cksum (provided) C4B7

  Eeprom : 24c04A

  PCB Layout:
   __________________________________________________________________________________________________
  |                                                                                                  |
  |                       SERIAL NUMBER                                                              |___
  |                                                                                                  (A)_|
  |  __     _____             ___________        _________                                            ___|
  | |  |   /     \           |9524 GNN   |      |YMZ284-D |                                           ___|
  | |  |  |BATTERY|          |HM514270AJ8|      |_________|                                           ___|
  | |A |  |  +3V  |          |___________|                                                            ___|
  | |  |  |       |                                       __________________         ________     __  ___|
  | |  |   \_____/            ___________                |                  |       |ULN2803A|   |..| ___|
  | |__|                     |9524 GNN   |               |  MUSIC           |       |________|   |..| ___|
  |                          |HM514270AJ8|               |  TR9C1710-11PCA  |                    |..|(J)_|
  |                          |___________|               |  SA119X/9612     |       _________    |..||
  |    ___    ___                                        |__________________|      |74HC273N |   |..||
  |   |   |  |   |                                                                 |_________|   |..||
  |   |   |  |   |                                                                               |..||
  |   | B |  | B |                                     _________                    _________    |..||
  |   |   |  |   |                                    | 74HC04N |                  |74HC245N |   |__||
  |   |   |  |   |                                    |_________|                  |_________|       |___
  |   |___|  |___|                                                                                   A___|
  |                                                                                                   ___|
  |    _____                  ______________                                                          ___|
  |   |     |                |              |                                        ________         ___|
  |   |     |                |  PHILIPS     |     ___    ___________    ________    |ULN2803A|        ___|
  |   |     | EI79465--A/02  |  SCC66470CAB |    | D |  |PIC16F84-10|  |   E    |   |________|        ___|
  |   |  C  | LPL-CPU V4.0   |  172632=1/2  |    |___|  |___________|  |________|                     ___|
  |   |     | MULTI GAME     |  DfD0032I3   |                                       _________         ___|
  |   |     | 8603186        |              |                                      |74HC273N |        ___|
  |   |     |                |              |                                      |_________|        ___|
  |   |     |                |              |                                                         ___|
  |   |_____|                |______________|                                 _     _________         ___|
  |                                                                          |G|   |74HC245N |        ___|
  |                                                                          |_|   |_________|        ___|
  |   _______   _______                                                                               ___|
  |  |]     [| |IC21   |                                 ______             ___                       ___|
  |  |]  E  [| |       |                                |ALTERA|           |___|                __    ___|
  |  |]  M  [| |  BIG  |                                | MAX  |             F      _________  |..|   ___|
  |  |]  P  [| |  DEAL |     ________________           |      |                   |74HC245N | |..|   ___|
  |  |]  T  [| |BELGIEN|    |                |          |EPM712|                   |_________| |..|   ___|
  |  |]  Y  [| |       |    |   PHILIPS      |          |8SQC10|                               |..|   ___|
  |  |]     [| |       |    |  SCC68070CCA84 |          |0-15  |                               |..|   ___|
  |  |]  S  [| |Version|    |  213140-1      |          |______|      X_TAL's                  |__|   ___|
  |  |]  O  [| |   5.04|    |  DfD0103V3     |                      _   _   _                         ___|
  |  |]  C  [| |       |    |                |    ALL RIGHTS       | | | | | |                        ___|
  |  |]  K  [| |Vnr.:  |    |                |    BY  IMPERA       |1| |2| |3|                        ___|
  |  |]  E  [| |210501 |    |                |                     |_| |_| |_|                        ___|
  |  |]  T  [| |       |    |                |                                                        ___|
  |  |]     [| |       |    |________________|                                                        ___|
  |  |]_____[| |27C4002|                                                                              ___|
  |  |_______| |_______|                             ___________                                      ___|
  |    ___________________                          |RTC2421 A  |   °°°°°                             ___|
  |   |   :::::::::::::   |                         |___________|    CON5                            Z___|
  |   |___________________|                                                                          |
  |__________________________________________________________________________________________________|

  Xtal 1: 30.000 MHz.
  Xtal 2:  8.000 MHz.
  Xtal 3: 19.660 MHz.

  A = LT 0030 / LTC695CN / U18708
  B = NEC Japan / D43256BGU-70LL / 0008XD041
  C = MX B9819 / 29F1610MC-12C3 / M25685 / TAIWAN
  D = 24C02C / 24C04 (Serial I2C Bus EEPROM with User-Defined Block Write Protection).
  E = P0030SG / CD40106BCN
  F = 74HCU04D
  G = 74HC74D

*******************************************************************************

  BEL SLOTS EXP. Version 5.01

  Hardware PCB informations : E179465--A/02 LPL-CPU V4.0/MULTI GAME 8603186
  Eprom type ST M27c4002
  Version 5.01
  vnr 01.12.99 Cksum (provided) F718

  Eeprom : 24c04A

  PCB Layout:
   __________________________________________________________________________________________________
  |                                                                                                  |
  |                       SERIAL NUMBER                                                              |___
  |                                                                                                  (A)_|
  |  __     _____             ___________        _________                                            ___|
  | |  |   /     \           |9524 GNN   |      |YMZ284-D |                                           ___|
  | |  |  |BATTERY|          |HM514270AJ8|      |_________|                                           ___|
  | |A |  |  +3V  |          |___________|                                                            ___|
  | |  |  |       |                                       __________________         ________     __  ___|
  | |  |   \_____/            ___________                |                  |       |ULN2803A|   |..| ___|
  | |__|                     |9524 GNN   |               |  MUSIC           |       |________|   |..| ___|
  |                          |HM514270AJ8|               |  TR9C1710-11PCA  |                    |..|(J)_|
  |                          |___________|               |  SA119X/9612     |       _________    |..||
  |    ___    ___                                        |__________________|      |74HC273N |   |..||
  |   |   |  |   |                                                                 |_________|   |..||
  |   |   |  |   |                                                                               |..||
  |   | B |  | B |                                     _________                    _________    |..||
  |   |   |  |   |                                    | 74HC04N |                  |74HC245N |   |__||
  |   |   |  |   |                                    |_________|                  |_________|       |___
  |   |___|  |___|                                                                                   A___|
  |                                                                                                   ___|
  |    _____                  ______________                                                          ___|
  |   |     |                |              |                                        ________         ___|
  |   |     |                |  PHILIPS     |     ___    ___________    ________    |ULN2803A|        ___|
  |   |     | EI79465--A/02  |  SCC66470CAB |    | D |  |PIC16F84-10|  |   E    |   |________|        ___|
  |   |  C  | LPL-CPU V4.0   |  172632=1/2  |    |___|  |___________|  |________|                     ___|
  |   |     | MULTI GAME     |  DfD0032I3   |                                       _________         ___|
  |   |     | 8603186        |              |                                      |74HC273N |        ___|
  |   |     |                |              |                                      |_________|        ___|
  |   |     |                |              |                                                         ___|
  |   |_____|                |______________|                                 _     _________         ___|
  |                                                                          |G|   |74HC245N |        ___|
  |                                                                          |_|   |_________|        ___|
  |   _______   _______                                                                               ___|
  |  |]     [| |IC21   |                                 ______             ___                       ___|
  |  |]  E  [| |       |                                |ALTERA|           |___|                __    ___|
  |  |]  M  [| |  BEL  |                                | MAX  |             F      _________  |..|   ___|
  |  |]  P  [| | SLOTS |     ________________           |      |                   |74HC245N | |..|   ___|
  |  |]  T  [| |  EXP. |    |                |          |EPM712|                   |_________| |..|   ___|
  |  |]  Y  [| |       |    |   PHILIPS      |          |8SQC10|                               |..|   ___|
  |  |]     [| |       |    |  SCC68070CCA84 |          |0-15  |                               |..|   ___|
  |  |]  S  [| |Version|    |  213140-1      |          |______|      X_TAL's                  |__|   ___|
  |  |]  O  [| |   5.01|    |  DfD0103V3     |                      _   _   _                         ___|
  |  |]  C  [| |       |    |                |    ALL RIGHTS       | | | | | |                        ___|
  |  |]  K  [| |Vnr.:  |    |                |    BY  IMPERA       |1| |2| |3|                        ___|
  |  |]  E  [| |011299 |    |                |                     |_| |_| |_|                        ___|
  |  |]  T  [| |       |    |                |                                                        ___|
  |  |]     [| |       |    |________________|                                                        ___|
  |  |]_____[| |27C4002|                                                                              ___|
  |  |_______| |_______|                             ___________                                      ___|
  |    ___________________                          |RTC2421 A  |   °°°°°                             ___|
  |   |   :::::::::::::   |                         |___________|    CON5                            Z___|
  |   |___________________|                                                                          |
  |__________________________________________________________________________________________________|

  Xtal 1: 30.000 MHz.
  Xtal 2:  8.000 MHz.
  Xtal 3: 19.660 MHz.

  A = LT 0030 / LTC695CN / U18708
  B = NEC Japan / D43256BGU-70LL / 0008XD041
  C = MX B9819 / 29F1610MC-12C3 / M25685 / TAIWAN
  D = 24C02C / 24C04 (Serial I2C Bus EEPROM with User-Defined Block Write Protection).
  E = P0030SG / CD40106BCN
  F = 74HCU04D
  G = 74HC74D

*******************************************************************************

  TODO:

  - Proper handling of the 68070 (68k with 32 address lines instead of 24)
    & handle the extra features properly (UART, DMA, timers, etc.)

  - Proper emulation of the 66470 Video Chip (still many unhandled features)

  - Inputs;

  - Many unknown memory maps;

  - Proper memory map and machine driver for magicardj & magicle.
    (different sound chip, extra undumped rom and PIC controller)


*******************************************************************************/

#include "emu.h"
#include "cpu/m68000/m68000.h"
#include "sound/ay8910.h"
#include "sound/saa1099.h"
#include "video/ramdac.h"
#include "screen.h"
#include "speaker.h"


#define CLOCK_A XTAL_30MHz
#define CLOCK_B XTAL_8MHz
#define CLOCK_C XTAL_19_6608MHz


class magicard_state : public driver_device
{
public:
	magicard_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_magicram(*this, "magicram"),
		m_magicramb(*this, "magicramb"),
		m_pcab_vregs(*this, "pcab_vregs"),
		m_scc68070_ext_irqc_regs(*this, "scc_xirqc_regs"),
		m_scc68070_iic_regs(*this, "scc_iic_regs"),
		m_scc68070_uart_regs(*this, "scc_uart_regs"),
		m_scc68070_timer_regs(*this, "scc_timer_regs"),
		m_scc68070_int_irqc_regs(*this, "scc_iirqc_regs"),
		m_scc68070_dma_ch1_regs(*this, "scc_dma1_regs"),
		m_scc68070_dma_ch2_regs(*this, "scc_dma2_regs"),
		m_scc68070_mmu_regs(*this, "scc_mmu_regs"),
		m_maincpu(*this, "maincpu"),
		m_screen(*this, "screen"),
		m_palette(*this, "palette")  { }

	required_shared_ptr<uint16_t> m_magicram;
	required_shared_ptr<uint16_t> m_magicramb;
	required_shared_ptr<uint16_t> m_pcab_vregs;
	required_shared_ptr<uint16_t> m_scc68070_ext_irqc_regs;
	required_shared_ptr<uint16_t> m_scc68070_iic_regs;
	required_shared_ptr<uint16_t> m_scc68070_uart_regs;
	required_shared_ptr<uint16_t> m_scc68070_timer_regs;
	required_shared_ptr<uint16_t> m_scc68070_int_irqc_regs;
	required_shared_ptr<uint16_t> m_scc68070_dma_ch1_regs;
	required_shared_ptr<uint16_t> m_scc68070_dma_ch2_regs;
	required_shared_ptr<uint16_t> m_scc68070_mmu_regs;
	DECLARE_READ16_MEMBER(test_r);
	DECLARE_READ16_MEMBER(philips_66470_r);
	DECLARE_WRITE16_MEMBER(philips_66470_w);
	DECLARE_READ16_MEMBER(scc68070_ext_irqc_r);
	DECLARE_WRITE16_MEMBER(scc68070_ext_irqc_w);
	DECLARE_READ16_MEMBER(scc68070_iic_r);
	DECLARE_WRITE16_MEMBER(scc68070_iic_w);
	DECLARE_READ16_MEMBER(scc68070_uart_r);
	DECLARE_WRITE16_MEMBER(scc68070_uart_w);
	DECLARE_READ16_MEMBER(scc68070_timer_r);
	DECLARE_WRITE16_MEMBER(scc68070_timer_w);
	DECLARE_READ16_MEMBER(scc68070_int_irqc_r);
	DECLARE_WRITE16_MEMBER(scc68070_int_irqc_w);
	DECLARE_READ16_MEMBER(scc68070_dma_ch1_r);
	DECLARE_WRITE16_MEMBER(scc68070_dma_ch1_w);
	DECLARE_READ16_MEMBER(scc68070_dma_ch2_r);
	DECLARE_WRITE16_MEMBER(scc68070_dma_ch2_w);
	DECLARE_READ16_MEMBER(scc68070_mmu_r);
	DECLARE_WRITE16_MEMBER(scc68070_mmu_w);
	DECLARE_DRIVER_INIT(magicard);
	virtual void machine_reset() override;
	virtual void video_start() override;
	uint32_t screen_update_magicard(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	INTERRUPT_GEN_MEMBER(magicard_irq);
	required_device<cpu_device> m_maincpu;
	required_device<screen_device> m_screen;
	required_device<palette_device> m_palette;
};


/*************************
*     Video Hardware     *
*************************/

/*
66470
video and system controller
19901219/wjvg
*/
/*
TODO: check this register,doesn't seem to be 100% correct.
1fffe0  csr = control and status register
    w 00...... ........ DM = slow timing speed, normal dram mode
    w 01...... ........ DM = fast timing speed, page dram mode
    w 10...... ........ DM = fast timing speed, nibble dram mode
    w 11...... ........ DM = slow timing speed, dual-port vram mode
    w ..1..... ........ TD = 256/64 k dram's
    w ...1.... ........ CG = enable character generator
    w ....1... ........ DD = rom data acknowledge delay
    w .....1.. ........ ED = early dtack
    w ......0. ........ not used
    w .......1 ........ BE  = enable bus error (watchdog timer)
   r  ........ 1....... DA  = vertical display active
   r  ........ .1...... FG  = set during frame grabbing (if fg in dcr set)
   r  ........ ..xxx... not used
   r  ........ .....1.. IT2 = intn active
   r  ........ ......1. IT1 = pixac free and intn active
   r  ........ .......1 BE  = bus error generated by watchdog timer
*/

/*63 at post test,6d all the time.*/
#define SCC_CSR_VREG    (m_pcab_vregs[0x00/2] & 0xffff)
#define SCC_CG_VREG     ((SCC_CSR_VREG & 0x10)>>4)

/*
1fffe2  dcr = display command register
    w 1....... ........  DE = enable display
    w .00..... ........  CF = 20   MHz (or 19.6608 MHz)
    w .01..... ........  CF = 24   MHz
    w .10..... ........  CF = 28.5 MHz
    w .11..... ........  CF = 30   MHz
    w ...1.... ........  FD = 60/50 Hz frame duration
    w ....00.. ........  SM/SS = non-interlaced scan mode
    w ....01.. ........  SM/SS = double frequency scan mode
    w ....10.. ........  SM/SS = interlaced scan mode
    w ....11.. ........  SM/SS = interlaced field repeat scan mode
    w ......1. ........  LS = full screen/border
    w .......1 ........  CM = logical/physical screen
    w ........ 1.......  FG = 4/8 bits per pixel
    w ........ .1......  DF = enable frame grabbing
    w ........ ..00....  IC/DC = ICA and DCA inactive
    w ........ ..01....  IC/DC = ICA active, reduced DCA mode (DCA sz=16 byts)
    w ........ ..10....  IC/DC = ICA active, DCA inactive
    w ........ ..11....  IC/DC = ICA active, DCA active (DCA size=64 bytes)
    w ........ ....aaaa  VSR:H = video start address (MSB's)
*/

#define SCC_DCR_VREG    (m_pcab_vregs[0x02/2] & 0xffff)
#define SCC_DE_VREG     ((SCC_DCR_VREG & 0x8000)>>15)
#define SCC_FG_VREG     ((SCC_DCR_VREG & 0x0080)>>7)
#define SCC_VSR_VREG_H  ((SCC_DCR_VREG & 0xf)>>0)

/*
1fffe4  vsr = video start register
    w aaaaaaaa aaaaaaaa  VSR:L = video start address (LSB's)
*/

#define SCC_VSR_VREG_L  (m_pcab_vregs[0x04/2] & 0xffff)
#define SCC_VSR_VREG    ((SCC_VSR_VREG_H)<<16) | (SCC_VSR_VREG_L)

/*
1fffe6  bcr = border colour register
    w ........ nnnnnnnn  in 8 bit mode
    w ........ nnnn....  in 4 bit mode
*/
/*
(Note: not present on the original vreg listing)
1fffe8 dcr2 = display command register 2
    w x....... ........  not used
    w .nn..... ........  OM = lower port of the video mode (with CM)
    w ...1.... ........  ID = Indipendent DCA bit
    w ....nn.. ........  MF = Mosaic Factor (2,4,8,16)
    w ......nn ........  FT = File Type (0/1 = bitmap, 2 = RLE, 3 = Mosaic)
    w ........ xxxx....  not used
    w ........ ....aaaa  "data" (dunno the purpose...)
*/
#define SCC_DCR2_VREG  (m_pcab_vregs[0x08/2] & 0xffff)

/*
(Note: not present on the original vreg listing)
1fffea dcp = ???
    w aaaaaaaa aaaaaa--  "data" (dunno the purpose...)
    w -------- ------xx not used
*/

/*
1fffec  swm = selective write mask register
    w nnnnnnnn ........  mask
*/
/*
1fffee  stm = selective mask register
    w ........ nnnnnnnn  mask
*/
/*
1ffff0  a = source register a
    w nnnnnnnn nnnnnnnn  source
*/
#define SCC_SRCA_VREG  (m_pcab_vregs[0x10/2] & 0xffff)

/*
1ffff2  b = destination register b
   rw nnnnnnnn nnnnnnnn  destination
*/

#define SCC_DSTB_VREG  (m_pcab_vregs[0x12/2] & 0xffff)

/*
1ffff4  pcr = pixac command register
    w 1....... ........  4N  = 8/4 bits per pixel
    w .1....00 ....x00.  COL = enable colour2 function
    w .1....00 .....01.  COL = enable colour1 function
    w .1...0.. .....10.  COL = enable bcolour2 function
    w .1...0.. .....11.  COL = enable bcolour1 function
    w ..1..000 ....x00.  EXC = enable exchange function
    w ..1..000 .....01.  EXC = enable swap function
    w ..1..000 .....10.  EXC = enable inverted exchange function
    w ..1..000 .....11.  EXC = enable inverted swap function
    w ...1..0. ....x00.  CPY = enable copy type b function
    w ...1...0 ....x10.  CPY = enable copy type a function
    w ...1..0. .....01.  CPY = enable patch type b function
    w ...1...0 .....11.  CPY = enable patch type a function
    w ....1000 .....00.  CMP = enable compare function
    w ....1000 .....10.  CMP = enable compact function
    w .....1.. ........  RTL = manipulate right to left
    w ......1. ........  SHK = shrink picture by factor 2
    w .......1 ........  ZOM = zoom picture by factor 2
    w ........ nnnn....  LGF = logical function
    w ........ 0000....  LGF = d=r
    w ........ 0001....  LGF = d=~r
    w ........ 0010....  LGF = d=0
    w ........ 0011....  LGF = d=1
    w ........ 0100....  LGF = d=~(d^r)
    w ........ 0101....  LGF = d=d^r
    w ........ 0110....  LGF = d=d&r
    w ........ 0111....  LGF = d=~d&r
    w ........ 1000....  LGF = d=~d&~r
    w ........ 1001....  LGF = d=d&~r
    w ........ 1010....  LGF = d=~d|r
    w ........ 1011....  LGF = d=d|r
    w ........ 1100....  LGF = d=d|~r
    w ........ 1101....  LGF = d=~d|~r
    w ........ 1110....  LGF = d=d
    w ........ 1111....  LGF = d=~d
    w ........ ....1...  INV = invert transparancy state of source bits
    w ........ .....1..  BIT = copy:     enable copy type a
    w ........ .....1..  BIT = colour:   enable bcolour/colour
    w ........ .....1..  BIT = compare:  compact/compare
    w ........ ......1.  TT  = perform transparancy test
    w ........ .......0
*/

#define SCC_PCR_VREG  (m_pcab_vregs[0x14/2] & 0xffff)

/*
1ffff6  mask = mask register
    w ........ ....nnnn  mask nibbles/0
*/
/*
1ffff8  shift = shift register
    w ......nn ........  shift by .. during source alignment
*/
/*
1ffffa  index = index register
    w ........ ......nn  bcolour: use bit .. in the source word
    w ........ ......nn  compact: nibble .. will hold the result
*/
/*
1ffffc  fc/bc = foreground/background colour register
    w nnnnnnnn ........  FC = foreground colour
    w ........ nnnnnnnn  BC = background colour
*/
/*
1ffffe  tc = transparent colour register
    w nnnnnnnn ........  transparent colour
*/


void magicard_state::video_start()
{
}

uint32_t magicard_state::screen_update_magicard(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	int x, y;
	uint32_t count;

	bitmap.fill(m_palette->black_pen(), cliprect); //TODO

	if(!(SCC_DE_VREG)) //display enable
		return 0;

	count = ((SCC_VSR_VREG) / 2);

	if(SCC_FG_VREG) //4bpp gfx
	{
		for(y = 0; y < 300; y++)
		{
			for(x = 0; x < 84; x++)
			{
				uint32_t color;

				color = ((m_magicram[count]) & 0x000f) >> 0;

				if(cliprect.contains((x * 4) + 3, y))
					bitmap.pix32(y, (x * 4) + 3) = m_palette->pen(color);

				color = ((m_magicram[count]) & 0x00f0) >> 4;

				if(cliprect.contains((x * 4) + 2, y))
					bitmap.pix32(y, (x * 4) + 2) = m_palette->pen(color);

				color = ((m_magicram[count]) & 0x0f00) >> 8;

				if(cliprect.contains((x * 4) + 1, y))
					bitmap.pix32(y, (x * 4) + 1) = m_palette->pen(color);

				color = ((m_magicram[count]) & 0xf000) >> 12;

				if(cliprect.contains((x * 4) + 0, y))
					bitmap.pix32(y, (x * 4) + 0) = m_palette->pen(color);

				count++;
			}
		}
	}
	else //8bpp gfx
	{
		for(y = 0; y < 300; y++)
		{
			for(x = 0; x < 168; x++)
			{
				uint32_t color;

				color = ((m_magicram[count]) & 0x00ff) >> 0;

				if(cliprect.contains((x * 2) + 1, y))
					bitmap.pix32(y, (x * 2) + 1) = m_palette->pen(color);

				color = ((m_magicram[count]) & 0xff00) >> 8;

				if(cliprect.contains((x * 2) + 0, y))
					bitmap.pix32(y, (x * 2) + 0) = m_palette->pen(color);

				count++;
			}
		}
	}

	return 0;
}


/*************************
*      R/W Handlers      *
*************************/

READ16_MEMBER(magicard_state::test_r)
{
	return machine().rand();
}

READ16_MEMBER(magicard_state::philips_66470_r)
{
	switch(offset)
	{
		case 0/2:
		{
			uint8_t vdisp;
			vdisp = m_screen->vpos() < 256;

			return (m_pcab_vregs[offset] & 0xff7f) | vdisp << 7; //TODO
		}
	}

	//printf("[%04x]\n",offset*2);


	return m_pcab_vregs[offset];
}

WRITE16_MEMBER(magicard_state::philips_66470_w)
{
	COMBINE_DATA(&m_pcab_vregs[offset]);

//  if(offset == 0x10/2)
//  {
		//printf("%04x %04x %04x\n",data,m_pcab_vregs[0x12/2],m_pcab_vregs[0x14/2]);
		//m_pcab_vregs[0x12/2] = m_pcab_vregs[0x10/2];
//  }
}

/* scc68070 specific stuff (to be moved) */

READ16_MEMBER(magicard_state::scc68070_ext_irqc_r)
{
	return m_scc68070_ext_irqc_regs[offset];
}

WRITE16_MEMBER(magicard_state::scc68070_ext_irqc_w)
{
	data &= mem_mask;

	m_scc68070_ext_irqc_regs[offset] = data;
}

READ16_MEMBER(magicard_state::scc68070_iic_r)
{
	//printf("%04x\n",offset*2);

	switch(offset)
	{
		case 0x04/2: return m_scc68070_iic_regs[offset] & 0xef; //iic status register, bit 4 = pending irq
	}

	return m_scc68070_iic_regs[offset];
}

WRITE16_MEMBER(magicard_state::scc68070_iic_w)
{
	data &= mem_mask;

	m_scc68070_iic_regs[offset] = data;
}

READ16_MEMBER(magicard_state::scc68070_uart_r)
{
	//printf("%02x\n",offset*2);

	switch(offset)
	{
		case 0x02/2: return machine().rand(); //uart mode register
	}

	return m_scc68070_uart_regs[offset];
}

WRITE16_MEMBER(magicard_state::scc68070_uart_w)
{
	data &= mem_mask;

	m_scc68070_uart_regs[offset] = data;
}

READ16_MEMBER(magicard_state::scc68070_timer_r)
{
	return m_scc68070_timer_regs[offset];
}

WRITE16_MEMBER(magicard_state::scc68070_timer_w)
{
	data &= mem_mask;

	m_scc68070_timer_regs[offset] = data;
}

READ16_MEMBER(magicard_state::scc68070_int_irqc_r)
{
	return m_scc68070_int_irqc_regs[offset];
}

WRITE16_MEMBER(magicard_state::scc68070_int_irqc_w)
{
	data &= mem_mask;

	m_scc68070_int_irqc_regs[offset] = data;
}

READ16_MEMBER(magicard_state::scc68070_dma_ch1_r)
{
	return m_scc68070_dma_ch1_regs[offset];
}

WRITE16_MEMBER(magicard_state::scc68070_dma_ch1_w)
{
	data &= mem_mask;

	m_scc68070_dma_ch1_regs[offset] = data;
}

READ16_MEMBER(magicard_state::scc68070_dma_ch2_r)
{
	return m_scc68070_dma_ch2_regs[offset];
}

WRITE16_MEMBER(magicard_state::scc68070_dma_ch2_w)
{
	data &= mem_mask;

	m_scc68070_dma_ch2_regs[offset] = data;
}

READ16_MEMBER(magicard_state::scc68070_mmu_r)
{
	return m_scc68070_mmu_regs[offset];
}

WRITE16_MEMBER(magicard_state::scc68070_mmu_w)
{
	data &= mem_mask;

	m_scc68070_mmu_regs[offset] = data;

	switch(offset)
	{
		case 0x0000/2:
			if(data & 0x80) //throw an error if the (unemulated) MMU is enabled
				fatalerror("SCC68070: MMU enable bit active\n");
			break;
	}
}


/*************************
*      Memory Maps       *
*************************/

static ADDRESS_MAP_START( scc68070_mem, AS_PROGRAM, 16, magicard_state )
	AM_RANGE(0x80001000, 0x8000100f) AM_READWRITE(scc68070_ext_irqc_r,scc68070_ext_irqc_w) AM_SHARE("scc_xirqc_regs") //lir
	AM_RANGE(0x80002000, 0x8000200f) AM_READWRITE(scc68070_iic_r,scc68070_iic_w) AM_SHARE("scc_iic_regs") //i2c
	AM_RANGE(0x80002010, 0x8000201f) AM_READWRITE(scc68070_uart_r,scc68070_uart_w) AM_SHARE("scc_uart_regs")
	AM_RANGE(0x80002020, 0x8000202f) AM_READWRITE(scc68070_timer_r,scc68070_timer_w) AM_SHARE("scc_timer_regs")
	AM_RANGE(0x80002040, 0x8000204f) AM_READWRITE(scc68070_int_irqc_r,scc68070_int_irqc_w) AM_SHARE("scc_iirqc_regs")
	AM_RANGE(0x80004000, 0x8000403f) AM_READWRITE(scc68070_dma_ch1_r,scc68070_dma_ch1_w) AM_SHARE("scc_dma1_regs")
	AM_RANGE(0x80004040, 0x8000407f) AM_READWRITE(scc68070_dma_ch2_r,scc68070_dma_ch2_w) AM_SHARE("scc_dma2_regs")
	AM_RANGE(0x80008000, 0x8000807f) AM_READWRITE(scc68070_mmu_r,scc68070_mmu_w) AM_SHARE("scc_mmu_regs")
ADDRESS_MAP_END

static ADDRESS_MAP_START( magicard_mem, AS_PROGRAM, 16, magicard_state )
//  ADDRESS_MAP_GLOBAL_MASK(0x1fffff)
	AM_IMPORT_FROM(scc68070_mem)
	AM_RANGE(0x00000000, 0x001ffbff) AM_MIRROR(0x00200000) AM_RAM AM_SHARE("magicram")
	AM_RANGE(0x00600000, 0x007ffbff) AM_RAM AM_SHARE("magicramb")
	/* 001ffc00-001ffdff System I/O */
	AM_RANGE(0x001ffc00, 0x001ffc01) AM_MIRROR(0x7fe00000) AM_READ(test_r)
	AM_RANGE(0x001ffc40, 0x001ffc41) AM_MIRROR(0x7fe00000) AM_READ(test_r)
	AM_RANGE(0x001ffd00, 0x001ffd01) AM_MIRROR(0x7fe00000) AM_DEVWRITE8("ramdac", ramdac_device, index_w, 0x00ff)
	AM_RANGE(0x001ffd02, 0x001ffd03) AM_MIRROR(0x7fe00000) AM_DEVWRITE8("ramdac", ramdac_device, pal_w, 0x00ff)
	AM_RANGE(0x001ffd04, 0x001ffd05) AM_MIRROR(0x7fe00000) AM_DEVWRITE8("ramdac", ramdac_device, mask_w, 0x00ff)
	AM_RANGE(0x001ffd40, 0x001ffd43) AM_MIRROR(0x7fe00000) AM_DEVWRITE8("saa", saa1099_device, write, 0x00ff)
	AM_RANGE(0x001ffd80, 0x001ffd81) AM_MIRROR(0x7fe00000) AM_READ(test_r)
	AM_RANGE(0x001ffd80, 0x001ffd81) AM_MIRROR(0x7fe00000) AM_WRITENOP //?
	AM_RANGE(0x001fff80, 0x001fffbf) AM_MIRROR(0x7fe00000) AM_RAM //DRAM I/O, not accessed by this game, CD buffer?
	AM_RANGE(0x001fffe0, 0x001fffff) AM_MIRROR(0x7fe00000) AM_READWRITE(philips_66470_r,philips_66470_w) AM_SHARE("pcab_vregs") //video registers
ADDRESS_MAP_END

static ADDRESS_MAP_START( hotslots_mem, AS_PROGRAM, 16, magicard_state )
//  ADDRESS_MAP_GLOBAL_MASK(0x1fffff)
	AM_IMPORT_FROM(scc68070_mem)
	AM_RANGE(0x00000000, 0x001ffbff) AM_MIRROR(0x00200000) AM_RAM AM_SHARE("magicram")
	AM_RANGE(0x00600000, 0x007ffbff) AM_RAM AM_SHARE("magicramb")
	AM_RANGE(0x001fff80, 0x001fffbf) AM_MIRROR(0x7fe00000) AM_RAM //DRAM I/O, not accessed by this game, CD buffer?
	AM_RANGE(0x001fffe0, 0x001fffff) AM_MIRROR(0x7fe00000) AM_READWRITE(philips_66470_r,philips_66470_w) AM_SHARE("pcab_vregs") //video registers
	AM_RANGE(0x00414000, 0x00414001) AM_DEVWRITE8("ramdac", ramdac_device, index_w, 0x00ff)
	AM_RANGE(0x00414002, 0x00414003) AM_DEVWRITE8("ramdac", ramdac_device, pal_w, 0x00ff)
	AM_RANGE(0x00414004, 0x00414005) AM_DEVWRITE8("ramdac", ramdac_device, mask_w, 0x00ff)
	AM_RANGE(0x00414006, 0x00414007) AM_DEVWRITE8("ramdac", ramdac_device, index_w, 0x00ff)
	AM_RANGE(0x00415002, 0x00415003) AM_DEVREAD8("ramdac", ramdac_device, pal_r, 0x00ff)
	AM_RANGE(0x00416000, 0x00416001) AM_DEVWRITE8("ssg", ymz284_device, data_w, 0x00ff)
	AM_RANGE(0x00417000, 0x00417001) AM_DEVWRITE8("ssg", ymz284_device, address_w, 0x00ff)
ADDRESS_MAP_END

/*************************
*      Input ports       *
*************************/

static INPUT_PORTS_START( magicard )
INPUT_PORTS_END


void magicard_state::machine_reset()
{
	uint16_t *src    = (uint16_t*)memregion("maincpu" )->base();
	uint16_t *dst    = m_magicram;
	memcpy (dst, src, 0x80000);
	memcpy (dst + 0x40000 * 1, src, 0x80000);
	memcpy (dst + 0x40000 * 2, src, 0x80000);
	memcpy (dst + 0x40000 * 3, src, 0x80000);
	dst = m_magicramb;
	memcpy (dst, src, 0x80000);
	memcpy (dst + 0x40000 * 1, src, 0x80000);
	memcpy (dst + 0x40000 * 2, src, 0x80000);
	memcpy (dst + 0x40000 * 3, src, 0x80000);
	m_maincpu->reset();
}


/*************************
*    Machine Drivers     *
*************************/

/*Probably there's a mask somewhere if it REALLY uses irqs at all...irq vectors dynamically changes after some time.*/
INTERRUPT_GEN_MEMBER(magicard_state::magicard_irq)
{
	if(machine().input().code_pressed(KEYCODE_Z)) //vblank?
		device.execute().set_input_line_and_vector(1, HOLD_LINE, 0xe4 / 4);
	if(machine().input().code_pressed(KEYCODE_X)) //uart irq
		device.execute().set_input_line_and_vector(1, HOLD_LINE, 0xf0 / 4);
}

static ADDRESS_MAP_START( ramdac_map, 0, 8, magicard_state )
	AM_RANGE(0x0000, 0x03ff) AM_DEVREADWRITE("ramdac", ramdac_device, ramdac_pal_r, ramdac_rgb666_w)
ADDRESS_MAP_END


static MACHINE_CONFIG_START( magicard )
	MCFG_CPU_ADD("maincpu", SCC68070, CLOCK_A / 2)    /* SCC-68070 CCA84 datasheet */
	MCFG_CPU_PROGRAM_MAP(magicard_mem)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", magicard_state, magicard_irq) /* no interrupts? (it erases the vectors..) */

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(50)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(400, 300)
	MCFG_SCREEN_VISIBLE_AREA(0, 320-1, 0, 256-1) //dynamic resolution,TODO
	MCFG_SCREEN_UPDATE_DRIVER(magicard_state, screen_update_magicard)

	MCFG_PALETTE_ADD("palette", 0x100)
	MCFG_RAMDAC_ADD("ramdac", ramdac_map, "palette")

	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SOUND_ADD("saa", SAA1099, CLOCK_B)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( hotslots, magicard )
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(hotslots_mem)

	MCFG_DEVICE_REMOVE("saa")
	MCFG_SOUND_ADD("ssg", YMZ284, 4000000)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 1.0)
MACHINE_CONFIG_END

/*************************
*        Rom Load        *
*************************/

ROM_START( magicard )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 68070 Code & GFX */
	ROM_LOAD16_WORD_SWAP( "magicorg.bin", 0x000000, 0x80000, CRC(810edf9f) SHA1(0f1638a789a4be7413aa019b4e198353ba9c12d9) )

	ROM_REGION( 0x0100, "sereeprom", 0 ) /* Serial EPROM */
	ROM_LOAD16_WORD_SWAP("mgorigee.bin",    0x0000, 0x0100, CRC(73522889) SHA1(3e10d6c1585c3a63cff717a0b950528d5373c781) )
ROM_END

ROM_START( magicarda )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 68070 Code & GFX */
	ROM_LOAD16_WORD_SWAP( "mcorigg2.bin", 0x00000, 0x20000, CRC(48546aa9) SHA1(23099a5e4c9f2c3386496f6d7f5bb7d435a6fb16) )
	ROM_RELOAD(                           0x40000, 0x20000 )
	ROM_LOAD16_WORD_SWAP( "mcorigg1.bin", 0x20000, 0x20000, CRC(c9e4a38d) SHA1(812e5826b27c7ad98142a0f52fbdb6b61a2e31d7) )
	ROM_RELOAD(                           0x40001, 0x20000 )

	ROM_REGION( 0x0100, "sereeprom", 0 ) /* Serial EPROM */
	ROM_LOAD("mgorigee.bin",    0x0000, 0x0100, CRC(73522889) SHA1(3e10d6c1585c3a63cff717a0b950528d5373c781) )
ROM_END

ROM_START( magicardb )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 68070 Code & GFX */
	ROM_LOAD16_WORD_SWAP( "mg_8.bin", 0x00000, 0x80000, CRC(f5499765) SHA1(63bcf40b91b43b218c1f9ec1d126a856f35d0844) )

	/*bigger than the other sets?*/
	ROM_REGION( 0x20000, "other", 0 ) /* unknown */
	ROM_LOAD16_WORD_SWAP("mg_u3.bin",   0x00000, 0x20000, CRC(2116de31) SHA1(fb9c21ca936532e7c342db4bcaaac31c478b1a35) )
ROM_END

ROM_START( magicardj )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 68070 Code & GFX */
	ROM_LOAD16_WORD_SWAP( "27c4002.ic21", 0x00000, 0x80000, CRC(ab2ed583) SHA1(a2d7148b785a8dfce8cff3b15ada293d65561c98) ) // sldh

	ROM_REGION( 0x0100, "pic16f84", 0 ) /* protected */
	ROM_LOAD("pic16f84.ic29",   0x0000, 0x0100, BAD_DUMP CRC(0d968558) SHA1(b376885ac8452b6cbf9ced81b1080bfd570d9b91) )

	ROM_REGION( 0x200000, "other", 0 ) /* unknown contents */
	ROM_LOAD("29f1610mc.ic30",  0x000000, 0x200000, NO_DUMP )

	ROM_REGION( 0x0100, "sereeprom", 0 ) /* Serial EPROM */
	ROM_LOAD("24c02c.ic26", 0x0000, 0x0100, CRC(b5c86862) SHA1(0debc0f7e7c506e5a4e2cae152548d80ad72fc2e) )
ROM_END

/*
  Magic Card Export 94
  International Ver. 2.11a
  Vnr.29.07.94    CHECKSUM: A63D

  1x Philips SCC66470CAB 383610
  1x Philips SCC68070 CCA84 347141
  1x ESI1 I9631
  1x MUSIC TR9C1710-11PCA SA121X/9617
  1x YAMAHA YM2149F 9614

  XTAL:

  Q1: 19.6608 Mhz
  Q2: 30.000 Mhz
  Q3: 3686.400  1Q08/95
*/

ROM_START( magicarde )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 68070 Code & GFX */
	ROM_LOAD16_WORD_SWAP( "27c4002.ic21", 0x00000, 0x80000, CRC(b5f24412) SHA1(73ff05c19132932a419fef0d5dc985440ce70e83) )

	ROM_REGION( 0x0200, "pic16c54", 0 ) /* protected */
	ROM_LOAD("pic16c54.ic29",   0x0000, 0x0200, BAD_DUMP CRC(73224200) SHA1(c9a1038146647430759d570bb5626047a476a05b) )

	ROM_REGION( 0x0100, "sereeprom", 0 ) /* Serial EPROM */
	ROM_LOAD("st24c02.ic26",    0x0000, 0x0100, CRC(98287c67) SHA1(ad34e55c1ce4f77c27049dac88050ed3c94af1a0) )
ROM_END

ROM_START( magicle )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 68070 Code & GFX */
	ROM_LOAD16_WORD_SWAP( "27c4002.ic21", 0x00000, 0x80000, CRC(73328346) SHA1(fca5f8a93f25377e659c2b291674d706ca37400e) )

	ROM_REGION( 0x0100, "pic16f84", 0 ) /* protected */
	ROM_LOAD("pic16f84.ic29",   0x0000, 0x0100, BAD_DUMP CRC(0d968558) SHA1(b376885ac8452b6cbf9ced81b1080bfd570d9b91) )

	ROM_REGION( 0x200000, "other", 0 ) /* unknown contents */
	ROM_LOAD("29f1610mc.ic30",  0x000000, 0x200000, NO_DUMP )

	ROM_REGION( 0x0200, "sereeprom", 0 ) /* Serial EPROM */
	ROM_LOAD("24c04a.ic26", 0x0000, 0x0200, CRC(48c4f473) SHA1(5355313cc96f655096e13bfae78be3ba2dfe8a2d) )
ROM_END

/*

  Hot Slots Version 6.00

  Hardware PCB informations:
  E179465--A/02 LPL-CPU V4.0/MULTI GAME

  Eprom type AM27C4096
  Version 6.00
  vnr 15.04.02 Cksum (provided) 0D08

*/
ROM_START( hotslots )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 68070 Code & GFX */
	ROM_LOAD16_WORD_SWAP( "hot_slots_v600_15.04.02.bin", 0x00000, 0x80000, CRC(35677999) SHA1(7462eef3734b9b6087102901967a168a60ab7710) )

	ROM_REGION( 0x0100, "sereeprom", 0 ) /* Serial EPROM */
	ROM_LOAD16_WORD_SWAP("hot_slots_24c02.bin",          0x0000,  0x0100,  CRC(fcac71ad) SHA1(1bb31e9a2d847430dc0d011f672cf3726dc6280c) )
ROM_END

/*

  QUINGO EXPORT Version 5.00

  Hardware PCB informations : E179465--A/02 LPL-CPU V4.0/MULTI GAME 8603186
  Eprom type ST M27c4002
  Version 5.00
  vnr 27.07.99 Cksum (provided) 79C5

  Eeprom : 24c04A

*/
ROM_START( quingo )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 68070 Code & GFX */
	ROM_LOAD16_WORD_SWAP( "quingo_export_v500_27.07.99.bin", 0x00000, 0x80000, CRC(2cd89fe3) SHA1(bdd256d5114227166aff1c9f84b573e5f00530fd) )

	ROM_REGION( 0x0200, "sereeprom", 0 ) /* Serial EPROM */
	ROM_LOAD16_WORD_SWAP("quingo_24c04a.bin", 0x0000, 0x0200, BAD_DUMP CRC(d5e82b49) SHA1(7dbdf7d539cbd59a3ac546b6f50861c4958abb3a) ) // all AA & 55
ROM_END

/*

  BIG DEAL BELGIEN Version 5.04

  Hardware PCB informations : E179465--A/02 LPL-CPU V4.0/MULTI GAME 8603186
  Eprom type ST M27c4002
  Version 5.04
  vnr 21.05.01 Cksum (provided) C4B7


  Eeprom : 24c04A

*/
ROM_START( bigdeal0 )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 68070 Code & GFX */
	ROM_LOAD16_WORD_SWAP( "big_deal_belgien_v504_21.05.01.bin", 0x00000, 0x80000, CRC(3e3484db) SHA1(78bb655deacc57ad041a46de7ef153ce25922a8a) )

	ROM_REGION( 0x0200, "sereeprom", 0 ) /* Serial EPROM */
	ROM_LOAD16_WORD_SWAP("big_deal_24c04a.bin", 0x0000, 0x0200, BAD_DUMP CRC(d5e82b49) SHA1(7dbdf7d539cbd59a3ac546b6f50861c4958abb3a) ) // all AA & 55
ROM_END

/*

  BEL SLOTS EXP. Version 5.01

  Hardware PCB informations : E179465--A/02 LPL-CPU V4.0/MULTI GAME 8603186
  Eprom type ST M27c4002
  Version 5.01
  vnr 01.12.99 Cksum (provided) F718

  Eeprom : 24c04A

*/
ROM_START( belslots )
	ROM_REGION( 0x80000, "maincpu", 0 ) /* 68070 Code & GFX */
	ROM_LOAD16_WORD_SWAP( "bel_slots_exp_v501_01.12.99.bin", 0x00000, 0x80000, CRC(bd0b97ff) SHA1(9431359f91fd059c61441f4cb4924500889552a9) )

	ROM_REGION( 0x0200, "sereeprom", 0 ) /* Serial EPROM */
	ROM_LOAD16_WORD_SWAP("bel_slots_exp_24c04a.bin", 0x0000, 0x0200, BAD_DUMP CRC(d5e82b49) SHA1(7dbdf7d539cbd59a3ac546b6f50861c4958abb3a) ) // all AA & 55
ROM_END



/*************************
*      Driver Init       *
*************************/

DRIVER_INIT_MEMBER(magicard_state, magicard)
{
	//...
}


/*************************
*      Game Drivers      *
*************************/

//    YEAR  NAME       PARENT    MACHINE   INPUT     STATE           INIT      ROT   COMPANY   FULLNAME                     FLAGS

GAME( 199?, magicard,  0,        magicard, magicard, magicard_state, magicard, ROT0, "Impera", "Magic Card (set 1)",        MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
GAME( 199?, magicarda, magicard, magicard, magicard, magicard_state, magicard, ROT0, "Impera", "Magic Card (set 2)",        MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
GAME( 199?, magicardb, magicard, magicard, magicard, magicard_state, magicard, ROT0, "Impera", "Magic Card (set 3)",        MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
GAME( 1994, magicarde, magicard, magicard, magicard, magicard_state, magicard, ROT0, "Impera", "Magic Card Export 94",      MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
GAME( 1998, magicardj, magicard, magicard, magicard, magicard_state, magicard, ROT0, "Impera", "Magic Card Jackpot (4.01)", MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
GAME( 2001, magicle,   0,        magicard, magicard, magicard_state, magicard, ROT0, "Impera", "Magic Lotto Export (5.03)", MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
GAME( 2002, hotslots,  0,        hotslots, magicard, magicard_state, magicard, ROT0, "Impera", "Hot Slots (6.00)",          MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
GAME( 1999, quingo,    0,        hotslots, magicard, magicard_state, magicard, ROT0, "Impera", "Quingo Export (5.00)",      MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
GAME( 1999, belslots,  0,        hotslots, magicard, magicard_state, magicard, ROT0, "Impera", "Bel Slots Export (5.01)",   MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
GAME( 2001, bigdeal0,  0,        hotslots, magicard, magicard_state, magicard, ROT0, "Impera", "Big Deal Belgien (5.04)",   MACHINE_NO_SOUND | MACHINE_NOT_WORKING )
