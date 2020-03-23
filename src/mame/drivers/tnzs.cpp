// license:BSD-3-Clause
// copyright-holders:Luca Elia, Mirko Buffoni, Takahiro Nogi
/***************************************************************************


Notes:
- There are four versions of TNZS supported.
  1) "tnzs".   New hardware revision. 3 Z80 and no M-Chip (8742 MPU).
  2) "tnzsj".  New hardware revision. 3 Z80 and no M-Chip (8742 MPU).
  3) "tnzsjo". Standard hardware. 2 Z80 and the M-Chip.
  4) "tnzsop".  Standard hardware. Harder gameplay, old Taito logo. Maybe a prototype?

- TNZS hidden level select: keep service coin pressed (default:9) and reset the game.
  When SERVICE SWITCH ERROR appears, release service coin and press 1 player start
  three times, service coin again, and 1 player start. Use up/down to select the
  starting level and press 1 player start to start. You'll also get 255 lives.
  If you have enough patience and go up until the level reads "Q-1" (which corresponds
  to 1-1), AND the "Invulnerability" dip switch is On, you'll be invulnerable.
  Invulnerability isn't possible in 'tnzsop' (level select is stucked to level 6-1).


Hardware datails for the newer tnzs board (from pictures):

  Main board
  M6100409A N.ZEALAND STORY (written on label)
  M6100356A (written on pcb)
  (note: Taito logo is the old version)
  SETA X1-001A 8740KX
  SETA X1-002A 8712KX
  SETA X1-004  815100
  SETA X1-006  8136KX
  SETA X1-007  805100
  these custom ics are also used in many other games (see drivers/seta.c, drivers/taito_x.c)
  Main xtal 12.0000 MHZ
  2xZ80-B
  YM2203C
  sockets for U2 (Z80-B) and U35/U39/U43/U46 (LH534000) are empty and the sub board
  connects to them.

  SUB PCB
  K9100209A N.Z.LAND STORY (written on label)
  K9100209A (written on pcb)
  J9100159A (written on pcb)
  (note: Taito logo is the new version)
  Z80-B



Stephh's notes (based on the games Z80 code and some tests) :

1) 'plumppop'

  - Region stored at 0x7fff.b (CPU1) then 0xef49 (shared RAM)
  - Sets :
      * 'plumppop' : region = 0x00
  - Coinage relies on bit 0 of the region (code at 0x5141 in CPU1) :
      * .......0 : TAITO_COINAGE_JAPAN_OLD
      * .......1 : TAITO_COINAGE_WORLD
  - Notice screen relies on bit 1 of the region (code at 0x16e8) :
      * ......0. : Yes
      * ......1. : No
  - Copyright relies on bits 2 and 3 of the region (code at 0x276d) :
      * ....00.. : "TAITO CORPORATION" / "ALL RIGHTS RESERVED" (Japan)
      * ....01.. : "TAITO AMERICA CORP." / "ALL RIGHTS RESERVED" (US)
      * ....10.. : "TAITO CORP. JAPAN" / "ALL RIGHTS RESERVED" (World)
      * ....11.. : "TAITO CORP. JAPAN" / "LICENCED TO -------"


2) 'extrmatn' and clones

2a) 'extrmatn'

  - Region stored at 0x7fff.b (CPU1) then 0x16 is stored at 0xef1d (shared RAM) !
  - Sets :
      * 'extrmatn' : region = 0x16
  - New value is stored at 0xef1d (shared RAM) :
      * 'extrmatn' : 0x16
  - Coinage relies on bit 0 of the region (code at 0x4703 in CPU1) :
      * .......0 : TAITO_COINAGE_JAPAN_OLD
      * .......1 : TAITO_COINAGE_WORLD
  - Notice screen relies on bit 1 of the new value (code at 0x4285) :
      * ......0. : Yes
      * ......1. : No
  - Copyright relies on bits 2 and 3 of the new value (code at 0x0cfe) :
      * ....00.. : "TAITO CORPORATION" / "     FOR U.S.A.    " (Japan)
      * ....01.. : "WORLD GAMES     " / "     FOR U.S.A.    " (US)
      * ....10.. : "TAITO CORP. JAPAN" / "     FOR U.S.A.    " (World)
      * ....11.. : "TAITO CORP. JAPAN" / "LICENCED TO        "
  - 'ret' + modified byte to please checksum at 0x8df5 (bank = 0x07) : unknown effect

2b) 'extrmatj'

  - Region stored at 0x7fff.b (CPU1) then 0x00 is stored at 0xef1d (shared RAM) !
  - Sets :
      * 'extrmatj' : region = 0x00
  - Coinage relies on bit 0 of the region (code at 0x4703 in CPU1) :
      * .......0 : TAITO_COINAGE_JAPAN_OLD
      * .......1 : TAITO_COINAGE_WORLD
  - Notice screen relies on bit 1 of the new value (code at 0x4285) :
      * ......0. : Yes
      * ......1. : No
  - Copyright relies on bits 2 and 3 of the new value (code at 0x0cfe) :
      * ....00.. : "TAITO CORPORATION" / "ALL RIGHTS RESERVED" (Japan)
      * ....01.. : "TAITO AMERICA CORP." / "ALL RIGHTS RESERVED" (US)
      * ....10.. : "TAITO CORP. JAPAN" / "ALL RIGHTS RESERVED" (World)
      * ....11.. : "TAITO CORP. JAPAN" / "LICENCED TO           "
  - Code and data differences :
      * 0x8df5 (bank = 0x07) : normal routine but still unknown effect
      * 0x01e0 (CPU1) : 0x00 instead of 0x16 (value for region tests)


3) 'arknoid2' and clones

3a) 'arknoid2' and 'arknid2j'

  - Region stored at 0x9fde.b (CPU1 - bank = 0x00) then stored at 0xe7f0 (shared RAM)
  - Coin mode stored at 0x9fff.b (CPU1 - bank = 0x00) then stored at 0xe7f1 (shared RAM)
  - Sets :
      * 'arknoid2' : region = 0x02 - coin mode = 0x01
      * 'arknid2j' : region = 0x00 - coin mode = 0x00
  - Coinage relies on coin mode (code at 0x0b17 in CPU1) :
      * 0x00 : TAITO_COINAGE_JAPAN_OLD
      * else : TAITO_COINAGE_WORLD
  - Notice screen relies on region (code at 0x495b) :
      * 0x00 : Yes
      * else : No
  - Copyright relies on region (code at 0x4774) :
      * 0x00 : "TAITO CORPORATION" / "ALL RIGHTS RESERVED" (Japan)
      * 0x01 : "TAITO AMERICA CORP." / "ALL RIGHTS RESERVED" (US)
      * 0x02 : "TAITO CORP. JAPAN" / "ALL RIGHTS RESERVED" (World)
      * 0x03 : "TAITO AMERICA CORP." / "LICENCED TO ROMSTAR" / "FOR U.S.A."

3b) 'arknid2u'

  - Region stored at 0x9fde.b (CPU1 - bank = 0x00) then stored at 0xe7f0 (shared RAM)
  - Coin mode stored at 0x9fff.b (CPU1 - bank = 0x00) then stored at 0xe7f1 (shared RAM)
  - Sets :
      * 'arknid2u' : region = 0x03 - coin mode = 0x00
  - Coinage relies on coin mode (code at 0x0b17 in CPU1) :
      * 0x00 : TAITO_COINAGE_JAPAN_OLD
      * else : TAITO_COINAGE_WORLD
  - Notice screen relies on region (code at 0x2e47) :
      * 0x00 : Yes
      * else : No
  - Copyright relies on region (code at 0x475b) :
      * 0x00 : "TAITO CORPORATION" / "ALL RIGHTS RESERVED" (Japan)
      * 0x01 : "TAITO AMERICA CORP." / "ALL RIGHTS RESERVED" (US)
      * 0x02 : "TAITO CORP. JAPAN" / "ALL RIGHTS RESERVED" (World)
      * 0x03 : "TAITO AMERICA CORP." / "LICENCED TO ROMSTAR" / "FOR U.S.A."
  - Levels differences :
      * new level 1
      * levels 3 left and 17 left are swapped
      * levels 17 right and 20 right are swapped


4) 'drtoppel' and clones

  - Region stored at 0x7fff.b (CPU1) then 0xe000 (shared RAM)
  - Sets :
      * 'drtoppel' : region = 0x03
      * 'drtopplu' : region = 0x02
      * 'drtopplj' : region = 0x01
  - These 3 games are 100% the same, only region differs !
    However, dumps for 'drtoppel' and 'drtopplu' are unsure (wrong ROM numbers).
  - Coinage relies on region (code at 0x5141 in CPU1) :
      * 0x01 and 0x02 : TAITO_COINAGE_JAPAN_OLD
      * 0x03 and 0x04 : TAITO_COINAGE_WORLD
  - Notice screen relies on region (code at 0x9331 - bank = 0x03) :
      * 0x01 : Yes
      * else : No
  - Title relies on region (code at 0x90a4 - bank = 0x03) :
      * 0x01 : Japanese
      * else : English
  - Copyright relies on region (code at 0x9b62 - bank = 0x03) :
      * 0x01 : "TAITO CORPORATION" / "ALL RIGHTS RESERVED" (Japan)
      * 0x02 : "TAITO AMERICA CORP." / "ALL RIGHTS RESERVED" (US)
      * else : "TAITO CORP. JAPAN" / "ALL RIGHTS RESERVED" (World / ???)


5) 'kageki' and clones

5a) 'kagekiu'

  - Region stored at 0x9fff.b (CPU1 - bank = 0x03) then 0xe000 (shared RAM)
  - Sets :
      * 'kagekiu'   : region = 0x02
  - Coinage relies on region (code at 0x0099 in CPU1) :
      * 0x01 and 0x02 : TAITO_COINAGE_JAPAN_OLD
      * 0x03 and 0x04 : TAITO_COINAGE_WORLD
  - No notice screen
  - Copyright relies on region (code at 0x1eeb) :
      * 0x01 : "TAITO CORPORATION" / "ALL RIGHTS RESERVED" (Japan)
      * 0x02 : "TAITO AMERICA CORP." / "LICENCED TO ROMSTAR" / "FOR U.S.A." (US)
      * else : "TAITO CORP. JAPAN" / "ALL RIGHTS RESERVED" (World / ???)
  - English text on bad guys screens when game starts + English samples
  - 2 players gameplay : players fight against each other in a best of 3 VS match,
    then the winner is allowed to fight against the CPU enemies.
    This is the reason why I've named DSWA bit 0 as "Unused".
    Note that if there is a draw (same energy for both players), player 1 wins.

5b) 'kagekij'

  - Region stored at 0x9fff.b (CPU1 - bank = 0x03) then 0xe000 (shared RAM)
  - Sets :
      * 'kagekij'  : region = 0x01
  - Coinage relies on region (code at 0x0099 in CPU1) :
      * 0x01 and 0x02 : TAITO_COINAGE_JAPAN_OLD
      * 0x03 and 0x04 : TAITO_COINAGE_WORLD
  - No notice screen
  - Copyright relies on region (code at 0x1eb3) :
      * 0x01 : "TAITO CORPORATION" / "ALL RIGHTS RESERVED" (Japan)
      * 0x02 : "TAITO AMERICA CORP." / "ALL RIGHTS RESERVED" (US)
      * else : "TAITO CORP. JAPAN" / "ALL RIGHTS RESERVED" (World / ???)
  - Japanese text on bad guys screens when game starts + Japanese samples
  - 2 players gameplay : players fight one after the other against CPU enemies.
    Once an enemy is defeated or a player is dead, it's the other player turn.
  - Code and data differences :
      * CPU0 to be looked at carefully
      * 0x9fff (CPU1 - bank = 0x03) : 0x01 instead of 0x02 (value for region tests)

5c) 'kagekih'

  - Region stored at 0x9fff.b (CPU1 - bank = 0x03) then 0xe000 (shared RAM)
  - Sets :
      * 'kagekij'  : region = 0x01
  - This set really looks like a hack :
      * year has been changed from 1988 to 1992
      * the game uses Japanese ROMS, but CPU0 ROM displays English text
        on bad guys screens when game starts
  - Coinage relies on region (code at 0x0099 in CPU1) :
      * 0x01 and 0x02 : TAITO_COINAGE_JAPAN_OLD
      * 0x03 and 0x04 : TAITO_COINAGE_WORLD
  - No notice screen
  - Copyright relies on region (code at 0x1ee0) :
      * 0x01 : "TAITO CORPORATION" / "ALL RIGHTS RESERVED" (Japan)
      * 0x02 : "TAITO AMERICA CORP." / "ALL RIGHTS RESERVED" (US)
      * else : "TAITO CORP. JAPAN" / "ALL RIGHTS RESERVED" (World / ???)
  - English text on bad guys screens when game starts but Japanese samples !
  - 2 players gameplay : players fight against each other in a best of 3 VS match,
    then the winner is allowed to fight against the CPU enemies.
    This is the reason why I've named DSWA bit 0 as "Unused".
    Note that if there is a draw (same energy for both players), player 1 wins.
  - Code and data differences :
      * CPU0 to be looked at carefully
      * 0x9fff (CPU1 - bank = 0x03) : 0x01 instead of 0x02 (value for region tests)


6) 'chukatai' and clones

  - Region stored at 0x9fff.b (CPU1 - bank = 0x03) then 0xe01a (shared RAM)
  - Sets :
      * 'chukatai' : region = 0x03
      * 'chukatau' : region = 0x02
      * 'chukataj' : region = 0x01
  - These 3 games are 100% the same, only region differs !
    However, dumps for 'chukatai' and 'chukatau' are unsure (wrong ROM numbers).
  - Coinage relies on region (code at 0x0114 in CPU1) :
      * 0x01 and 0x02 : TAITO_COINAGE_JAPAN_OLD
      * 0x03 and 0x04 : TAITO_COINAGE_WORLD
  - Notice screen relies on region (code at 0x0253) :
      * 0x01 : Yes
      * else : No
  - Copyright relies on region (code at 0x0b38) :
      * 0x01 : "TAITO CORPORATION" / "ALL RIGHTS RESERVED" (Japan)
      * 0x02 : "TAITO AMERICA CORPORATION" / "ALL RIGHTS RESERVED" (US)
      * else : "TAITO CORPORATION JAPAN" / "ALL RIGHTS RESERVED" (World / ???)
    I still need to understand how the strings are displayed though.
  - Power-ups display is affected by the region


7) 'tnzs' and clones

7a) 'tnzs' and 'tnzsj'

  - Region stored at 0x7fff.b (CPU1)
  - Sets :
      * 'tnzs'     : region = 0x03
      * 'tnzsj'    : region = 0x01
  - These 2 games are 100% the same, only region differs !
  - Code at 0x00fb (CPU1) changes region to another value :
      * 0x01 -> 0x00
      * 0x02 -> 0x06
      * 0x03 -> 0x0b
      * 0x04 -> 0x0e
    This value is stored at 0xd035 (CPU1) then 0xef1d (shared RAM).
  - Coinage relies on bit 0 of the new value (code at 0x0aa7 in CPU1) :
      * .......0 : TAITO_COINAGE_JAPAN_OLD
      * .......1 : TAITO_COINAGE_WORLD
  - Notice screen relies on bit 1 of the new value (code at 0xbed0 - bank = 0x05) :
      * ......0. : Yes
      * ......1. : No
  - Copyright relies on bits 2 and 3 of the new value (code at 0x12eb) :
      * ....00.. : "TAITO CORPORATION" / "ALL RIGHTS RESERVED" (Japan)
      * ....01.. : "TAITO AMERICA CORP." / "ALL RIGHTS RESERVED" (US)
      * ....10.. : "TAITO CORP. JAPAN" / "ALL RIGHTS RESERVED" (World)
      * ....11.. : "TAITO CORP. JAPAN" / "LICENCED TO           "
  - New Taito logo

7b) 'tnzsjo'

  - Region stored at 0x7fff.b (CPU1)
  - Sets :
      * 'tnzsjo'   : region = 0x01
  - Code at 0x00f7 (CPU1) changes region to another value :
      * 0x01 -> 0x00
      * 0x02 -> 0x06
      * 0x03 -> 0x0b
      * 0x04 -> 0x0e
    This value is stored at 0xd035 (CPU1) then 0xef1d (shared RAM).
  - Coinage relies on bit 0 of the new value (code at 0x5f68 in CPU1) :
      * .......0 : TAITO_COINAGE_JAPAN_OLD
      * .......1 : TAITO_COINAGE_WORLD
  - Notice screen relies on bit 1 of the new value (code at 0xbbb5 - bank = 0x05) :
      * ......0. : Yes
      * ......1. : No
  - Copyright relies on bits 2 and 3 of the new value (code at 0x12ed) :
      * ....00.. : "TAITO CORPORATION" / "ALL RIGHTS RESERVED" (Japan)
      * ....01.. : "TAITO AMERICA CORP." / "ALL RIGHTS RESERVED" (US)
      * ....10.. : "TAITO CORP. JAPAN" / "ALL RIGHTS RESERVED" (World)
      * ....11.. : "TAITO CORP. JAPAN" / "LICENCED TO           "
  - New Taito logo
  - levels are different from 'tnzs'

7c) 'tnzsop'

  - Region stored at 0x7fff.b (CPU1)
  - Sets :
      * 'tnzso'    : region = 0x03
  - Code at 0x021e (CPU1) changes region to another value :
      * 0x01 -> 0x00
      * 0x02 -> 0x06
      * 0x03 -> 0x0b
      * 0x04 -> 0x0e
    This value is stored at 0xd46b (CPU1) then 0xef1d (shared RAM).
  - Coinage relies on bit 0 of the new value (code at 0x5d91 in CPU1) :
      * .......0 : TAITO_COINAGE_JAPAN_OLD
      * .......1 : TAITO_COINAGE_WORLD
  - Notice screen relies on bit 1 of the new value (code at 0x0cdd) :
      * ......0. : Yes
      * ......1. : No
  - Copyright relies on bits 2 and 3 of the new value (code at 0x1062) :
      * ....00.. : "TAITO CORPORATION" / "ALL RIGHTS RESERVED" (Japan)
      * ....01.. : "TAITO AMERICA CORP." / "ALL RIGHTS RESERVED" (US)
      * ....10.. : "TAITO CORP. JAPAN" / "ALL RIGHTS RESERVED" (World)
      * ....11.. : "TAITO CORP. JAPAN" / "LICENCED TO           "
  - Old Taito logo
  - levels are different from 'tnzs' and 'tnzsjo'


8) 'kabukiz' and clones

  - No region byte : a value is loaded in register A each time it is needed !
  - Sets :
      * 'kabukiz'  : value = 0x02
      * 'kabukizj' : value = 0x00
  - Coinage relies on register A (code at 0x016c in CPU1)
      * 0x02 : TAITO_COINAGE_WORLD
      * else : TAITO_COINAGE_JAPAN_OLD
  - Notice screen relies on register A (code at 0x0b30 in CPU1) :
      * 0x00 : Yes
      * else : No
  - Copyright relies on register A (code at 0x0d4e in CPU1)
      * 0x00 : "TAITO CORPORATION" / "ALL RIGHTS RESERVED" (Japan)
      * 0x01 : "TAITO AMERICA CORP." / "ALL RIGHTS RESERVED" (US)
      * else : "TAITO CORP. JAPAN" / "ALL RIGHTS RESERVED" (World)
  - 'babukizj' has extra code in CPU1 to be looked at carefully.


9) 'insectx'

  - Region stored at 0x9fff.b (CPU1 - bank = 0x03) then 0xec21 (shared RAM)
  - Sets :
      * 'insectx'  : region = 0x03
  - Coinage relies on region (code at 0x025b in CPU1) :
      * 0x01 : TAITO_COINAGE_JAPAN_OLD
      * 0x02 : TAITO_COINAGE_US
      * else : TAITO_COINAGE_WORLD
  - Notice screen relies on region (code at 0x0a6c) :
      * 0x01 : Yes
      * else : No
  - Copyright relies on region (code at 0xac16 - bank = 0x02) :
      * 0x01 : "TAITO CORPORATION" / "ALL RIGHTS RESERVED" (Japan)
      * 0x02 : "TAITO AMERICA CORPORATION" / "ALL RIGHTS RESERVED" (US)
      * else : "TAITO CORPORATION JAPAN" / "ALL RIGHTS RESERVED" (World)
  - This game doesn't use standard settings for "Difficulty" Dip Switch :
    look at table at 0x1b86 (4 * 2 bytes) and especially table at 0x4731
    (4 bytes) which determines end of level boss energy via code at 0x46e2,
    and you'll notice that "Easy" and "Hard" settings are swapped.
    End of level boss energy is stored at 0xe484 and 0xe485 (LSB first).


10) 'jpopnics'

  - Region stored at 0x7fff.b (CPU1) then 0xef49 (shared RAM)
  - Sets :
      * 'jpopnics' : region = 0x00
  - Coinage relies on bit 0 of the region (code at 0x5141 in CPU1) :
      * .......0 : TAITO_COINAGE_JAPAN_OLD
      * .......1 : TAITO_COINAGE_WORLD
    However, coinage is in fact always 1C_1C for both coin slots due to
    'ret' instruction at 0x5151 and way coins are handled at 0x4f60.
  - Notice screen relies on bit 1 of the region (code at 0x16e8) :
      * ......0. : Yes
      * ......1. : No
    Notice screen displays "Korea" instead of "Japan".
  - Copyright relies on bits 2 and 3 of the region (code at 0x276d),
    but table at 0x2781 is the same for any combinaison :
      * ....??.. : "NICS CO. LTD. 1992" / "ALL RIGHTS RESERVED"


TODO:
-----
- Find out how the hardware credit-counter works (MPU)
- Fix MCU simulation errors :
    * avoid credits to be increased when in "test mode" to avoid coin lockout
      (needed to complete special test mode in 'extrmatn' and 'arknoid2')
    * why are credits limited to 9 in some games ?
      pressing SERVICE1 allows to go up to 100 and cause this :
        . 'plumppop' : freeze
        . 'extrmatn' : reset
        . 'arknoid2' : coin overflow
- Fix video offsets (See Dr Toppel in Flip-Screen - also affects Chuka Taisen)
- Video scroll side flicker in Chuka Taisen, Insector X, Dr Toppel, Kabuki Z
- Sprite/background sync during scrolling, e.g. insectx, kabukiz.
- Merge video driver with seta.c (it's the same thing but seta.c assumes a 16-bit CPU)
- Figure out remaining unknown Dip Switches in 'plumppop' and 'jpopnics'
- Get rid of the COMMON_* macros to use generic INPUT_PORTS definition
  and PORT_INCLUDE macro by changing the ym2203_config definition


Arkanoid 2:
  - What do writes at $f400 do ?
  - Why does the game zero the $fd00 area ?
Extrmatn:
  - What do reads from $f600 do ? (discarded)
Chuka Taisen:
  - What do writes at  $f400 do ? (value 40h)
  - What do reads from $f600 do in service mode ?
Dr Toppel:
  - What do writes at  $f400 do ? (value 40h)
  - What do reads from $f600 do in service mode ?

****************************************************************************

extrmatn and arknoid2 have a special test mode. The correct procedure to make
it succeed is as follows:
- enter service mode
- on the color test screen, press 2 (player 2 start)
- set dip switch 1 and dip switch 2 so that they read 00000001
- reset the emulation, and skip the previous step.
- press 5 (coin 1). Text at the bottom will change to "CHECKING NOW".
- use all the inputs, including tilt, until all inputs are OK
- press 5 (coin 1) - to confirm that coin lockout 1 works
- press 5 (coin 1) - to confirm that coin lockout 2 works
- set dip switch 1 to 00000000
- set dip switch 1 to 10101010
- set dip switch 1 to 11111111
- set dip switch 2 to 00000000
- set dip switch 2 to 10101010
- set dip switch 2 to 11111111
- speaker should now output a tone
- press 5 (coin 1), to confirm that OPN works
- press 5 (coin 1), to confirm that SSGCH1 works
- press 5 (coin 1), to confirm that SSGCH2 works
- press 5 (coin 1), to confirm that SSGCH3 works
- finished ("CHECK ALL OK!")

****************************************************************************

The New Zealand Story memory map (preliminary)

CPU #1
0000-7fff ROM
8000-bfff banked - banks 0-1 RAM; banks 2-7 ROM
c000-dfff object RAM, including:
  c000-c1ff sprites (code, low byte)
  c200-c3ff sprites (x-coord, low byte)
  c400-c5ff tiles (code, low byte)

  d000-d1ff sprites (code, high byte)
  d200-d3ff sprites (x-coord and colour, high byte)
  d400-d5ff tiles (code, high byte)
  d600-d7ff tiles (colour)
e000-efff RAM shared with CPU #2
f000-ffff VDC RAM, including:
  f000-f1ff sprites (y-coord)
  f200-f2ff scrolling info
  f300-f301 vdc controller
  f302-f303 scroll x-coords (high bits)
  f600      bankswitch
  f800-fbff palette

CPU #2
0000-7fff ROM
8000-9fff banked ROM
a000      bankswitch
b000-b001 YM2203 interface (with DIPs on YM2203 ports)
c000-c001 I8742 MCU
e000-efff RAM shared with CPU #1
f000-f003 inputs (used only by Arkanoid 2)

****************************************************************************/
/***************************************************************************

                Arkanoid 2 - Revenge of Doh!
                    (C) 1987 Taito

                        driver by

                Luca Elia (l.elia@tin.it)
                Mirko Buffoni

- The game doesn't write to f800-fbff (static palette)



            Interesting routines (main cpu)
            -------------------------------

1ed prints the test screen (first string at 206)

47a prints dipsw1&2 e 1p&2p paddleL values:
    e821        IN DIPSW1       e823-4  1P PaddleL (lo-hi)
    e822        IN DIPSW2       e825-6  2P PaddleL (lo-hi)

584 prints OK or NG on each entry:
    if (*addr)!=0 { if (*addr)!=2 OK else NG }
    e880    1P PADDLEL      e88a    IN SERVICE
    e881    1P PADDLER      e88b    IN TILT
    e882    1P BUTTON       e88c    OUT LOCKOUT1
    e883    1P START        e88d    OUT LOCKOUT2
    e884    2P PADDLEL      e88e    IN DIP-SW1
    e885    2P PADDLER      e88f    IN DIP-SW2
    e886    2P BUTTON       e890    SND OPN
    e887    2P START        e891    SND SSGCH1
    e888    IN COIN1        e892    SND SSGCH2
    e889    IN COIN2        e893    SND SSGCH3

672 prints a char
715 prints a string (0 terminated)

        Shared Memory (values written mainly by the sound cpu)
        ------------------------------------------------------

e001=dip-sw A   e399=coin counter value     e72c-d=1P paddle (lo-hi)
e002=dip-sw B   e3a0-2=1P score/10 (BCD)    e72e-f=2P paddle (lo-hi)
e008=level=2*(shown_level-1)+x <- remember it's a binary tree (42 last)
e7f0=country code(from 9fde in sound rom)
e807=counter, reset by sound cpu, increased by main cpu each vblank
e80b=test progress=0(start) 1(first 8) 2(all ok) 3(error)
ec09-a~=ed05-6=xy pos of cursor in hi-scores
ec81-eca8=hi-scores(8bytes*5entries)

addr    bit name        active  addr    bit name        active
e72d    6   coin[1]     low     e729    1   2p select   low
        5   service     high            0   1p select   low
        4   coin[2]     low

addr    bit name        active  addr    bit name        active
e730    7   tilt        low     e7e7    4   1p fire     low
                                        0   2p fire     low

            Interesting routines (sound cpu)
            --------------------------------

4ae check starts    B73,B7a,B81,B99 coin related
8c1 check coins     62e lockout check       664 dsw check

            Interesting locations (sound cpu)
            ---------------------------------

d006=each bit is on if a corresponding location (e880-e887) has changed
d00b=(c001)>>4=tilt if 0E (security sequence must be reset?)
addr    bit name        active
d00c    7   tilt
        6   ?service?
        5   coin2       low
        4   coin1       low

d00d=each bit is on if the corresponding location (e880-e887) is 1 (OK)
d00e=each of the 4 MSBs is on if ..
d00f=FF if tilt, 00 otherwise
d011=if 00 checks counter, if FF doesn't
d23f=input port 1 value

***************************************************************************/
/***************************************************************************

Kageki
(c) 1988 Taito Corporation

Driver by Takahiro Nogi (nogi@kt.rim.or.jp) 1999/11/06

***************************************************************************/

/***************************************************************************
  Jumping Pop
   - added by David Haywood, thanks to Robin Cooper

  This is not a Taito board, it's a copy, it contains no original Taito or
   Seta components.

  The game is a bootleg of Plump Pop produced by NICs of Korea, it has new
   graphics, sounds etc.

  Uses a YM2151 instead of the YM2203, has no MCU

  Uses Palette RAM instead of PROMs.

  ToDo:

  Palette format (or gfx decode?) appears to be incorrect, some colours
   clearly don't match the screenshot.

  Frequencies have not been measured

  The bar behind the players flickers first boot / first attract levels, bug?

  Inputs might be wrong, should be joystick not spinner? (can't select character)

***************************************************************************/

#include "emu.h"
#include "includes/tnzs.h"
#include "includes/taitoipt.h"

#include "cpu/z80/z80.h"
#include "sound/2203intf.h"
#include "sound/volt_reg.h"
#include "sound/ym2151.h"
#include "screen.h"
#include "speaker.h"


SAMPLES_START_CB_MEMBER(kageki_state::init_samples)
{
	uint8_t *src = memregion("samples")->base() + 0x0090;
	for (int i = 0; i < MAX_SAMPLES; i++)
	{
		int start = (src[(i * 2) + 1] * 256) + src[(i * 2)];
		uint8_t *scan = &src[start];
		int size = 0;

		// check sample length
		while (*scan++ != 0x00)
			size++;

		/* 2009-11 FP: should these be saved? */
		m_sampledata[i] = std::make_unique<int16_t[]>(size);
		m_samplesize[i] = size;

		if (start < 0x100)
			start = size = 0;

		// signed 8-bit sample to unsigned 8-bit sample convert
		int16_t *dest = m_sampledata[i].get();
		scan = &src[start];
		for (int n = 0; n < size; n++)
		{
			*dest++ = (int8_t)((*scan++) ^ 0x80) * 256;
		}
	//  logerror("samples num:%02X ofs:%04X lng:%04X\n", i, start, size);
	}
}


READ8_MEMBER(kageki_state::csport_r)
{
	int dsw, dsw1, dsw2;

	dsw1 = m_dswa->read();
	dsw2 = m_dswb->read();

	switch (m_csport_sel)
	{
		case    0x00:           // DSW2 5,1 / DSW1 5,1
			dsw = (((dsw2 & 0x10) >> 1) | ((dsw2 & 0x01) << 2) | ((dsw1 & 0x10) >> 3) | ((dsw1 & 0x01) >> 0));
			break;
		case    0x01:           // DSW2 7,3 / DSW1 7,3
			dsw = (((dsw2 & 0x40) >> 3) | ((dsw2 & 0x04) >> 0) | ((dsw1 & 0x40) >> 5) | ((dsw1 & 0x04) >> 2));
			break;
		case    0x02:           // DSW2 6,2 / DSW1 6,2
			dsw = (((dsw2 & 0x20) >> 2) | ((dsw2 & 0x02) << 1) | ((dsw1 & 0x20) >> 4) | ((dsw1 & 0x02) >> 1));
			break;
		case    0x03:           // DSW2 8,4 / DSW1 8,4
			dsw = (((dsw2 & 0x80) >> 4) | ((dsw2 & 0x08) >> 1) | ((dsw1 & 0x80) >> 6) | ((dsw1 & 0x08) >> 3));
			break;
		default:
			dsw = 0x00;
		//  logerror("csport_sel error !! (0x%08X)\n", m_csport_sel);
	}

	return (dsw & 0xff);
}

WRITE8_MEMBER(kageki_state::csport_w)
{
	char mess[80];

	if (data > 0x3f)
	{
		// read dipsw port
		m_csport_sel = (data & 0x03);
	}
	else
	{
		if (data > MAX_SAMPLES)
		{
			// stop samples
			m_samples->stop(0);
			sprintf(mess, "VOICE:%02X STOP", data);
		}
		else
		{
			// play samples
			m_samples->start_raw(0, m_sampledata[data].get(), m_samplesize[data], 7000);
			sprintf(mess, "VOICE:%02X PLAY", data);
		}
	//  popmessage(mess);
	}
}

static ADDRESS_MAP_START( main_map, AS_PROGRAM, 8, tnzs_base_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x8000, 0xbfff) AM_DEVICE("mainbank", address_map_bank_device, amap8)
	AM_RANGE(0xc000, 0xcfff) AM_RAM AM_DEVREADWRITE("spritegen", seta001_device, spritecodelow_r8, spritecodelow_w8)
	AM_RANGE(0xd000, 0xdfff) AM_RAM AM_DEVREADWRITE("spritegen", seta001_device, spritecodehigh_r8, spritecodehigh_w8)
	AM_RANGE(0xe000, 0xefff) AM_RAM AM_SHARE("share1")
	AM_RANGE(0xf000, 0xf2ff) AM_RAM AM_DEVREADWRITE("spritegen", seta001_device, spriteylow_r8, spriteylow_w8)
	AM_RANGE(0xf300, 0xf303) AM_MIRROR(0xfc) AM_DEVWRITE("spritegen", seta001_device, spritectrl_w8)  /* control registers (0x80 mirror used by Arkanoid 2) */
	AM_RANGE(0xf400, 0xf400) AM_DEVWRITE("spritegen", seta001_device, spritebgflag_w8)   /* enable / disable background transparency */
	AM_RANGE(0xf600, 0xf600) AM_READNOP AM_WRITE(ramrom_bankswitch_w)
	AM_RANGE(0xf800, 0xfbff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
ADDRESS_MAP_END

static ADDRESS_MAP_START( prompal_main_map, AS_PROGRAM, 8, extrmatn_state )
	/* arknoid2, extrmatn, plumppop and drtoppel have PROMs instead of RAM */
	/* drtoppel writes here anyway! (maybe leftover from tests during development) */
	AM_RANGE(0xf800, 0xfbff) AM_WRITENOP
	AM_IMPORT_FROM(main_map)
ADDRESS_MAP_END

static ADDRESS_MAP_START( tnzsb_main_map, AS_PROGRAM, 8, tnzsb_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x8000, 0xbfff) AM_DEVICE("mainbank", address_map_bank_device, amap8)
	AM_RANGE(0xc000, 0xcfff) AM_RAM AM_DEVREADWRITE("spritegen", seta001_device, spritecodelow_r8, spritecodelow_w8)
	AM_RANGE(0xd000, 0xdfff) AM_RAM AM_DEVREADWRITE("spritegen", seta001_device, spritecodehigh_r8, spritecodehigh_w8)
	AM_RANGE(0xe000, 0xefff) AM_RAM AM_SHARE("share1")
	AM_RANGE(0xf000, 0xf2ff) AM_RAM AM_DEVREADWRITE("spritegen", seta001_device, spriteylow_r8, spriteylow_w8)
	AM_RANGE(0xf300, 0xf303) AM_MIRROR(0xfc) AM_DEVWRITE("spritegen", seta001_device, spritectrl_w8) /* control registers (0x80 mirror used by Arkanoid 2) */
	AM_RANGE(0xf400, 0xf400) AM_DEVWRITE("spritegen", seta001_device, spritebgflag_w8)   /* enable / disable background transparency */
	AM_RANGE(0xf600, 0xf600) AM_WRITE(ramrom_bankswitch_w)
	/* kabukiz still writes here but it's not used (it's paletteram in type1 map) */
	AM_RANGE(0xf800, 0xfbff) AM_WRITENOP
ADDRESS_MAP_END

static ADDRESS_MAP_START( base_sub_map, AS_PROGRAM, 8, tnzs_base_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x8000, 0x9fff) AM_ROMBANK("subbank")
	AM_RANGE(0xa000, 0xa000) AM_WRITE(bankswitch1_w)
	AM_RANGE(0xb000, 0xb001) AM_DEVREADWRITE("ymsnd", ym2203_device, read, write)
	AM_RANGE(0xd000, 0xdfff) AM_RAM
	AM_RANGE(0xe000, 0xefff) AM_RAM AM_SHARE("share1")
ADDRESS_MAP_END

static ADDRESS_MAP_START( tnzs_sub_map, AS_PROGRAM, 8, tnzs_mcu_state )
	AM_RANGE(0xc000, 0xc001) AM_READWRITE(mcu_r, mcu_w)   /* not present in insectx */
	AM_RANGE(0xa000, 0xa000) AM_WRITE(bankswitch1_w)
	AM_RANGE(0xf000, 0xf003) AM_READ(analog_r) /* paddles in arkanoid2/plumppop. The ports are */
											   /* read but not used by the other games, and are not read at */
											   /* all by insectx. */
	AM_IMPORT_FROM(base_sub_map)
ADDRESS_MAP_END

static ADDRESS_MAP_START( arknoid2_sub_map, AS_PROGRAM, 8, arknoid2_state )
	AM_RANGE(0xc000, 0xc001) AM_READWRITE(mcu_r, mcu_w)
	AM_IMPORT_FROM(tnzs_sub_map)
ADDRESS_MAP_END

static ADDRESS_MAP_START( kageki_sub_map, AS_PROGRAM, 8, kageki_state )
	AM_RANGE(0xc000, 0xc000) AM_READ_PORT("IN0")
	AM_RANGE(0xc001, 0xc001) AM_READ_PORT("IN1")
	AM_RANGE(0xc002, 0xc002) AM_READ_PORT("IN2")
	AM_IMPORT_FROM(base_sub_map)
ADDRESS_MAP_END

static ADDRESS_MAP_START( insectx_sub_map, AS_PROGRAM, 8, insectx_state )
	AM_RANGE(0xc000, 0xc000) AM_READ_PORT("IN0")
	AM_RANGE(0xc001, 0xc001) AM_READ_PORT("IN1")
	AM_RANGE(0xc002, 0xc002) AM_READ_PORT("IN2")
	AM_IMPORT_FROM(base_sub_map)
ADDRESS_MAP_END

/* the later board is different, it has a third CPU (and of course no mcu) */

static ADDRESS_MAP_START( tnzsb_base_sub_map, AS_PROGRAM, 8, tnzsb_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x8000, 0x9fff) AM_ROMBANK("subbank")
	AM_RANGE(0xa000, 0xa000) AM_WRITE(bankswitch1_w)
	AM_RANGE(0xb002, 0xb002) AM_READ_PORT("DSWA")
	AM_RANGE(0xb003, 0xb003) AM_READ_PORT("DSWB")
	AM_RANGE(0xb004, 0xb004) AM_WRITE(sound_command_w)
	AM_RANGE(0xc000, 0xc000) AM_READ_PORT("IN0")
	AM_RANGE(0xc001, 0xc001) AM_READ_PORT("IN1")
	AM_RANGE(0xc002, 0xc002) AM_READ_PORT("IN2")
	AM_RANGE(0xd000, 0xdfff) AM_RAM
	AM_RANGE(0xe000, 0xefff) AM_RAM AM_SHARE("share1")
ADDRESS_MAP_END

static ADDRESS_MAP_START( tnzsb_sub_map, AS_PROGRAM, 8, tnzsb_state )
	AM_RANGE(0xf000, 0xf003) AM_READONLY
	AM_RANGE(0xf000, 0xf3ff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_IMPORT_FROM(tnzsb_base_sub_map)
ADDRESS_MAP_END

static ADDRESS_MAP_START( kabukiz_sub_map, AS_PROGRAM, 8, kabukiz_state )
	AM_RANGE(0xf800, 0xfbff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
	AM_IMPORT_FROM(tnzsb_base_sub_map)
ADDRESS_MAP_END

static ADDRESS_MAP_START( tnzsb_cpu2_map, AS_PROGRAM, 8, tnzsb_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0xc000, 0xdfff) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START( kabukiz_cpu2_map, AS_PROGRAM, 8, kabukiz_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x8000, 0xbfff) AM_ROMBANK("audiobank")
	AM_RANGE(0xe000, 0xffff) AM_RAM
ADDRESS_MAP_END

static ADDRESS_MAP_START( tnzsb_io_map, AS_IO, 8, tnzsb_state )
	ADDRESS_MAP_GLOBAL_MASK(0xff)
	AM_RANGE(0x00, 0x01) AM_DEVREADWRITE("ymsnd", ym2203_device, read, write)
	AM_RANGE(0x02, 0x02) AM_DEVREAD("soundlatch", generic_latch_8_device, read)
ADDRESS_MAP_END

static ADDRESS_MAP_START( jpopnics_main_map, AS_PROGRAM, 8, jpopnics_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x8000, 0xbfff) AM_DEVICE("mainbank", address_map_bank_device, amap8)
	AM_RANGE(0xc000, 0xcfff) AM_RAM AM_DEVREADWRITE("spritegen", seta001_device, spritecodelow_r8, spritecodelow_w8)
	AM_RANGE(0xd000, 0xdfff) AM_RAM AM_DEVREADWRITE("spritegen", seta001_device, spritecodehigh_r8, spritecodehigh_w8)
	AM_RANGE(0xe000, 0xefff) AM_RAM AM_SHARE("share1") /* WORK RAM (shared by the 2 z80's) */
	AM_RANGE(0xf000, 0xf2ff) AM_RAM AM_DEVREADWRITE("spritegen", seta001_device, spriteylow_r8, spriteylow_w8)
	AM_RANGE(0xf300, 0xf303) AM_MIRROR(0xfc) AM_DEVWRITE("spritegen", seta001_device, spritectrl_w8) /* control registers (0x80 mirror used by Arkanoid 2) */
	AM_RANGE(0xf400, 0xf400) AM_DEVWRITE("spritegen", seta001_device, spritebgflag_w8)   /* enable / disable background transparency */
	AM_RANGE(0xf600, 0xf600) AM_READNOP AM_WRITE(ramrom_bankswitch_w)
	AM_RANGE(0xf800, 0xffff) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette")
ADDRESS_MAP_END

static ADDRESS_MAP_START( jpopnics_sub_map, AS_PROGRAM, 8, jpopnics_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
	AM_RANGE(0x8000, 0x9fff) AM_ROMBANK("subbank")

	AM_RANGE(0xa000, 0xa000) AM_WRITE(subbankswitch_w)
	AM_RANGE(0xb000, 0xb001) AM_DEVREADWRITE("ymsnd", ym2151_device, read, write)
	AM_RANGE(0xc000, 0xc000) AM_READ_PORT("IN1")
	AM_RANGE(0xc001, 0xc001) AM_READ_PORT("IN2")
	AM_RANGE(0xc600, 0xc600) AM_READ_PORT("DSWA")
	AM_RANGE(0xc601, 0xc601) AM_READ_PORT("DSWB")

	AM_RANGE(0xd000, 0xdfff) AM_RAM
	AM_RANGE(0xe000, 0xefff) AM_RAM AM_SHARE("share1")

	AM_RANGE(0xf000, 0xf003) AM_DEVREAD("upd4701", upd4701_device, read_xy)
ADDRESS_MAP_END

/* RAM/ROM bank that maps at 0x8000-0xbfff on maincpu */
static ADDRESS_MAP_START( mainbank_map, AS_PROGRAM, 8, tnzs_base_state )
	AM_RANGE(0x00000, 0x07fff) AM_RAM // instead of the first two banks of ROM being repeated redundantly the hardware maps RAM here
	AM_RANGE(0x08000, 0x1ffff) AM_ROM AM_REGION(":maincpu", 0x8000)
ADDRESS_MAP_END

MACHINE_CONFIG_START( tnzs_mainbank )
	MCFG_DEVICE_ADD("mainbank", ADDRESS_MAP_BANK, 0)
	MCFG_DEVICE_PROGRAM_MAP(mainbank_map)
	MCFG_ADDRESS_MAP_BANK_ENDIANNESS(ENDIANNESS_LITTLE)
	MCFG_ADDRESS_MAP_BANK_DATABUS_WIDTH(8)
	MCFG_ADDRESS_MAP_BANK_ADDRBUS_WIDTH(17)
	MCFG_ADDRESS_MAP_BANK_STRIDE(0x4000)
MACHINE_CONFIG_END

#define COMMON_IN2\
	PORT_START("IN2")\
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_SERVICE1 )\
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_TILT )\
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNKNOWN )\
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNKNOWN )\
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNKNOWN )\
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN )\
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )\
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

static INPUT_PORTS_START( plumppop )
	/* 0xb001 (CPU1) port 0 -> 0xef0e (shared RAM) */
	PORT_START("DSWA")
	PORT_DIPNAME( 0x01, 0x01, DEF_STR( Unknown ) ) PORT_DIPLOCATION("SWA:1") /* code at 0x6e99 - is it ever called ? */
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	TAITO_DSWA_BITS_1_TO_3_LOC(SWA)
	TAITO_COINAGE_JAPAN_OLD_LOC(SWA)

	/* 0xb001 (CPU1) port 1 -> 0xef0f (shared RAM) */
	PORT_START("DSWB")
	TAITO_DIFFICULTY_LOC(SWB)
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Bonus_Life ) ) PORT_DIPLOCATION("SWB:3,4") /* table at 0x2b86 */
	PORT_DIPSETTING(    0x08, "50k 200k 150k+" )
	PORT_DIPSETTING(    0x0c, "50k 250k 200k+" )
	PORT_DIPSETTING(    0x04, "100k 300k 200k+" )
	PORT_DIPSETTING(    0x00, "100k 400k 300k+" )
	PORT_DIPNAME( 0x30, 0x30, DEF_STR( Lives ) ) PORT_DIPLOCATION("SWB:5,6")
	PORT_DIPSETTING(    0x20, "2" )
	PORT_DIPSETTING(    0x30, "3" )
	PORT_DIPSETTING(    0x10, "4" )
	PORT_DIPSETTING(    0x00, "5" )
	PORT_DIPNAME( 0x40, 0x40, "P1 & P2 Children Collision" ) PORT_DIPLOCATION("SWB:7") /* code at 0x3dcc */
	PORT_DIPSETTING(    0x40, DEF_STR( No ) )  // both players' children collide with each other / Off=No / On=Yes
	PORT_DIPSETTING(    0x00, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Allow_Continue ) ) PORT_DIPLOCATION("SWB:8")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Yes ) )

	PORT_START("IN0")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(1) PORT_NAME("P1 Button 2 (Cheat)")
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_START1 )

	PORT_START("IN1")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2) PORT_NAME("P2 Button 2 (Cheat)")    /* not working ? */
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_START2 )

	PORT_START("IN2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_TILT )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x10, IP_ACTIVE_HIGH, IPT_COIN1 )
	PORT_BIT( 0x20, IP_ACTIVE_HIGH, IPT_COIN2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )

	PORT_START("COIN1")
	PORT_BIT( 1, IP_ACTIVE_HIGH, IPT_COIN1 )

	PORT_START("COIN2")
	PORT_BIT( 1, IP_ACTIVE_HIGH, IPT_COIN2 )

	PORT_START("AN1")       /* spinner 1 - read at f000/1 */
	PORT_BIT( 0x0fff, 0x0000, IPT_DIAL ) PORT_SENSITIVITY(70) PORT_KEYDELTA(15) PORT_PLAYER(1) PORT_RESET

	PORT_START("AN2")       /* spinner 2 - read at f002/3 */
	PORT_BIT( 0x0fff, 0x0000, IPT_DIAL ) PORT_SENSITIVITY(70) PORT_KEYDELTA(15) PORT_PLAYER(2) PORT_RESET
INPUT_PORTS_END


static INPUT_PORTS_START( extrmatn )
	/* 0xb001 (CPU1) port 0 -> 0xef0e (shared RAM) */
	PORT_START("DSWA")
	PORT_DIPUNUSED_DIPLOC( 0x01, IP_ACTIVE_LOW, "SWA:1" )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Flip_Screen ) ) PORT_DIPLOCATION("SWA:2")
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_SERVICE_DIPLOC( 0x04, IP_ACTIVE_LOW, "SWA:3" )
	PORT_DIPUNUSED_DIPLOC( 0x08, IP_ACTIVE_LOW, "SWA:4" )
	TAITO_COINAGE_JAPAN_OLD_LOC(SWA)

	/* 0xb001 (CPU1) port 1 -> 0xef0f (shared RAM) */
	PORT_START("DSWB")
	TAITO_DIFFICULTY_LOC(SWB)
	PORT_DIPUNUSED_DIPLOC( 0x04, IP_ACTIVE_LOW, "SWB:3" )
	PORT_DIPUNUSED_DIPLOC( 0x08, IP_ACTIVE_LOW, "SWB:4" )
	PORT_DIPUNUSED_DIPLOC( 0x10, IP_ACTIVE_LOW, "SWB:5" )
	PORT_DIPUNUSED_DIPLOC( 0x20, IP_ACTIVE_LOW, "SWB:6" )
	PORT_DIPNAME( 0xc0, 0xc0, "Damage Multiplier" ) PORT_DIPLOCATION("SWB:7,8")
	PORT_DIPSETTING(    0xc0, "*1" )
	PORT_DIPSETTING(    0x80, "*1.5" )
	PORT_DIPSETTING(    0x40, "*2" )
	PORT_DIPSETTING(    0x00, "*3" )

	PORT_START("IN0")
	TAITO_JOY_LRUD_2_BUTTONS_START( 1 )

	PORT_START("IN1")
	TAITO_JOY_LRUD_2_BUTTONS_START( 2 )

	COMMON_IN2

	PORT_START("COIN1")
	PORT_BIT( 1, IP_ACTIVE_HIGH, IPT_COIN1 )

	PORT_START("COIN2")
	PORT_BIT( 1, IP_ACTIVE_HIGH, IPT_COIN2 )
INPUT_PORTS_END


static INPUT_PORTS_START( arknoid2 )
	/* 0xb001 (CPU1) port 0 -> 0xe001 (shared RAM) */
	PORT_START("DSWA")
	TAITO_MACHINE_COCKTAIL_LOC(SWA)
	TAITO_COINAGE_WORLD_LOC(SWA)

	/* 0xb001 (CPU1) port 1 -> 0xe002 (shared RAM) */
	PORT_START("DSWB")
	TAITO_DIFFICULTY_LOC(SWB)
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Bonus_Life ) ) PORT_DIPLOCATION("SWB:3,4") /* table at 0x6f1e (W and J) or 0x6f1b (Romstar) */
	PORT_DIPSETTING(    0x00, "50k 150k 150k+" )
	PORT_DIPSETTING(    0x0c, "100k 200k 200k+" )
	PORT_DIPSETTING(    0x04, "50k only" )
	PORT_DIPSETTING(    0x08, "100k only" )
	PORT_DIPNAME( 0x30, 0x30, DEF_STR( Lives ) ) PORT_DIPLOCATION("SWB:5,6")
	PORT_DIPSETTING(    0x20, "2" )
	PORT_DIPSETTING(    0x30, "3" )
	PORT_DIPSETTING(    0x10, "4" )
	PORT_DIPSETTING(    0x00, "5" )
	PORT_DIPUNUSED_DIPLOC( 0x40, IP_ACTIVE_LOW, "SWB:7" )
	PORT_DIPNAME( 0x80, 0x00, DEF_STR( Allow_Continue ) ) PORT_DIPLOCATION("SWB:8")
	PORT_DIPSETTING(    0x80, DEF_STR( No ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Yes ) )

	PORT_START("IN0")       /* read at c000 (sound cpu) */
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_START2 )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_START1 )

	PORT_START("IN1")
	PORT_BIT( 0xff, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("IN2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_SERVICE1 ) PORT_WRITE_LINE_DEVICE_MEMBER("upd4701", upd4701_device, right_w)
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_TILT )
	PORT_BIT( 0xfc, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("COIN1")
	PORT_BIT( 1, IP_ACTIVE_HIGH, IPT_COIN1 ) PORT_WRITE_LINE_DEVICE_MEMBER("upd4701", upd4701_device, left_w)

	PORT_START("COIN2")
	PORT_BIT( 1, IP_ACTIVE_HIGH, IPT_COIN2 ) PORT_WRITE_LINE_DEVICE_MEMBER("upd4701", upd4701_device, middle_w)

	PORT_START("AN1")       /* spinner 1 - read at f000/1 */
	PORT_BIT( 0x0fff, 0x0000, IPT_DIAL ) PORT_SENSITIVITY(70) PORT_KEYDELTA(15) PORT_PLAYER(1) PORT_RESET

	PORT_START("AN2")       /* spinner 2 - read at f002/3 */
	PORT_BIT( 0x0fff, 0x0000, IPT_DIAL ) PORT_SENSITIVITY(70) PORT_KEYDELTA(15) PORT_PLAYER(2) PORT_RESET
INPUT_PORTS_END

static INPUT_PORTS_START( arknid2u )
	PORT_INCLUDE( arknoid2 )

	PORT_MODIFY("DSWA")
	TAITO_COINAGE_JAPAN_OLD_LOC(SWA)
INPUT_PORTS_END


static INPUT_PORTS_START( drtoppel )
	/* 0xb001 (CPU1) port 0 -> 0xe042 (shared RAM) */
	PORT_START("DSWA")
	TAITO_MACHINE_COCKTAIL_LOC(SWA)
	TAITO_COINAGE_WORLD_LOC(SWA)

	/* 0xb001 (CPU1) port 1 -> 0xe043 (shared RAM) */
	PORT_START("DSWB")
	TAITO_DIFFICULTY_LOC(SWB)
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Bonus_Life ) ) PORT_DIPLOCATION("SWB:3,4") /* table at 0x256d (4 * 9 bytes) */
	PORT_DIPSETTING(    0x0c, "30k 100k 200k 100k+" )             /* 30k  100k  200k  300k  400k  500k  600k  700k */
	PORT_DIPSETTING(    0x00, "50k 100k 200k 200k+" )             /* 50k  100k  200k  400k  600k  800k 1000k 1200k */
	PORT_DIPSETTING(    0x04, "30k 100k" )
	PORT_DIPSETTING(    0x08, "30k only" )
	PORT_DIPNAME( 0x30, 0x30, DEF_STR( Lives ) ) PORT_DIPLOCATION("SWB:5,6")
	PORT_DIPSETTING(    0x20, "2" )
	PORT_DIPSETTING(    0x30, "3" )
	PORT_DIPSETTING(    0x10, "4" )
	PORT_DIPSETTING(    0x00, "5" )
	PORT_DIPUNUSED_DIPLOC( 0x40, IP_ACTIVE_LOW, "SWB:7" )
	PORT_DIPUNUSED_DIPLOC( 0x80, IP_ACTIVE_LOW, "SWB:8" )

	PORT_START("IN0")
	TAITO_JOY_LRUD_2_BUTTONS_START( 1 )

	PORT_START("IN1")
	TAITO_JOY_LRUD_2_BUTTONS_START( 2 )

	COMMON_IN2

	PORT_START("COIN1")
	PORT_BIT( 1, IP_ACTIVE_HIGH, IPT_COIN1 )

	PORT_START("COIN2")
	PORT_BIT( 1, IP_ACTIVE_HIGH, IPT_COIN2 )
INPUT_PORTS_END

static INPUT_PORTS_START( drtopplu )
	PORT_INCLUDE( drtoppel )

	PORT_MODIFY("DSWA")
	TAITO_COINAGE_JAPAN_OLD_LOC(SWA)
INPUT_PORTS_END


static INPUT_PORTS_START( kageki )
	/* special (see kageki_csport_* handlers) -> 0xe03b (shared RAM) */
	PORT_START("DSWA")
	TAITO_MACHINE_NO_COCKTAIL_LOC(SWA)                           /* see notes */
	TAITO_COINAGE_WORLD_LOC(SWA)

	/* special (see kageki_csport_* handlers) -> 0xe03c (shared RAM) */
	PORT_START("DSWB")
	TAITO_DIFFICULTY_LOC(SWB)
	PORT_DIPUNUSED_DIPLOC( 0x04, IP_ACTIVE_LOW, "SWB:3" )
	PORT_DIPUNUSED_DIPLOC( 0x08, IP_ACTIVE_LOW, "SWB:4" )
	PORT_DIPUNUSED_DIPLOC( 0x10, IP_ACTIVE_LOW, "SWB:5" )
	PORT_DIPUNUSED_DIPLOC( 0x20, IP_ACTIVE_LOW, "SWB:6" )
	PORT_DIPUNUSED_DIPLOC( 0x40, IP_ACTIVE_LOW, "SWB:7" )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Allow_Continue ) ) PORT_DIPLOCATION("SWB:8")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Yes ) )

	PORT_START("IN0")
	TAITO_JOY_LRUD_2_BUTTONS_START( 1 )

	PORT_START("IN1")
	TAITO_JOY_LRUD_2_BUTTONS_START( 2 )

	PORT_START("IN2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_TILT )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )
INPUT_PORTS_END

static INPUT_PORTS_START( kagekiu )
	PORT_INCLUDE( kageki )

	PORT_MODIFY("DSWA")
	TAITO_COINAGE_JAPAN_OLD_LOC(SWA)
INPUT_PORTS_END

static INPUT_PORTS_START( kagekij )
	PORT_INCLUDE( kagekiu )

	PORT_MODIFY("DSWA")
	TAITO_MACHINE_COCKTAIL_LOC(SWA)                              /* see notes */
INPUT_PORTS_END


static INPUT_PORTS_START( chukatai )
	/* 0xb001 (CPU1) port 0 -> 0xe015 (shared RAM) */
	PORT_START("DSWA")
	TAITO_MACHINE_COCKTAIL_LOC(SWA)
	TAITO_COINAGE_WORLD_LOC(SWA)

	/* 0xb001 (CPU1) port 1 -> 0xe016 (shared RAM) */
	PORT_START("DSWB")
	TAITO_DIFFICULTY_LOC(SWB)
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Bonus_Life ) ) PORT_DIPLOCATION("SWB:3,4") /* tables : 1st at 0xb070 (bank = 0x02) and inc. at 0x09df */
	PORT_DIPSETTING(    0x08, "40k 240k 200k+" )
	PORT_DIPSETTING(    0x0c, "60k 360k 300k+" )
	PORT_DIPSETTING(    0x04, "100k 500k 400k+" )
	PORT_DIPSETTING(    0x00, "150k 650k 500k+" )
	PORT_DIPNAME( 0x30, 0x30, DEF_STR( Lives ) ) PORT_DIPLOCATION("SWB:5,6")
	PORT_DIPSETTING(    0x00, "1" )
	PORT_DIPSETTING(    0x10, "2" )
	PORT_DIPSETTING(    0x30, "3" )
	PORT_DIPSETTING(    0x20, "4" )
	PORT_DIPUNUSED_DIPLOC( 0x40, IP_ACTIVE_LOW, "SWB:7" )
	PORT_DIPUNUSED_DIPLOC( 0x80, IP_ACTIVE_LOW, "SWB:8" )

	PORT_START("IN0")
	TAITO_JOY_LRUD_2_BUTTONS_START( 1 )

	PORT_START("IN1")
	TAITO_JOY_LRUD_2_BUTTONS_START( 2 )

	COMMON_IN2

	PORT_START("COIN1")
	PORT_BIT( 1, IP_ACTIVE_HIGH, IPT_COIN1 )

	PORT_START("COIN2")
	PORT_BIT( 1, IP_ACTIVE_HIGH, IPT_COIN2 )
INPUT_PORTS_END

static INPUT_PORTS_START( chukatau )
	PORT_INCLUDE( chukatai )

	PORT_MODIFY("DSWA")
	TAITO_COINAGE_JAPAN_OLD_LOC(SWA)
INPUT_PORTS_END


static INPUT_PORTS_START( tnzs )
	/* 0xb002 (CPU1) -> 0xef0e (shared RAM) */
	PORT_START("DSWA")
	PORT_DIPNAME( 0x01, 0x00, DEF_STR( Cabinet ) ) PORT_DIPLOCATION("SWA:1")
	PORT_DIPSETTING(    0x00, DEF_STR( Upright ) )
	PORT_DIPSETTING(    0x01, DEF_STR( Cocktail ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Flip_Screen ) ) PORT_DIPLOCATION("SWA:2")
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_SERVICE_DIPLOC( 0x04, IP_ACTIVE_LOW, "SWA:3" )
	PORT_DIPNAME( 0x08, 0x08, "Invulnerability (Debug)" ) PORT_DIPLOCATION("SWA:4") // see notes
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	TAITO_COINAGE_WORLD_LOC(SWA)

	/* 0xb003 (CPU1) -> 0xef0f (shared RAM) */
	PORT_START("DSWB")
	TAITO_DIFFICULTY_LOC(SWB)
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Bonus_Life ) ) PORT_DIPLOCATION("SWB:3,4") /* table at 0x09c84 */
	PORT_DIPSETTING(    0x00, "50k 150k 150k+" )
	PORT_DIPSETTING(    0x0c, "70k 200k 200k+" )
	PORT_DIPSETTING(    0x04, "100k 250k 250k+" )
	PORT_DIPSETTING(    0x08, "200k 300k 300k+" )
	PORT_DIPNAME( 0x30, 0x30, DEF_STR( Lives ) ) PORT_DIPLOCATION("SWB:5,6")
	PORT_DIPSETTING(    0x20, "2" )
	PORT_DIPSETTING(    0x30, "3" )
	PORT_DIPSETTING(    0x00, "4" )
	PORT_DIPSETTING(    0x10, "5" )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Allow_Continue ) ) PORT_DIPLOCATION("SWB:7")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Yes ) )
	PORT_DIPUNUSED_DIPLOC( 0x80, IP_ACTIVE_LOW, "SWB:8" )

	PORT_START("IN0")
	TAITO_JOY_LRUD_2_BUTTONS_START( 1 )

	PORT_START("IN1")
	TAITO_JOY_LRUD_2_BUTTONS_START( 2 )

	PORT_START("IN2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_TILT )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )
INPUT_PORTS_END

static INPUT_PORTS_START( tnzsj )
	PORT_INCLUDE( tnzs )

	PORT_MODIFY("DSWA")
	TAITO_COINAGE_JAPAN_OLD_LOC(SWA)
INPUT_PORTS_END

static INPUT_PORTS_START( tnzsjo )
	/* 0xb001 (CPU1) port 0 -> 0xef0e (shared RAM) */
	PORT_START("DSWA")
	PORT_DIPNAME( 0x01, 0x00, DEF_STR( Cabinet ) ) PORT_DIPLOCATION("SWA:1")
	PORT_DIPSETTING(    0x00, DEF_STR( Upright ) )
	PORT_DIPSETTING(    0x01, DEF_STR( Cocktail ) )
	PORT_DIPNAME( 0x02, 0x02, DEF_STR( Flip_Screen ) ) PORT_DIPLOCATION("SWA:2")
	PORT_DIPSETTING(    0x02, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	PORT_SERVICE_DIPLOC( 0x04, IP_ACTIVE_LOW, "SWA:3" )
	PORT_DIPNAME( 0x08, 0x08, "Invulnerability (Debug)" ) PORT_DIPLOCATION("SWA:4") // see notes
	PORT_DIPSETTING(    0x08, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	TAITO_COINAGE_JAPAN_OLD_LOC(SWA)

	/* 0xb001 (CPU1) port 1 -> 0xef0f (shared RAM) */
	PORT_START("DSWB")
	TAITO_DIFFICULTY_LOC(SWB)
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Bonus_Life ) ) PORT_DIPLOCATION("SWB:3,4") /* table at 0x09caf */
	PORT_DIPSETTING(    0x00, "50k 150k 150k+" )
	PORT_DIPSETTING(    0x0c, "70k 200k 200k+" )
	PORT_DIPSETTING(    0x04, "100k 250k 250k+" )
	PORT_DIPSETTING(    0x08, "200k 300k 300k+" )
	PORT_DIPNAME( 0x30, 0x30, DEF_STR( Lives ) ) PORT_DIPLOCATION("SWB:5,6")
	PORT_DIPSETTING(    0x20, "2" )
	PORT_DIPSETTING(    0x30, "3" )
	PORT_DIPSETTING(    0x00, "4" )
	PORT_DIPSETTING(    0x10, "5" )
	PORT_DIPNAME( 0x40, 0x40, DEF_STR( Allow_Continue ) ) PORT_DIPLOCATION("SWB:7")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x40, DEF_STR( Yes ) )
	PORT_DIPUNUSED_DIPLOC( 0x80, IP_ACTIVE_LOW, "SWB:8" )

	PORT_START("IN0")
	TAITO_JOY_LRUD_2_BUTTONS_START( 1 )

	PORT_START("IN1")
	TAITO_JOY_LRUD_2_BUTTONS_START( 2 )

	COMMON_IN2

	PORT_START("COIN1")
	PORT_BIT( 1, IP_ACTIVE_LOW, IPT_COIN1 )

	PORT_START("COIN2")
	PORT_BIT( 1, IP_ACTIVE_LOW, IPT_COIN2 )
INPUT_PORTS_END

static INPUT_PORTS_START( tnzsop )
	PORT_INCLUDE( tnzsjo )

	PORT_MODIFY("DSWA")
	PORT_DIPUNUSED_DIPLOC( 0x08, IP_ACTIVE_LOW, "SWA:4" )        /* value read at 0x356b but not tested nor stored elsewhere */
	TAITO_COINAGE_WORLD_LOC(SWA)

	PORT_MODIFY("DSWB")
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Bonus_Life ) ) PORT_DIPLOCATION("SWB:3,4") /* table at 0x09afb */
	PORT_DIPSETTING(    0x00, "10k 100k 100k+" )
	PORT_DIPSETTING(    0x0c, "10k 150k 150k+" )
	PORT_DIPSETTING(    0x04, "10k 200k 200k+" )
	PORT_DIPSETTING(    0x08, "10k 300k 300k+" )
INPUT_PORTS_END


static INPUT_PORTS_START( kabukiz )
	/* 0xb002 (CPU1) */
	PORT_START("DSWA")
	TAITO_MACHINE_COCKTAIL_LOC(SWA)
	PORT_DIPUNUSED_DIPLOC( 0x10, IP_ACTIVE_LOW, "SWA:5" )
	PORT_DIPUNUSED_DIPLOC( 0x20, IP_ACTIVE_LOW, "SWA:6" )
	PORT_DIPUNUSED_DIPLOC( 0x40, IP_ACTIVE_LOW, "SWA:7" )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Allow_Continue ) ) PORT_DIPLOCATION("SWA:8")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Yes ) )

	/* 0xb003 (CPU1) */
	PORT_START("DSWB")
	PORT_DIPNAME( 0x03, 0x02, DEF_STR( Difficulty ) ) PORT_DIPLOCATION("SWB:1,2") // different from many other taito
	PORT_DIPSETTING(    0x03, DEF_STR( Easy ) )   // other taito : [ On Off ] < [ Off Off ] < [ Off On ] < [ On On ]
	PORT_DIPSETTING(    0x02, DEF_STR( Medium ) ) //     kabukiz : [ Off Off ] < [ On Off ] < [ Off On ] < [ On On ]
	PORT_DIPSETTING(    0x01, DEF_STR( Hard ) )
	PORT_DIPSETTING(    0x00, DEF_STR( Hardest ) )
	PORT_DIPUNUSED_DIPLOC( 0x04, IP_ACTIVE_LOW, "SWB:3" )
	PORT_DIPUNUSED_DIPLOC( 0x08, IP_ACTIVE_LOW, "SWB:4" )
	TAITO_COINAGE_WORLD_LOC(SWB)

	PORT_START("IN0")
	TAITO_JOY_LRUD_2_BUTTONS_START( 1 )

	PORT_START("IN1")
	TAITO_JOY_LRUD_2_BUTTONS_START( 2 )

	PORT_START("IN2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )
INPUT_PORTS_END

static INPUT_PORTS_START( kabukizj )
	PORT_INCLUDE( kabukiz )

	PORT_MODIFY("DSWB")
	TAITO_COINAGE_JAPAN_OLD_LOC(SWB)
INPUT_PORTS_END


static INPUT_PORTS_START( insectx )
	/* 0xb001 (CPU1) port 0 -> 0xec06 (shared RAM) */
	PORT_START("DSWA")
	TAITO_MACHINE_COCKTAIL_LOC(SWA)
	TAITO_COINAGE_WORLD_LOC(SWA)

	/* 0xb001 (CPU1) port 1 -> 0xec07 (shared RAM) */
	PORT_START("DSWB")
	PORT_DIPNAME( 0x03, 0x03, DEF_STR( Difficulty ) ) PORT_DIPLOCATION("SWB:1,2") /* see notes */
	PORT_DIPSETTING(    0x01, DEF_STR( Easy ) )   // other taito : [ On Off ] < [ Off Off ] < [ Off On ] < [ On On ]
	PORT_DIPSETTING(    0x03, DEF_STR( Medium ) ) //       notes : [ Off On ] < [ Off Off ] < [ On Off ] < [ On On ]
	PORT_DIPSETTING(    0x02, DEF_STR( Hard ) )   //      manual : [ Off Off ] < [ On Off ] < [ Off On ] < [ On On ]
	PORT_DIPSETTING(    0x00, DEF_STR( Hardest ) )
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Bonus_Life ) ) PORT_DIPLOCATION("SWB:3,4") /* code at 0xaacc - bank = 0x02 */
	PORT_DIPSETTING(    0x08, "40k 240k 200k+" )
	PORT_DIPSETTING(    0x0c, "60k 360k 300k+" )
	PORT_DIPSETTING(    0x04, "100k 500k 400k+" )
	PORT_DIPSETTING(    0x00, "150k 650k 500k+" )
	PORT_DIPNAME( 0x30, 0x30, DEF_STR( Lives ) ) PORT_DIPLOCATION("SWB:5,6")
	PORT_DIPSETTING(    0x00, "1" )
	PORT_DIPSETTING(    0x10, "2" )
	PORT_DIPSETTING(    0x30, "3" )
	PORT_DIPSETTING(    0x20, "4" )
	PORT_DIPUNUSED_DIPLOC( 0x40, IP_ACTIVE_LOW, "SWB:7" )
	PORT_DIPUNUSED_DIPLOC( 0x80, IP_ACTIVE_LOW, "SWB:8" )

	PORT_START("IN0")
	TAITO_JOY_LRUD_2_BUTTONS_START( 1 )

	PORT_START("IN1")
	TAITO_JOY_LRUD_2_BUTTONS_START( 2 )

	PORT_START("IN2")
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_SERVICE1 )
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_TILT )
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNKNOWN )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNKNOWN )
INPUT_PORTS_END

static INPUT_PORTS_START( insectxj )
	PORT_INCLUDE( insectx )

	PORT_MODIFY("DSWA")
	TAITO_COINAGE_JAPAN_OLD_LOC(SWA)
INPUT_PORTS_END


static INPUT_PORTS_START( jpopnics )
	/* 0xc600 (CPU1) -> 0xef0e (shared RAM) */
	PORT_START("DSWA")
	PORT_DIPNAME( 0x01, 0x01, DEF_STR( Unknown ) ) PORT_DIPLOCATION("SWA:1") /* code at 0x6e99 - is it ever called ? */
	PORT_DIPSETTING(    0x01, DEF_STR( Off ) )
	PORT_DIPSETTING(    0x00, DEF_STR( On ) )
	TAITO_DSWA_BITS_1_TO_3
	PORT_DIPUNUSED_DIPLOC( 0x10, IP_ACTIVE_LOW, "SWA:5" )        /* see notes */
	PORT_DIPUNUSED_DIPLOC( 0x20, IP_ACTIVE_LOW, "SWA:6" )        /* see notes */
	PORT_DIPUNUSED_DIPLOC( 0x40, IP_ACTIVE_LOW, "SWA:7" )        /* see notes */
	PORT_DIPUNUSED_DIPLOC( 0x80, IP_ACTIVE_LOW, "SWA:8" )        /* see notes */

	/* 0xc601 (CPU1) -> 0xef0f (shared RAM) */
	PORT_START("DSWB")
	TAITO_DIFFICULTY_LOC(SWB)
	PORT_DIPNAME( 0x0c, 0x0c, DEF_STR( Bonus_Life ) ) PORT_DIPLOCATION("SWB:3,4") /* table at 0x2b86 */
	PORT_DIPSETTING(    0x08, "50k 200k 150k+" )
	PORT_DIPSETTING(    0x0c, "50k 250k 200k+" )
	PORT_DIPSETTING(    0x04, "100k 300k 200k+" )
	PORT_DIPSETTING(    0x00, "100k 400k 300k+" )
	PORT_DIPNAME( 0x30, 0x30, DEF_STR( Lives ) ) PORT_DIPLOCATION("SWB:5,6")
	PORT_DIPSETTING(    0x20, "2" )
	PORT_DIPSETTING(    0x30, "3" )
	PORT_DIPSETTING(    0x10, "4" )
	PORT_DIPSETTING(    0x00, "5" )
	PORT_DIPNAME( 0x40, 0x40, "P1 & P2 Children Collision" ) PORT_DIPLOCATION("SWB:7") /* code at 0x3dcc */
	PORT_DIPSETTING(    0x40, DEF_STR( No ) )  // both players' children collide with each other / Off=No / On=Yes
	PORT_DIPSETTING(    0x00, DEF_STR( Yes ) )
	PORT_DIPNAME( 0x80, 0x80, DEF_STR( Allow_Continue ) ) PORT_DIPLOCATION("SWB:8")
	PORT_DIPSETTING(    0x00, DEF_STR( No ) )
	PORT_DIPSETTING(    0x80, DEF_STR( Yes ) )

	PORT_START("IN0")
	PORT_BIT( 0xff, IP_ACTIVE_LOW, IPT_UNUSED )

	PORT_START("IN1")
	PORT_BIT( 0x0f, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(1)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(1) PORT_NAME("P1 Button 2 (Cheat)")
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_COIN1 )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_START1 )

	PORT_START("IN2")
	PORT_BIT( 0x0f, IP_ACTIVE_LOW, IPT_UNUSED )
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_BUTTON1 ) PORT_PLAYER(2)
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_BUTTON2 ) PORT_PLAYER(2) PORT_NAME("P2 Button 2 (Cheat)")    /* not working ? */
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_COIN2 )
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_START2 )

	PORT_START("AN1")       /* spinner 1 - read at f000/1 */
	PORT_BIT( 0x0fff, 0x0000, IPT_DIAL ) PORT_SENSITIVITY(70) PORT_KEYDELTA(15) PORT_PLAYER(1) PORT_RESET

	PORT_START("AN2")       /* spinner 2 - read at f002/3 */
	PORT_BIT( 0x0fff, 0x0000, IPT_DIAL ) PORT_SENSITIVITY(70) PORT_KEYDELTA(15) PORT_PLAYER(2) PORT_RESET
INPUT_PORTS_END


static const gfx_layout tnzs_charlayout =
{
	16,16,
	RGN_FRAC(1,4),
	4,
	{ RGN_FRAC(3,4), RGN_FRAC(2,4), RGN_FRAC(1,4), RGN_FRAC(0,4) },
	{ 0, 1, 2, 3, 4, 5, 6, 7,
			8*8+0, 8*8+1, 8*8+2, 8*8+3, 8*8+4, 8*8+5, 8*8+6, 8*8+7 },
	{ 0*8, 1*8, 2*8, 3*8, 4*8, 5*8, 6*8, 7*8,
			16*8, 17*8, 18*8, 19*8, 20*8, 21*8, 22*8, 23*8 },
	32*8
};

static const gfx_layout insectx_charlayout =
{
	16,16,
	8192,
	4,
	{ 8, 0, 8192*64*8+8, 8192*64*8 },
	{ 0, 1, 2, 3, 4, 5, 6, 7,
			8*16+0, 8*16+1, 8*16+2, 8*16+3, 8*16+4, 8*16+5, 8*16+6, 8*16+7 },
	{ 0*16, 1*16, 2*16, 3*16, 4*16, 5*16, 6*16, 7*16,
		16*16, 17*16, 18*16, 19*16, 20*16, 21*16, 22*16, 23*16 },
	64*8
};

static GFXDECODE_START( tnzs )
	GFXDECODE_ENTRY( "gfx1", 0, tnzs_charlayout, 0, 32 )
GFXDECODE_END

static GFXDECODE_START( insectx )
	GFXDECODE_ENTRY( "gfx1", 0, insectx_charlayout, 0, 32 )
GFXDECODE_END

static MACHINE_CONFIG_START( tnzs_base )
	/* basic machine hardware */
	MCFG_CPU_ADD("maincpu", Z80,XTAL_12MHz/2)       /* 6.0 MHz ??? - Main board Crystal is 12MHz, verified on insectx, kageki, tnzsb */
	MCFG_CPU_PROGRAM_MAP(main_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", tnzs_base_state,  irq0_line_hold)

	MCFG_CPU_ADD("sub", Z80,XTAL_12MHz/2)       /* 6.0 MHz ??? - Main board Crystal is 12MHz, verified on insectx, kageki, tnzsb */
	MCFG_CPU_PROGRAM_MAP(base_sub_map)
	MCFG_CPU_VBLANK_INT_DRIVER("screen", tnzs_base_state,  irq0_line_hold)

	MCFG_FRAGMENT_ADD(tnzs_mainbank)

	MCFG_QUANTUM_PERFECT_CPU("maincpu")

	/* video hardware */
	MCFG_DEVICE_ADD("spritegen", SETA001_SPRITE, 0)
	MCFG_SETA001_SPRITE_GFXDECODE("gfxdecode")

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(32*8, 32*8)
	MCFG_SCREEN_VISIBLE_AREA(0*8, 32*8-1, 2*8, 30*8-1)
	MCFG_SCREEN_UPDATE_DRIVER(tnzs_base_state, screen_update_tnzs)
	MCFG_SCREEN_VBLANK_CALLBACK(WRITELINE(tnzs_base_state, screen_vblank_tnzs))
	MCFG_SCREEN_PALETTE("palette")

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", tnzs)
	MCFG_PALETTE_ADD("palette", 512)
	MCFG_PALETTE_FORMAT(xRRRRRGGGGGBBBBB)

	/* sound hardware */
	MCFG_SPEAKER_STANDARD_MONO("speaker")
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( tnzs, tnzs_base )
	MCFG_CPU_ADD("mcu", I8742, 12000000/2)  /* 400KHz ??? - Main board Crystal is 12MHz */
	MCFG_MCS48_PORT_P1_IN_CB(READ8(tnzs_mcu_state, mcu_port1_r))
	MCFG_MCS48_PORT_P2_IN_CB(READ8(tnzs_mcu_state, mcu_port2_r))
	MCFG_MCS48_PORT_P2_OUT_CB(WRITE8(tnzs_mcu_state, mcu_port2_w))
	MCFG_MCS48_PORT_T0_IN_CB(IOPORT("COIN1"))
	MCFG_MCS48_PORT_T1_IN_CB(IOPORT("COIN2"))

	MCFG_CPU_MODIFY("sub")
	MCFG_CPU_PROGRAM_MAP(tnzs_sub_map)

	/* video hardware */
	MCFG_SCREEN_MODIFY("screen")
	MCFG_SCREEN_REFRESH_RATE(59.15)   /* it should be the same as the newer pcb vsync */
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(2500) /* not accurate */)

	/* sound hardware */
	MCFG_SOUND_ADD("ymsnd", YM2203, XTAL_12MHz/4)
	MCFG_AY8910_PORT_A_READ_CB(IOPORT("DSWA"))
	MCFG_AY8910_PORT_B_READ_CB(IOPORT("DSWB"))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.3)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( extrmatn, tnzs )
	/* basic machine hardware */
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(prompal_main_map)

	/* video hardware */
	MCFG_SCREEN_MODIFY("screen")
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))

	MCFG_PALETTE_MODIFY("palette")
	MCFG_PALETTE_INIT_OWNER(tnzs_base_state, prompalette)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( plumppop, extrmatn )
	MCFG_DEVICE_ADD("upd4701", UPD4701A, 0)
	MCFG_UPD4701_PORTX("AN1")
	MCFG_UPD4701_PORTY("AN2")
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( arknoid2, plumppop )
	/* basic machine hardware */
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_VBLANK_INT_DRIVER("screen", arknoid2_state, mcu_interrupt)

	MCFG_CPU_MODIFY("sub")
	MCFG_CPU_PROGRAM_MAP(arknoid2_sub_map)

	MCFG_CPU_MODIFY("mcu")
	MCFG_DEVICE_DISABLE()
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( insectx, tnzs_base )
	/* basic machine hardware */
	MCFG_CPU_MODIFY("sub")
	MCFG_CPU_PROGRAM_MAP(insectx_sub_map)

	/* video hardware */
	MCFG_GFXDECODE_MODIFY("gfxdecode", insectx)

	/* sound hardware */
	MCFG_SOUND_ADD("ymsnd", YM2203, XTAL_12MHz/4) /* verified on pcb */
	MCFG_AY8910_PORT_A_READ_CB(IOPORT("DSWA"))
	MCFG_AY8910_PORT_B_READ_CB(IOPORT("DSWB"))
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.3)
MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( kageki, tnzs_base )
	/* basic machine hardware */
	MCFG_CPU_MODIFY("sub")
	MCFG_CPU_PROGRAM_MAP(kageki_sub_map)

	/* sound hardware */
	MCFG_SOUND_ADD("ymsnd", YM2203, XTAL_12MHz/4) /* verified on pcb */
	MCFG_AY8910_PORT_A_READ_CB(READ8(kageki_state, csport_r))
	MCFG_AY8910_PORT_B_WRITE_CB(WRITE8(kageki_state, csport_w))
	MCFG_SOUND_ROUTE(0, "speaker", 0.15)
	MCFG_SOUND_ROUTE(1, "speaker", 0.15)
	MCFG_SOUND_ROUTE(2, "speaker", 0.15)
	MCFG_SOUND_ROUTE(3, "speaker", 0.35)

	MCFG_SOUND_ADD("samples", SAMPLES, 0)
	MCFG_SAMPLES_CHANNELS(1)
	MCFG_SAMPLES_START_CB(kageki_state, init_samples)
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 1.0)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( tnzsb, tnzs_base )
	/* basic machine hardware */
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(tnzsb_main_map)

	MCFG_CPU_MODIFY("sub") /* verified on pcb */
	MCFG_CPU_PROGRAM_MAP(tnzsb_sub_map)

	MCFG_CPU_ADD("audiocpu", Z80, XTAL_12MHz/2) /* verified on pcb */
	MCFG_CPU_PROGRAM_MAP(tnzsb_cpu2_map)
	MCFG_CPU_IO_MAP(tnzsb_io_map)

	/* video hardware */
	MCFG_SCREEN_MODIFY("screen")
	MCFG_SCREEN_REFRESH_RATE(59.15)   /* verified on pcb */

	/* sound hardware */
	MCFG_GENERIC_LATCH_8_ADD("soundlatch")

	MCFG_SOUND_ADD("ymsnd", YM2203, XTAL_12MHz/4) /* verified on pcb */
	MCFG_YM2203_IRQ_HANDLER(WRITELINE(tnzsb_state, ym2203_irqhandler))
	MCFG_SOUND_ROUTE(0, "speaker", 1.0)
	MCFG_SOUND_ROUTE(1, "speaker", 1.0)
	MCFG_SOUND_ROUTE(2, "speaker", 1.0)
	MCFG_SOUND_ROUTE(3, "speaker", 2.0)
MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( kabukiz, tnzsb )

	/* basic machine hardware */
	MCFG_CPU_MODIFY("sub")
	MCFG_CPU_PROGRAM_MAP(kabukiz_sub_map)

	MCFG_CPU_MODIFY("audiocpu")
	MCFG_CPU_PROGRAM_MAP(kabukiz_cpu2_map)

	/* sound hardware */
	MCFG_SOUND_MODIFY("ymsnd")
	MCFG_AY8910_PORT_A_WRITE_CB(WRITE8(kabukiz_state, sound_bank_w))
	MCFG_AY8910_PORT_B_WRITE_CB(DEVWRITE8("dac", dac_byte_interface, write))

	MCFG_SOUND_ADD("dac", DAC_8BIT_R2R, 0) MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.5) // unknown DAC
	MCFG_DEVICE_ADD("vref", VOLTAGE_REGULATOR, 0) MCFG_VOLTAGE_REGULATOR_OUTPUT(5.0)
	MCFG_SOUND_ROUTE_EX(0, "dac", 1.0, DAC_VREF_POS_INPUT) MCFG_SOUND_ROUTE_EX(0, "dac", -1.0, DAC_VREF_NEG_INPUT)
MACHINE_CONFIG_END


static MACHINE_CONFIG_DERIVED( jpopnics, tnzs_base )
	/* basic machine hardware */
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(jpopnics_main_map)

	MCFG_CPU_MODIFY("sub")
	MCFG_CPU_PROGRAM_MAP(jpopnics_sub_map)

	MCFG_DEVICE_ADD("upd4701", UPD4701A, 0)
	MCFG_UPD4701_PORTX("AN1")
	MCFG_UPD4701_PORTY("AN2")

	/* video hardware */
	MCFG_PALETTE_MODIFY("palette")
	MCFG_PALETTE_ENTRIES(1024)
	MCFG_PALETTE_FORMAT(GGGGBBBBRRRRxxxx) /* wrong, the other 4 bits seem to be used as well */
	MCFG_PALETTE_ENDIANNESS(ENDIANNESS_BIG)

	/* sound hardware */
	MCFG_YM2151_ADD("ymsnd", XTAL_12MHz/4) /* Not verified - Main board Crystal is 12MHz */
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "speaker", 0.3)
MACHINE_CONFIG_END

/***************************************************************************

  PCBs

***************************************************************************/
/*  The TNZS/Seta hardware has a variety of somewhat different pcbs, all of
    which have both Seta and Taito Part numbers.
    All pcbs have Z80B processors and one 6264 mainram chip and an X1-001
    and X1-002 video chip and an X1-004 I/O? Chip, and four PALs

Seta#       Taito#s             CPUS    RxM2    ROM1    MCU?    Video ram   PROMs   SETA X1 GFXROMs     QUADRATURE  ESD. PROT   Games                           Picture
P0-022-A    K1100245A J1100108A 2xZ80B  512/256 512/256 8042    4x6116      Yes, 2  03      23c1000     uPD4701AC   3x X2-003*4 arkanoid2                       http://www.classicarcaderesource.com/RevengeOfDoh3.jpg
P0-022-B    K1100234A J1100108A 2xZ80B  512/256 512/256 8042    4x6116      Yes, 2  03      27c512(A)   uPD4701AC   3x X2-003*4 plumppop                        N/A
P0-025-A    K1100241A J1100107A 2xZ80B  512/256 512/256 8042    4x6116      Yes, 2  03      23c1000     N/A         3x X2-003   drtoppel,extermatn,chukatai(B)  http://arcade.ym2149.com/pcb/taito/drtoppel_pcb_partside.jpg
P0-028-A    K1100416A J1100332A 2xZ80B  512/256 512/256 8042    4x6116      No      05,06   23c1000     N/A         3x X2-004   chukatai(B)                     http://i.ebayimg.com/images/g/AhoAAOSw-FZXj5A5/s-l1600.jpg
P0-038A     M6100309A           3xZ80B  512/256 512/256 NONE    1x6164      No      05,06   23c1000     N/A         3x X2-003   kageki                          http://i.ebayimg.com/images/a/(KGrHqJ,!lwE6C8-G97lBOjOu9mwVw~~/s-l1600.jpg
P0-041-1    CA403001A           2xZ80B  61256   27c1000 8742    1x6164      No      05,06   27c1000     N/A         5x X2-005   tnzsop(C)                       http://arcade.ym2149.com/pcb/taito/tnzs_pcb3_partside.jpg
P0-041-A    K1100356A J1100156A 2xZ80B  61256   27c1000 8042    1x6164      No      05,06   23c1000     N/A         5x X2-005   tnzs(j,u)o                      http://arcade.ym2149.com/pcb/taito/tnzs_pcb1_partside.jpg
P0-043A     M6100356A           3xZ80B* 61256   27512** NONE    1x6164      No      05,06   LH534000*   N/A         4x X2-004   tnzs(j,u), kabukiz              http://arcade.ym2149.com/pcb/taito/tnzs_pcb2_mainboard_partside.jpg
P0-056A     K1100476A J1100201A 3xZ80B  EMPTY*3 27c1000 NONE    1x6164      No      05,06   LH534000    N/A         5x X2-005   insectx(D)                      http://www.jammarcade.net/images/2014/04/InsectorX.jpg

(A) GFX rom mapping is slightly different to P0-022-A pcb, possibly configured
    by a jumper.
(B) chukatai has one set which unlike its earlier sets uses the P0-025-A
    PCB, but with a daughterboard which converts four of the 23c1000 gfx ROM
    sockets into 8 27c1000 eprom sockets, and DOES use color PROMs!
    The other pcb set uses P0-028-A pcb and 23c1000 mask roms and color RAM,
    but has lower rom id numbers. The higher numbered set was likely created
    by Taito to 'use up' a stock of older P0-025-A pcbs.
(C) This is a development/prototype PCB, hence it has 32 pin sockets for the
    gfx ROMs as 27c1000 eproms, instead of 28 pin sockets for 23c1000 mask
    ROMs. It also uses an (unprotected?) 8742 MCU.
    Another curious thing is the Taito ID number may have accidentally been
    printed in backwards order, i.e should be C1100304A which fits the pattern
    of the other boards.
(D) InsectorX has a lot of rework on its PCB, two greenwires for each of the
    two LH534000 mask ROMs, and four wires connected to the X1-004 I/O chip
    pins 18, 19, 20, and 21, connecting it to the 4 input pins of a Toshiba
    TD62064AP Darlington driver @ U43.
*   tnzs(j,u) uses a sub board with a z80b and 23c1000 mask ROMs on it for gfx,
    plugged into the four LH534000 mask ROM sockets and the 2nd z80 socket.
    Like Kageki's P0-038A mainboard, this mainboard has a third z80 on it which
    acts in place of the 8x42 mcu used by the older tnzs sets.
**  This is a 28-pin 27512 in a 32-pin socket which alternately holds a 27c1000.
*3  This is unpopulated, but the pcb can accept a 61256 SRAM here.
*4  arkanoid2 and plumppop lack all but one or two buttons, so two of the three
    ESD protection modules are unpopulated.
*/


/***************************************************************************

  Game driver(s)

***************************************************************************/

ROM_START( plumppop )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "a98__09.11c", 0x00000, 0x10000, CRC(107f9e06) SHA1(0aa7f32721c3cab96eccc7c831b9f57877c4e1dc) )
	ROM_LOAD( "a98__10.9c", 0x10000, 0x10000, CRC(df6e6af2) SHA1(792f97f587e84cdd67f0d1efe1fd13ea904d7e20) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "a98__11.4e", 0x00000, 0x10000, CRC(bc56775c) SHA1(0c22c22c0e9d7ec0e34f8ab4bfe61068f65e8759) )

	ROM_REGION( 0x10000, "mcu", 0 )    /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b06__14.1g", 0x0000, 0x0800, CRC(28907072) SHA1(21c7017af8a8ceb8e43d7e798f48518b136fd45c) ) /* Labeled B06-14 and under printed label "Taito M-001, 128P, 720100", is a mask 8042 */

	ROM_REGION( 0x100000, "gfx1", 0 )
	ROM_LOAD( "a98__01.mbm27c512.13a", 0x00000, 0x10000, CRC(f3033dca) SHA1(130744998f0531a82de2814231dddea3ad710f60) )
	ROM_RELOAD(             0x10000, 0x10000 )
	ROM_LOAD( "a98__02.mbm27c512.12a", 0x20000, 0x10000, CRC(f2d17b0c) SHA1(418c8e383b8d4d54d723ae3512829a95e6897ee1) )
	ROM_RELOAD(             0x30000, 0x10000 )
	ROM_LOAD( "a98__03.mbm27c512.10a", 0x40000, 0x10000, CRC(1a519b0a) SHA1(9217c6bf564ccd4a44f9cf2045102e667dc0b036) )
	ROM_RELOAD(             0x40000, 0x10000 )
	ROM_LOAD( "a98__04.mbm27c512.8a", 0x60000, 0x10000, CRC(b64501a1) SHA1(6d96172b7d7d2276787013fe6b47bb7fef0a4e36) )
	ROM_RELOAD(             0x70000, 0x10000 )
	ROM_LOAD( "a98__05.mbm27c512.7a", 0x80000, 0x10000, CRC(45c36963) SHA1(2f23bff22e218f542c50bf7e4ae8ab6db93180b0) )
	ROM_RELOAD(             0x90000, 0x10000 )
	ROM_LOAD( "a98__06.mbm27c512.5a", 0xa0000, 0x10000, CRC(e075341b) SHA1(b5e68b5da7933c7eff21fa832e089edcbb49cdb4) )
	ROM_RELOAD(             0xb0000, 0x10000 )
	ROM_LOAD( "a98__07.mbm27c512.4a", 0xc0000, 0x10000, CRC(8e16cd81) SHA1(6bc9dc8e29197b75c3c4ac4f066037bb9b8cebb4) )
	ROM_RELOAD(             0xd0000, 0x10000 )
	ROM_LOAD( "a98__08.mbm27c512.2a", 0xe0000, 0x10000, CRC(bfa7609a) SHA1(0b9aa89b5954334f40dda1f14b1691852c74fc37) )
	ROM_RELOAD(             0xf0000, 0x10000 )

	ROM_REGION( 0x0400, "proms", 0 )        /* color proms */
	ROM_LOAD( "a98-13.15f", 0x0000, 0x200, CRC(7cde2da5) SHA1(0cccfc35fb716ebb4cffa85c75681f33ca80a56e) )   /* hi bytes, AM27S29 or compatible like MB7124 */
	ROM_LOAD( "a98-12.17f", 0x0200, 0x200, CRC(90dc9da7) SHA1(f719dead7f4597e5ee6f1103599505b98cb58299) )   /* lo bytes, AM27S29 or compatible like MB7124 */

	ROM_REGION( 0x10000, "pal", 0 ) /* pals on plumppop are the same set as arkanoid2/extrmatn/drtoppel/chukataio/etc with the exception of d9? */
	ROM_LOAD( "b06-10-1.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

ROM_START( jpopnics )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "u96cpu2", 0x00000, 0x20000, CRC(649e951c) SHA1(b26bb157da9fcf5d3eddbb637a4cb2cb1b0fedac) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "u124cpu1", 0x00000, 0x10000,  CRC(8453e8e4) SHA1(aac1bd501a15f79e3ed566c949504169b2aa762d) )

	ROM_REGION( 0x100000, "gfx1", 0 )
	ROM_LOAD( "u94gfx", 0x00000, 0x10000, CRC(e49f2fdd) SHA1(6824c6520d0039c062f028e69cbfa7b3790ea756) )
	ROM_CONTINUE(       0x20000, 0x10000 )
	ROM_LOAD( "u93gfx", 0x40000, 0x10000, CRC(a7791b5b) SHA1(4abfe9b2612ed0d17f1282a60879cf1d0620ae4c) )
	ROM_CONTINUE(       0x60000, 0x10000 )
	ROM_LOAD( "u92gfx", 0x80000, 0x10000, CRC(b30caac7) SHA1(a434f67e1bec9848d9c3e184734d8cebee048176) )
	ROM_CONTINUE(       0xa0000, 0x10000 )
	ROM_LOAD( "u91gfx", 0xc0000, 0x10000, CRC(18ada5f2) SHA1(3307dd11e5cd0d0abe8b7751a5fbf54998558b34) )
	ROM_CONTINUE(       0xe0000, 0x10000 )
ROM_END

ROM_START( extrmatn )
	ROM_REGION( 0x20000, "maincpu", 0 )             /* Region 0 - main cpu */
	ROM_LOAD( "b06-05.11c", 0x00000, 0x10000, CRC(918e1fe3) SHA1(1aa69e7ae393f275d440b3d5bf817475e443045d) )
	ROM_LOAD( "b06-06.9c",  0x10000, 0x10000, CRC(8842e105) SHA1(68675e77801504c5f67f82fae42f55152ffadebe) )

	ROM_REGION( 0x10000, "sub", 0 )             /* Region 2 - sound cpu */
	ROM_LOAD( "b06-19.4e", 0x00000, 0x10000, CRC(8de43ed9) SHA1(53e6d8fa93889c38733d169e983f2caf1da71f43) )

	ROM_REGION( 0x10000, "mcu", 0 )    /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b06__14.1g", 0x0000, 0x0800, CRC(28907072) SHA1(21c7017af8a8ceb8e43d7e798f48518b136fd45c) ) /* Labeled B06-14 and under printed label "Taito M-001, 128P, 720100", is a mask 8042 */

	ROM_REGION( 0x80000, "gfx1", 0 )
	ROM_LOAD( "b06-01.13a", 0x00000, 0x20000, CRC(d2afbf7e) SHA1(28b4cf94798f049a9f8375464741dbef208d7290) )
	ROM_LOAD( "b06-02.10a", 0x20000, 0x20000, CRC(e0c2757a) SHA1(3c89044caa28b10b4d1bef1515881810c23d312a) )
	ROM_LOAD( "b06-03.7a",  0x40000, 0x20000, CRC(ee80ab9d) SHA1(f4e4833cadff7d856b5a8075a61d902427653e16) )
	ROM_LOAD( "b06-04.4a",  0x60000, 0x20000, CRC(3697ace4) SHA1(6d6e4e64147365bcfcf74a84eb7ae84dffedd304) )

	ROM_REGION( 0x0400, "proms", 0 )
	ROM_LOAD( "b06-09.15f", 0x00000, 0x200, CRC(f388b361) SHA1(f00db6ad6994cfe9b7ad76e30b7049b11f8c16e4) )  /* hi bytes, AM27S29 or compatible like MB7124 */
	ROM_LOAD( "b06-08.17f", 0x00200, 0x200, CRC(10c9aac3) SHA1(09d6f791dea358e78099af7a370b00b8504ffc97) )  /* lo bytes, AM27S29 or compatible like MB7124 */

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with several other games on this hardware */
	ROM_LOAD( "b06-10.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

ROM_START( extrmatnu )
	ROM_REGION( 0x20000, "maincpu", 0 )             /* Region 0 - main cpu */
	ROM_LOAD( "b06-20.11c", 0x00000, 0x10000, CRC(04e3fc1f) SHA1(b1cf2f79f43fa33d6175368c897f84ec6aa6e746) )
	ROM_LOAD( "b06-21.9c",  0x10000, 0x10000, CRC(1614d6a2) SHA1(f23d465af231ab5653c55748f686d8f25f52394b) )

	ROM_REGION( 0x10000, "sub", 0 )             /* Region 2 - sound cpu */
	ROM_LOAD( "b06-22.4e", 0x00000, 0x10000, CRC(744f2c84) SHA1(7565c1594c2a3bae1ae45afcbf93363fe2b12d58) )

	ROM_REGION( 0x10000, "mcu", 0 )    /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b06__14.1g", 0x0000, 0x0800, CRC(28907072) SHA1(21c7017af8a8ceb8e43d7e798f48518b136fd45c) ) /* Labeled B06-14 and under printed label "Taito M-001, 128P, 720100", is a mask 8042 */

	ROM_REGION( 0x80000, "gfx1", 0 )
	ROM_LOAD( "b06-01.13a", 0x00000, 0x20000, CRC(d2afbf7e) SHA1(28b4cf94798f049a9f8375464741dbef208d7290) )
	ROM_LOAD( "b06-02.10a", 0x20000, 0x20000, CRC(e0c2757a) SHA1(3c89044caa28b10b4d1bef1515881810c23d312a) )
	ROM_LOAD( "b06-03.7a",  0x40000, 0x20000, CRC(ee80ab9d) SHA1(f4e4833cadff7d856b5a8075a61d902427653e16) )
	ROM_LOAD( "b06-04.4a",  0x60000, 0x20000, CRC(3697ace4) SHA1(6d6e4e64147365bcfcf74a84eb7ae84dffedd304) )

	ROM_REGION( 0x0400, "proms", 0 )
	ROM_LOAD( "b06-09.15f", 0x00000, 0x200, CRC(f388b361) SHA1(f00db6ad6994cfe9b7ad76e30b7049b11f8c16e4) )  /* hi bytes, AM27S29 or compatible like MB7124 */
	ROM_LOAD( "b06-08.17f", 0x00200, 0x200, CRC(10c9aac3) SHA1(09d6f791dea358e78099af7a370b00b8504ffc97) )  /* lo bytes, AM27S29 or compatible like MB7124 */

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with several other games on this hardware */
	ROM_LOAD( "b06-10.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

ROM_START( extrmatnur )
	ROM_REGION( 0x20000, "maincpu", 0 )             /* Region 0 - main cpu */
	ROM_LOAD( "b06_15",  0x00000, 0x10000, CRC(4b3ee597) SHA1(024964faebd0fa894ab4868a8e009267e828cbfb) )
	ROM_LOAD( "b06_16",  0x10000, 0x10000, CRC(86175ea4) SHA1(0f30cbb1a6a32355528543707799f752a1b9b75e) )

	ROM_REGION( 0x10000, "sub", 0 )             /* Region 2 - sound cpu */
	ROM_LOAD( "b06_17", 0x00000, 0x10000, CRC(744f2c84) SHA1(7565c1594c2a3bae1ae45afcbf93363fe2b12d58) )

	ROM_REGION( 0x10000, "mcu", 0 )    /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b06__14.1g", 0x0000, 0x0800, CRC(28907072) SHA1(21c7017af8a8ceb8e43d7e798f48518b136fd45c) ) /* Labeled B06-14 and under printed label "Taito M-001, 128P, 720100", is a mask 8042 */

	ROM_REGION( 0x80000, "gfx1", 0 )
	ROM_LOAD( "b06-01.13a", 0x00000, 0x20000, CRC(d2afbf7e) SHA1(28b4cf94798f049a9f8375464741dbef208d7290) )
	ROM_LOAD( "b06-02.10a", 0x20000, 0x20000, CRC(e0c2757a) SHA1(3c89044caa28b10b4d1bef1515881810c23d312a) )
	ROM_LOAD( "b06-03.7a",  0x40000, 0x20000, CRC(ee80ab9d) SHA1(f4e4833cadff7d856b5a8075a61d902427653e16) )
	ROM_LOAD( "b06-04.4a",  0x60000, 0x20000, CRC(3697ace4) SHA1(6d6e4e64147365bcfcf74a84eb7ae84dffedd304) )

	ROM_REGION( 0x0400, "proms", 0 )
	ROM_LOAD( "b06-09.15f", 0x00000, 0x200, CRC(f388b361) SHA1(f00db6ad6994cfe9b7ad76e30b7049b11f8c16e4) )  /* hi bytes, AM27S29 or compatible like MB7124 */
	ROM_LOAD( "b06-08.17f", 0x00200, 0x200, CRC(10c9aac3) SHA1(09d6f791dea358e78099af7a370b00b8504ffc97) )  /* lo bytes, AM27S29 or compatible like MB7124 */

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with several other games on this hardware */
	ROM_LOAD( "b06-10.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

ROM_START( extrmatnj )
	ROM_REGION( 0x20000, "maincpu", 0 )             /* Region 0 - main cpu */
	ROM_LOAD( "b06-05.11c", 0x00000, 0x10000, CRC(918e1fe3) SHA1(1aa69e7ae393f275d440b3d5bf817475e443045d) )
	ROM_LOAD( "b06-06.9c",  0x10000, 0x10000, CRC(8842e105) SHA1(68675e77801504c5f67f82fae42f55152ffadebe) )

	ROM_REGION( 0x10000, "sub", 0 )             /* Region 2 - sound cpu */
	ROM_LOAD( "b06-07.4e", 0x00000, 0x10000, CRC(b37fb8b3) SHA1(10696914b9e39d34d56069a69b9d641339ea2309) )

	ROM_REGION( 0x10000, "mcu", 0 )    /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b06__14.1g", 0x0000, 0x0800, CRC(28907072) SHA1(21c7017af8a8ceb8e43d7e798f48518b136fd45c) ) /* Labeled B06-14 and under printed label "Taito M-001, 128P, 720100", is a mask 8042 */

	ROM_REGION( 0x80000, "gfx1", 0 )
	ROM_LOAD( "b06-01.13a", 0x00000, 0x20000, CRC(d2afbf7e) SHA1(28b4cf94798f049a9f8375464741dbef208d7290) )
	ROM_LOAD( "b06-02.10a", 0x20000, 0x20000, CRC(e0c2757a) SHA1(3c89044caa28b10b4d1bef1515881810c23d312a) )
	ROM_LOAD( "b06-03.7a",  0x40000, 0x20000, CRC(ee80ab9d) SHA1(f4e4833cadff7d856b5a8075a61d902427653e16) )
	ROM_LOAD( "b06-04.4a",  0x60000, 0x20000, CRC(3697ace4) SHA1(6d6e4e64147365bcfcf74a84eb7ae84dffedd304) )

	ROM_REGION( 0x0400, "proms", 0 )
	ROM_LOAD( "b06-09.15f", 0x00000, 0x200, CRC(f388b361) SHA1(f00db6ad6994cfe9b7ad76e30b7049b11f8c16e4) )  /* hi bytes, AM27S29 or compatible like MB7124 */
	ROM_LOAD( "b06-08.17f", 0x00200, 0x200, CRC(10c9aac3) SHA1(09d6f791dea358e78099af7a370b00b8504ffc97) )  /* lo bytes, AM27S29 or compatible like MB7124 */

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with several other games on this hardware */
	ROM_LOAD( "b06-10.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

/*
PCB:
Seta: P0-022-A
Taito: K1100245A J1100108A

The arknoid2 PCB has a sticker label which says "K1100245A // REVENGE OF DOH"
*/

ROM_START( arknoid2 )
	ROM_REGION( 0x20000, "maincpu", 0 )             /* Region 0 - main cpu */
	ROM_LOAD( "b08__05.11c", 0x00000, 0x10000, CRC(136edf9d) SHA1(f632321650897eee585511a84f451a205d1f7704) )
	/* 0x10000 - 0x1ffff empty */

	ROM_REGION( 0x10000, "sub", 0 )             /* Region 2 - sound cpu */
	ROM_LOAD( "b08__13.3e", 0x00000, 0x10000, CRC(e8035ef1) SHA1(9a54e952cff0036c4b6affd9ffb1097cdccbe255) )

	ROM_REGION( 0x10000, "mcu", 0 )    /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b08__09.3g", 0x0000, 0x0800, NO_DUMP ) /* Labeled B08 // 09 and under printed label "?Taito M-009?", is a mask 8042 */

	ROM_REGION( 0x80000, "gfx1", 0 )
	ROM_LOAD( "b08-01.13a", 0x00000, 0x20000, CRC(2ccc86b4) SHA1(eced1d7e687db0331507726946b6a19a690a7604) )
	ROM_LOAD( "b08-02.10a", 0x20000, 0x20000, CRC(056a985f) SHA1(6333b71c631d3307929aae633760870451830e10) )
	ROM_LOAD( "b08-03.7a",  0x40000, 0x20000, CRC(274a795f) SHA1(49353590e1a418843f57c715185e407a20021936) )
	ROM_LOAD( "b08-04.4a",  0x60000, 0x20000, CRC(9754f703) SHA1(0018ebf7da3f501345f3f5085d98d7614f8ce1b6) )

	ROM_REGION( 0x0400, "proms", 0 )
	ROM_LOAD( "b08-08.15f", 0x00000, 0x200, CRC(a4f7ebd9) SHA1(094eb63c18898c6ee8d722492bdfd28091c61773) )  /* hi bytes, AM27S29 or compatible like MB7124 */
	ROM_LOAD( "b08-07.16f", 0x00200, 0x200, CRC(ea34d9f7) SHA1(9a46edc64f961bd96908419cabd92445d300fc19) )  /* lo bytes, AM27S29 or compatible like MB7124 */

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with extermination */
	ROM_LOAD( "b06-10.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

ROM_START( arknoid2u )
	ROM_REGION( 0x20000, "maincpu", 0 )             /* Region 0 - main cpu */
	ROM_LOAD( "b08__11.11c", 0x00000, 0x10000, CRC(99555231) SHA1(2798f3f5b3f1fa27598fe7a6e95c75d9142c8d34) )
	/* 0x10000 - 0x1ffff empty */

	ROM_REGION( 0x10000, "sub", 0 )             /* Region 2 - sound cpu */
	ROM_LOAD( "b08__12.3e", 0x00000, 0x10000, CRC(dc84e27d) SHA1(d549d8c9fbec0521517f0c5f5cee763e27d48633) )

	ROM_REGION( 0x10000, "mcu", 0 )    /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b08__09.3g", 0x0000, 0x0800, NO_DUMP ) /* Labeled B08 // 09 and under printed label "?Taito M-009?", is a mask 8042 */

	ROM_REGION( 0x80000, "gfx1", 0 )
	ROM_LOAD( "b08-01.13a", 0x00000, 0x20000, CRC(2ccc86b4) SHA1(eced1d7e687db0331507726946b6a19a690a7604) )
	ROM_LOAD( "b08-02.10a", 0x20000, 0x20000, CRC(056a985f) SHA1(6333b71c631d3307929aae633760870451830e10) )
	ROM_LOAD( "b08-03.7a",  0x40000, 0x20000, CRC(274a795f) SHA1(49353590e1a418843f57c715185e407a20021936) )
	ROM_LOAD( "b08-04.4a",  0x60000, 0x20000, CRC(9754f703) SHA1(0018ebf7da3f501345f3f5085d98d7614f8ce1b6) )

	ROM_REGION( 0x0400, "proms", 0 )
	ROM_LOAD( "b08-08.15f", 0x00000, 0x200, CRC(a4f7ebd9) SHA1(094eb63c18898c6ee8d722492bdfd28091c61773) )  /* hi bytes, AM27S29 or compatible like MB7124 */
	ROM_LOAD( "b08-07.16f", 0x00200, 0x200, CRC(ea34d9f7) SHA1(9a46edc64f961bd96908419cabd92445d300fc19) )  /* lo bytes, AM27S29 or compatible like MB7124 */

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with extermination */
	ROM_LOAD( "b06-10.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

ROM_START( arknoid2j )
	ROM_REGION( 0x20000, "maincpu", 0 )             /* Region 0 - main cpu */
	ROM_LOAD( "b08_05.11c", 0x00000, 0x10000, CRC(136edf9d) SHA1(f632321650897eee585511a84f451a205d1f7704) )
	/* 0x10000 - 0x1ffff empty */

	ROM_REGION( 0x10000, "sub", 0 )             /* Region 2 - sound cpu */
	ROM_LOAD( "b08_06.3e", 0x00000, 0x10000, CRC(adfcd40c) SHA1(f91299407ed21e2dd244c9b1a315b27ed32f5514) )

	ROM_REGION( 0x10000, "mcu", 0 )    /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b08__09.3g", 0x0000, 0x0800, NO_DUMP ) /* Labeled B08 // 09 and under printed label "?Taito M-009?", is a mask 8042 */

	ROM_REGION( 0x80000, "gfx1", 0 )
	ROM_LOAD( "b08-01.13a", 0x00000, 0x20000, CRC(2ccc86b4) SHA1(eced1d7e687db0331507726946b6a19a690a7604) )
	ROM_LOAD( "b08-02.10a", 0x20000, 0x20000, CRC(056a985f) SHA1(6333b71c631d3307929aae633760870451830e10) )
	ROM_LOAD( "b08-03.7a",  0x40000, 0x20000, CRC(274a795f) SHA1(49353590e1a418843f57c715185e407a20021936) )
	ROM_LOAD( "b08-04.4a",  0x60000, 0x20000, CRC(9754f703) SHA1(0018ebf7da3f501345f3f5085d98d7614f8ce1b6) )

	ROM_REGION( 0x0400, "proms", 0 )
	ROM_LOAD( "b08-08.15f", 0x00000, 0x200, CRC(a4f7ebd9) SHA1(094eb63c18898c6ee8d722492bdfd28091c61773) )  /* hi bytes, AM27S29 or compatible like MB7124 */
	ROM_LOAD( "b08-07.16f", 0x00200, 0x200, CRC(ea34d9f7) SHA1(9a46edc64f961bd96908419cabd92445d300fc19) )  /* lo bytes, AM27S29 or compatible like MB7124 */

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with extermination */
	ROM_LOAD( "b06-10.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

ROM_START( arknoid2b )
	ROM_REGION( 0x20000, "maincpu", 0 )             /* Region 0 - main cpu */
	ROM_LOAD( "boot.11c",  0x00000, 0x10000, CRC(3847dfb0) SHA1(993c8af3df7a4d5a2523f0e31a6df1c07ba13c7d) )
	/* 0x10000 - 0x1ffff empty */

	ROM_REGION( 0x10000, "sub", 0 )             /* Region 2 - sound cpu */
	ROM_LOAD( "b08_13.3e", 0x00000, 0x10000, CRC(e8035ef1) SHA1(9a54e952cff0036c4b6affd9ffb1097cdccbe255) )

	ROM_REGION( 0x10000, "mcu", 0 )    /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "ark28742.3g", 0x0000, 0x0800, NO_DUMP ) /* Labeled B08 // 09 and under printed label "?Taito M-009?", is a mask 8042... does the bootleg set even HAVE the mcu? */

	ROM_REGION( 0x80000, "gfx1", 0 )
	ROM_LOAD( "b08-01.13a", 0x00000, 0x20000, CRC(2ccc86b4) SHA1(eced1d7e687db0331507726946b6a19a690a7604) )
	ROM_LOAD( "b08-02.10a", 0x20000, 0x20000, CRC(056a985f) SHA1(6333b71c631d3307929aae633760870451830e10) )
	ROM_LOAD( "b08-03.7a",  0x40000, 0x20000, CRC(274a795f) SHA1(49353590e1a418843f57c715185e407a20021936) )
	ROM_LOAD( "b08-04.4a",  0x60000, 0x20000, CRC(9754f703) SHA1(0018ebf7da3f501345f3f5085d98d7614f8ce1b6) )

	ROM_REGION( 0x0400, "proms", 0 )
	ROM_LOAD( "b08-08.15f", 0x00000, 0x200, CRC(a4f7ebd9) SHA1(094eb63c18898c6ee8d722492bdfd28091c61773) )  /* hi bytes, AM27S29 or compatible like MB7124 */
	ROM_LOAD( "b08-07.16f", 0x00200, 0x200, CRC(ea34d9f7) SHA1(9a46edc64f961bd96908419cabd92445d300fc19) )  /* lo bytes, AM27S29 or compatible like MB7124 */

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with extermination */
	ROM_LOAD( "b06-10.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

/*
PCB:
Seta: P0-025-A
Taito: K1100241A J1100107A

The drtoppelj PCB has a sticker label which says "K1100269A // DR. ドツペル タンケン" (DoTuPeRu TaNKeN)
*/
ROM_START( drtoppel )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "b19__09.11c", 0x00000, 0x10000, CRC(3e654f82) SHA1(d9e351d82546b08eb7887ea1d976fa97a259db6e) )
	ROM_LOAD( "b19__10.9c",  0x10000, 0x10000, CRC(7e72fd25) SHA1(6035e4db75e6dc57b13bb6e92217d1c2d0ffdfd2) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "b19__15.3e", 0x00000, 0x10000, BAD_DUMP CRC(37a0d3fb) SHA1(f65fb9382af5f5b09725c39b660c5138b3912f53) ) /* Region-Hacked??, need correct Taito rom number */

	ROM_REGION( 0x10000, "mcu", 0 )    /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b06__14.1g", 0x0000, 0x0800, CRC(28907072) SHA1(21c7017af8a8ceb8e43d7e798f48518b136fd45c) ) /* Labeled B06 // 14 and under printed label "Taito M-001, 128P, 720100", is a mask 8042 */

	ROM_REGION( 0x100000, "gfx1", 0 )
	ROM_LOAD( "b19-01.23c1000.13a", 0x00000, 0x20000, CRC(a7e8a0c1) SHA1(a2f017ae5b6472d4202f126d0247b3fe4b1321d1) )
	ROM_LOAD( "b19-02.23c1000.12a", 0x20000, 0x20000, CRC(790ae654) SHA1(5fd6b89918e1539e00c918959b96d2a9394c8abe) )
	ROM_LOAD( "b19-03.23c1000.10a", 0x40000, 0x20000, CRC(495c4c5a) SHA1(a23b512cda4c0c535df5508a52faebe401c1797d) )
	ROM_LOAD( "b19-04.23c1000.8a",  0x60000, 0x20000, CRC(647007a0) SHA1(10ec35a15091967038bb26fb116c47d730f69edc) )
	ROM_LOAD( "b19-05.23c1000.7a",  0x80000, 0x20000, CRC(49f2b1a5) SHA1(5e98bb421afaa02471ad02213ea6ca23ff2f0e27) )
	ROM_LOAD( "b19-06.23c1000.5a",  0xa0000, 0x20000, CRC(2d39f1d0) SHA1(2aa89a5cc7f026c8db9922b183319ff66ac4a071) )
	ROM_LOAD( "b19-07.23c1000.4a",  0xc0000, 0x20000, CRC(8bb06f41) SHA1(a0c182d473317f2cdb31bdf39a2593c032002305) )
	ROM_LOAD( "b19-08.23c1000.2a",  0xe0000, 0x20000, CRC(3584b491) SHA1(d0aca90708be241bbd3a1097220a85083337a4bc) )

	ROM_REGION( 0x0400, "proms", 0 )        /* color proms */
	ROM_LOAD( "b19-13.am27s29.15f", 0x0000, 0x200, CRC(6a547980) SHA1(c82f8dfad028565b4b4e5be1167f2f290c929090) )   /* hi bytes, AM27S29 or compatible like MB7124 */
	ROM_LOAD( "b19-12.am27s29.16f", 0x0200, 0x200, CRC(5754e9d8) SHA1(8c7d29e22c90b1f72929b95675dc15e431aae044) )   /* lo bytes, AM27S29 or compatible like MB7124 */

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with extermination */
	ROM_LOAD( "b06-10.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

ROM_START( drtoppelu )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "b19__09.11c", 0x00000, 0x10000, CRC(3e654f82) SHA1(d9e351d82546b08eb7887ea1d976fa97a259db6e) )
	ROM_LOAD( "b19__10.9c",  0x10000, 0x10000, CRC(7e72fd25) SHA1(6035e4db75e6dc57b13bb6e92217d1c2d0ffdfd2) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "b19__14.3e", 0x00000, 0x10000, BAD_DUMP CRC(05565b22) SHA1(d1aa47b438d3b44c5177337809e38b50f6445c36) ) /* Region-Hacked??, need correct Taito rom number */

	ROM_REGION( 0x10000, "mcu", 0 )    /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b06__14.1g", 0x0000, 0x0800, CRC(28907072) SHA1(21c7017af8a8ceb8e43d7e798f48518b136fd45c) ) /* Labeled B06 // 14 and under printed label "Taito M-001, 128P, 720100", is a mask 8042 */

	ROM_REGION( 0x100000, "gfx1", 0 )
	ROM_LOAD( "b19-01.23c1000.13a", 0x00000, 0x20000, CRC(a7e8a0c1) SHA1(a2f017ae5b6472d4202f126d0247b3fe4b1321d1) )
	ROM_LOAD( "b19-02.23c1000.12a", 0x20000, 0x20000, CRC(790ae654) SHA1(5fd6b89918e1539e00c918959b96d2a9394c8abe) )
	ROM_LOAD( "b19-03.23c1000.10a", 0x40000, 0x20000, CRC(495c4c5a) SHA1(a23b512cda4c0c535df5508a52faebe401c1797d) )
	ROM_LOAD( "b19-04.23c1000.8a",  0x60000, 0x20000, CRC(647007a0) SHA1(10ec35a15091967038bb26fb116c47d730f69edc) )
	ROM_LOAD( "b19-05.23c1000.7a",  0x80000, 0x20000, CRC(49f2b1a5) SHA1(5e98bb421afaa02471ad02213ea6ca23ff2f0e27) )
	ROM_LOAD( "b19-06.23c1000.5a",  0xa0000, 0x20000, CRC(2d39f1d0) SHA1(2aa89a5cc7f026c8db9922b183319ff66ac4a071) )
	ROM_LOAD( "b19-07.23c1000.4a",  0xc0000, 0x20000, CRC(8bb06f41) SHA1(a0c182d473317f2cdb31bdf39a2593c032002305) )
	ROM_LOAD( "b19-08.23c1000.2a",  0xe0000, 0x20000, CRC(3584b491) SHA1(d0aca90708be241bbd3a1097220a85083337a4bc) )

	ROM_REGION( 0x0400, "proms", 0 )        /* color proms */
	ROM_LOAD( "b19-13.am27s29.15f", 0x0000, 0x200, CRC(6a547980) SHA1(c82f8dfad028565b4b4e5be1167f2f290c929090) )   /* hi bytes, AM27S29 or compatible like MB7124 */
	ROM_LOAD( "b19-12.am27s29.16f", 0x0200, 0x200, CRC(5754e9d8) SHA1(8c7d29e22c90b1f72929b95675dc15e431aae044) )   /* lo bytes, AM27S29 or compatible like MB7124 */

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with extermination */
	ROM_LOAD( "b06-10.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

ROM_START( drtoppelj )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "b19__09.11c", 0x00000, 0x10000, CRC(3e654f82) SHA1(d9e351d82546b08eb7887ea1d976fa97a259db6e) )
	ROM_LOAD( "b19__10.9c",  0x10000, 0x10000, CRC(7e72fd25) SHA1(6035e4db75e6dc57b13bb6e92217d1c2d0ffdfd2) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "b19__11.3e", 0x00000, 0x10000, CRC(524dc249) SHA1(158b2de0fcd17ad16ba72bb24888122bf704e216) )

	ROM_REGION( 0x10000, "mcu", 0 )    /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b06__14.1g", 0x0000, 0x0800, CRC(28907072) SHA1(21c7017af8a8ceb8e43d7e798f48518b136fd45c) ) /* Labeled B06 // 14 and under printed label "Taito M-001, 128P, 720100", is a mask 8042 */

	ROM_REGION( 0x100000, "gfx1", 0 )
	ROM_LOAD( "b19-01.23c1000.13a", 0x00000, 0x20000, CRC(a7e8a0c1) SHA1(a2f017ae5b6472d4202f126d0247b3fe4b1321d1) )
	ROM_LOAD( "b19-02.23c1000.12a", 0x20000, 0x20000, CRC(790ae654) SHA1(5fd6b89918e1539e00c918959b96d2a9394c8abe) )
	ROM_LOAD( "b19-03.23c1000.10a", 0x40000, 0x20000, CRC(495c4c5a) SHA1(a23b512cda4c0c535df5508a52faebe401c1797d) )
	ROM_LOAD( "b19-04.23c1000.8a",  0x60000, 0x20000, CRC(647007a0) SHA1(10ec35a15091967038bb26fb116c47d730f69edc) )
	ROM_LOAD( "b19-05.23c1000.7a",  0x80000, 0x20000, CRC(49f2b1a5) SHA1(5e98bb421afaa02471ad02213ea6ca23ff2f0e27) )
	ROM_LOAD( "b19-06.23c1000.5a",  0xa0000, 0x20000, CRC(2d39f1d0) SHA1(2aa89a5cc7f026c8db9922b183319ff66ac4a071) )
	ROM_LOAD( "b19-07.23c1000.4a",  0xc0000, 0x20000, CRC(8bb06f41) SHA1(a0c182d473317f2cdb31bdf39a2593c032002305) )
	ROM_LOAD( "b19-08.23c1000.2a",  0xe0000, 0x20000, CRC(3584b491) SHA1(d0aca90708be241bbd3a1097220a85083337a4bc) )

	ROM_REGION( 0x0400, "proms", 0 )        /* color proms */
	ROM_LOAD( "b19-13.am27s29.15f", 0x0000, 0x200, CRC(6a547980) SHA1(c82f8dfad028565b4b4e5be1167f2f290c929090) )   /* hi bytes, AM27S29 or compatible like MB7124 */
	ROM_LOAD( "b19-12.am27s29.16f", 0x0200, 0x200, CRC(5754e9d8) SHA1(8c7d29e22c90b1f72929b95675dc15e431aae044) )   /* lo bytes, AM27S29 or compatible like MB7124 */

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with extermination */
	ROM_LOAD( "b06-10.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

ROM_START( kageki )
	ROM_REGION( 0x20000, "maincpu", 0 )
	ROM_LOAD( "b35-13.bin", 0x00000, 0x10000, CRC(dc4b025f) SHA1(ed7e0d846693abe0a0ac198e23b272f84b30af46) )    /* World ver */
	ROM_LOAD( "b35-10.9c",  0x10000, 0x10000, CRC(b150457d) SHA1(a58e46e7dfdc93c2cc7c04d623d7754f85ba693b) )

	ROM_REGION( 0x10000, "sub", 0 )
	ROM_LOAD( "b35-14.bin", 0x00000, 0x10000, CRC(8adef2d0) SHA1(0dc8206b35e898b8fed5cdccbdcc5ff1bad68da4) )    /* World ver */

	ROM_REGION( 0x100000, "gfx1", 0 )
	ROM_LOAD( "b35__01.13a",  0x00000, 0x20000, CRC(01d83a69) SHA1(92a84329306b58a45f7bb443a8642eeaeb04d553) )
	ROM_LOAD( "b35__02.12a",  0x20000, 0x20000, CRC(d8af47ac) SHA1(2ef9ca991bf55ed6c12bf3a7dc4aa904d7749d5c) )
	ROM_LOAD( "b35__03.10a",  0x40000, 0x20000, CRC(3cb68797) SHA1(e7669b1a9a26dede560cc87695004d29510bc1f5) )
	ROM_LOAD( "b35__04.8a",   0x60000, 0x20000, CRC(71c03f91) SHA1(edce6e5a52b0c83c1c3c6bf9bc6b7957f7941521) )
	ROM_LOAD( "b35__05.7a",   0x80000, 0x20000, CRC(a4e20c08) SHA1(5d1d23d1410fea8650b18c595b0170a17e5d89a6) )
	ROM_LOAD( "b35__06.5a",   0xa0000, 0x20000, CRC(3f8ab658) SHA1(44de7ee2bdb89bc520ed9bc812c26789c3f31411) )
	ROM_LOAD( "b35__07.4a",   0xc0000, 0x20000, CRC(1b4af049) SHA1(09783816d5076219d241538e2711402eb8c4cd03) )
	ROM_LOAD( "b35__08.2a",   0xe0000, 0x20000, CRC(deb2268c) SHA1(318bf3da6cbe20758397d5f78caf3cda02f322d7) )

	ROM_REGION( 0x10000, "samples", 0 ) /* samples */
	ROM_LOAD( "b35-15.98g",  0x00000, 0x10000, CRC(e6212a0f) SHA1(43891f4fd141b00ed458be47a107a2550a0534c2) )

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with extermination except d9 */
	ROM_LOAD( "b06-101.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

/* M6100309A PCB
   P0-038A */
ROM_START( kagekiu )
	ROM_REGION( 0x20000, "maincpu", 0 )
	ROM_LOAD( "b35-16.11c", 0x00000, 0x10000, CRC(a4e6fd58) SHA1(7cfe5b3fa6c88cdab45719f5b58541270825ad30) )    /* US ver */
	ROM_LOAD( "b35-10.9c",  0x10000, 0x10000, CRC(b150457d) SHA1(a58e46e7dfdc93c2cc7c04d623d7754f85ba693b) )

	ROM_REGION( 0x10000, "sub", 0 )
	ROM_LOAD( "b35-17.43e", 0x00000, 0x10000, CRC(fdd9c246) SHA1(ac7a59ed19d0d81748cabd8b77a6ba3937e3cc99) )    /* US ver */

	ROM_REGION( 0x100000, "gfx1", 0 )
	ROM_LOAD( "b35__01.13a",  0x00000, 0x20000, CRC(01d83a69) SHA1(92a84329306b58a45f7bb443a8642eeaeb04d553) )
	ROM_LOAD( "b35__02.12a",  0x20000, 0x20000, CRC(d8af47ac) SHA1(2ef9ca991bf55ed6c12bf3a7dc4aa904d7749d5c) )
	ROM_LOAD( "b35__03.10a",  0x40000, 0x20000, CRC(3cb68797) SHA1(e7669b1a9a26dede560cc87695004d29510bc1f5) )
	ROM_LOAD( "b35__04.8a",   0x60000, 0x20000, CRC(71c03f91) SHA1(edce6e5a52b0c83c1c3c6bf9bc6b7957f7941521) )
	ROM_LOAD( "b35__05.7a",   0x80000, 0x20000, CRC(a4e20c08) SHA1(5d1d23d1410fea8650b18c595b0170a17e5d89a6) )
	ROM_LOAD( "b35__06.5a",   0xa0000, 0x20000, CRC(3f8ab658) SHA1(44de7ee2bdb89bc520ed9bc812c26789c3f31411) )
	ROM_LOAD( "b35__07.4a",   0xc0000, 0x20000, CRC(1b4af049) SHA1(09783816d5076219d241538e2711402eb8c4cd03) )
	ROM_LOAD( "b35__08.2a",   0xe0000, 0x20000, CRC(deb2268c) SHA1(318bf3da6cbe20758397d5f78caf3cda02f322d7) )

	ROM_REGION( 0x10000, "samples", 0 ) /* samples */
	ROM_LOAD( "b35-15.98g",  0x00000, 0x10000, CRC(e6212a0f) SHA1(43891f4fd141b00ed458be47a107a2550a0534c2) )   /* matches World ver */

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with extermination except d9 */
	ROM_LOAD( "b06-101.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

ROM_START( kagekij )
	ROM_REGION( 0x20000, "maincpu", 0 )
	ROM_LOAD( "b35-09.11c", 0x00000, 0x10000, CRC(829637d5) SHA1(0239ae925968336a90cbe16e23519773b6f2f2ac) )    /* JP ver */
	ROM_LOAD( "b35-10.9c",  0x10000, 0x10000, CRC(b150457d) SHA1(a58e46e7dfdc93c2cc7c04d623d7754f85ba693b) )

	ROM_REGION( 0x10000, "sub", 0 )
	ROM_LOAD( "b35-11.43e", 0x00000, 0x10000, CRC(64d093fc) SHA1(3ca3f69d8946c453c0edb8586b92e2948a2d0b6c) )    /* JP ver */

	ROM_REGION( 0x100000, "gfx1", 0 )
	ROM_LOAD( "b35-01.13a",  0x00000, 0x20000, CRC(01d83a69) SHA1(92a84329306b58a45f7bb443a8642eeaeb04d553) )
	ROM_LOAD( "b35-02.12a",  0x20000, 0x20000, CRC(d8af47ac) SHA1(2ef9ca991bf55ed6c12bf3a7dc4aa904d7749d5c) )
	ROM_LOAD( "b35-03.10a",  0x40000, 0x20000, CRC(3cb68797) SHA1(e7669b1a9a26dede560cc87695004d29510bc1f5) )
	ROM_LOAD( "b35-04.8a",   0x60000, 0x20000, CRC(71c03f91) SHA1(edce6e5a52b0c83c1c3c6bf9bc6b7957f7941521) )
	ROM_LOAD( "b35-05.7a",   0x80000, 0x20000, CRC(a4e20c08) SHA1(5d1d23d1410fea8650b18c595b0170a17e5d89a6) )
	ROM_LOAD( "b35-06.5a",   0xa0000, 0x20000, CRC(3f8ab658) SHA1(44de7ee2bdb89bc520ed9bc812c26789c3f31411) )
	ROM_LOAD( "b35-07.4a",   0xc0000, 0x20000, CRC(1b4af049) SHA1(09783816d5076219d241538e2711402eb8c4cd03) )
	ROM_LOAD( "b35-08.2a",   0xe0000, 0x20000, CRC(deb2268c) SHA1(318bf3da6cbe20758397d5f78caf3cda02f322d7) )

	ROM_REGION( 0x10000, "samples", 0 ) /* samples */
	ROM_LOAD( "b35-12.98g", 0x00000, 0x10000, CRC(184409f1) SHA1(711bdd499670e86630ebb6820262b1d8d651c987) )    /* JP ver */

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with extermination except d9 */
	ROM_LOAD( "b06-101.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

/* Board ID is M6100309A - program rom has been hacked to say 1992 :/
    supported because it appears to be a different code revision to the other supported sets
*/

ROM_START( kagekih )
	ROM_REGION( 0x20000, "maincpu", 0 )
	ROM_LOAD( "b35_16.11c", 0x00000, 0x10000, CRC(1cf67603) SHA1(0627285ac69e44312d7694c64b96a81489d8663c) )    /* hacked ver of the World set */
	ROM_LOAD( "b35-10.9c",  0x10000, 0x10000, CRC(b150457d) SHA1(a58e46e7dfdc93c2cc7c04d623d7754f85ba693b) )

	ROM_REGION( 0x10000, "sub", 0 )
	ROM_LOAD( "b35-11.43e", 0x00000, 0x10000, CRC(64d093fc) SHA1(3ca3f69d8946c453c0edb8586b92e2948a2d0b6c) )    /* JP ver */

	ROM_REGION( 0x100000, "gfx1", 0 )
	ROM_LOAD( "b35-01.13a",  0x00000, 0x20000, CRC(01d83a69) SHA1(92a84329306b58a45f7bb443a8642eeaeb04d553) )
	ROM_LOAD( "b35-02.12a",  0x20000, 0x20000, CRC(d8af47ac) SHA1(2ef9ca991bf55ed6c12bf3a7dc4aa904d7749d5c) )
	ROM_LOAD( "b35-03.10a",  0x40000, 0x20000, CRC(3cb68797) SHA1(e7669b1a9a26dede560cc87695004d29510bc1f5) )
	ROM_LOAD( "b35-04.8a",   0x60000, 0x20000, CRC(71c03f91) SHA1(edce6e5a52b0c83c1c3c6bf9bc6b7957f7941521) )
	ROM_LOAD( "b35-05.7a",   0x80000, 0x20000, CRC(a4e20c08) SHA1(5d1d23d1410fea8650b18c595b0170a17e5d89a6) )
	ROM_LOAD( "b35-06.5a",   0xa0000, 0x20000, CRC(3f8ab658) SHA1(44de7ee2bdb89bc520ed9bc812c26789c3f31411) )
	ROM_LOAD( "b35-07.4a",   0xc0000, 0x20000, CRC(1b4af049) SHA1(09783816d5076219d241538e2711402eb8c4cd03) )
	ROM_LOAD( "b35-08.2a",   0xe0000, 0x20000, CRC(deb2268c) SHA1(318bf3da6cbe20758397d5f78caf3cda02f322d7) )

	ROM_REGION( 0x10000, "samples", 0 ) /* samples */
	ROM_LOAD( "b35-12.98g", 0x00000, 0x10000, CRC(184409f1) SHA1(711bdd499670e86630ebb6820262b1d8d651c987) )    /* JP ver */

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with extermination except d9 */
	ROM_LOAD( "b06-101.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END


/*
Chuka Taisen
Taito, 1988

This PCB comes in two variations: one which is on the older, color prom based P0-025-A PCB as used on extrmatn, drtoppel
and one which is a unique PCB sort of an 'older version of p0-051-a' which uses color ram like tnzs.
The color prom version is not dumped yet.

The chukatai (p0-025-a) PCB has a sticker label which says "????????? // CHUKATAISEN"
The chukatai (p0-028-a) PCB has a sticker label which says "K1100??2A // CHUKATAISEN"

PCB:
Older(technically 'newer rom ids but used to get rid of old pcb stock') set is:
Seta: P0-025-A
Taito: K1100241A J1100107A

The two newer sets are:
Seta: P0-028-A
Taito: K1100416A J1100332A
which is described in the diagram below.

|--------------------------------------------------|
|    SETA                     SETA         12MHz   |
|    X1-003     6116          X1-001               |
|               6116          YM3906               |
|    SETA       6116                               |
|    X1-006     6116                               |
|                                                  |
|                             SETA                 |
|                             X1-002A              |
|                                                  |
|       DSWB                                       |
|J                                   B44-01.8      |
|A                      PRG10.32                   |
|M      DSWA            PRG11.31     B44-05.7      |
|M                      6264                       |
|A                                   B44-02.6      |
|     YM3014                                       |
|                                    B44-06.5      |
|             B06-101.36(PAL)                      |
|                                    B44-03.4      |
|       YM2203   Z80B                              |
|                    B06-11.35(PAL)  B44-07.3      |
|                B44_12.38                         |
|   SETA                             B44-04.2      |
|   X1-004         Z80B                            |
|                    B06-12.26(PAL)  B44-08.1      |
|                    B06-11.25(PAL)                |
|--------------------------------------------------|
Notes:
      6264: 8K x8 SRAM
      6116: 2K x8 SRAM

The undumped Chuka Taisen set has a daughterboard which plugs into the
GFX ROM sockets and allows use of 27c1000d eproms instead of 23c1000 mask roms.
The undumped set, unlike the two here, runs on an older drtoppel/extrmatn pcb
and uses color proms!


*/

ROM_START( chukatai )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "b44-10", 0x00000, 0x10000, CRC(8c69e008) SHA1(7825965f517f3562a508345b7c0d32b8a57bd38a) )
	ROM_LOAD( "b44-11", 0x10000, 0x10000, CRC(32484094) SHA1(f320fea2910816b5085ca9aa37e30af665fb6be1) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "b44-12w", 0x00000, 0x10000, CRC(e80ecdca) SHA1(cd96403ca97f18f630118dcb3dc2179c01147213) ) /* Hacked??, need correct Taito rom number */

	ROM_REGION( 0x10000, "mcu", 0 ) /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b44-8742.mcu", 0x0000, 0x0800, CRC(7dff3f9f) SHA1(bbf4e036d025fe8179b053d639f9b8ad401e6e68) ) /* B44 // 09 is the label? what is the mask number under the label? maybe Taito M-011? last digit is definitely 1 */

	ROM_REGION( 0x100000, "gfx1", 0 )
	ROM_LOAD( "b44-01.a13", 0x00000, 0x20000, CRC(aae7b3d5) SHA1(52809ea22d98811ece2fb27e80db6ddf4fbacb07) )
	ROM_LOAD( "b44-02.a12", 0x20000, 0x20000, CRC(7f0b9568) SHA1(415d2638d1b0eb36b2e2f63219cbc0dbebe02dc6) )
	ROM_LOAD( "b44-03.a10", 0x40000, 0x20000, CRC(5a54a3b9) SHA1(6b219f1c3570f16eb4a06221d7e527c735437bac) )
	ROM_LOAD( "b44-04.a08", 0x60000, 0x20000, CRC(3c5f544b) SHA1(d3b0ee18f1027483a36ef02757b62f42a086a8e2) )
	ROM_LOAD( "b44-05.a07", 0x80000, 0x20000, CRC(d1b7e314) SHA1(8b4181caa32955b4274614a4238bb24d67ecb729) )
	ROM_LOAD( "b44-06.a05", 0xa0000, 0x20000, CRC(269978a8) SHA1(aef7b8d3d00dcc4201e0a1e28026f6f1bdafd0b7) )
	ROM_LOAD( "b44-07.a04", 0xc0000, 0x20000, CRC(3e0e737e) SHA1(f8d62c7b69c79da9df7ef5ce454060d3645e5884) )
	ROM_LOAD( "b44-08.a02", 0xe0000, 0x20000, CRC(6cb1e8fc) SHA1(4ab0c2cce1de2616044a9bfb9bf17f95a49baffd) )

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with extermination except d9 */
	ROM_LOAD( "b06-101.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

ROM_START( chukataiu )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "b44-10", 0x00000, 0x10000, CRC(8c69e008) SHA1(7825965f517f3562a508345b7c0d32b8a57bd38a) )
	ROM_LOAD( "b44-11", 0x10000, 0x10000, CRC(32484094) SHA1(f320fea2910816b5085ca9aa37e30af665fb6be1) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "b44-12u", 0x00000, 0x10000, BAD_DUMP CRC(9f09fd5c) SHA1(ae92f2e893e1e666dcabbd793f1a778c5e3d7bab) ) /* Hacked??, need correct Taito rom number */

	ROM_REGION( 0x1000, "mcu", 0 )  /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b44-8742.mcu", 0x0000, 0x0800, CRC(7dff3f9f) SHA1(bbf4e036d025fe8179b053d639f9b8ad401e6e68) ) /* B44 // 09 is the label? what is the mask number under the label? maybe Taito M-011? last digit is definitely 1 */

	ROM_REGION( 0x100000, "gfx1", 0 )
	ROM_LOAD( "b44-01.a13", 0x00000, 0x20000, CRC(aae7b3d5) SHA1(52809ea22d98811ece2fb27e80db6ddf4fbacb07) )
	ROM_LOAD( "b44-02.a12", 0x20000, 0x20000, CRC(7f0b9568) SHA1(415d2638d1b0eb36b2e2f63219cbc0dbebe02dc6) )
	ROM_LOAD( "b44-03.a10", 0x40000, 0x20000, CRC(5a54a3b9) SHA1(6b219f1c3570f16eb4a06221d7e527c735437bac) )
	ROM_LOAD( "b44-04.a08", 0x60000, 0x20000, CRC(3c5f544b) SHA1(d3b0ee18f1027483a36ef02757b62f42a086a8e2) )
	ROM_LOAD( "b44-05.a07", 0x80000, 0x20000, CRC(d1b7e314) SHA1(8b4181caa32955b4274614a4238bb24d67ecb729) )
	ROM_LOAD( "b44-06.a05", 0xa0000, 0x20000, CRC(269978a8) SHA1(aef7b8d3d00dcc4201e0a1e28026f6f1bdafd0b7) )
	ROM_LOAD( "b44-07.a04", 0xc0000, 0x20000, CRC(3e0e737e) SHA1(f8d62c7b69c79da9df7ef5ce454060d3645e5884) )
	ROM_LOAD( "b44-08.a02", 0xe0000, 0x20000, CRC(6cb1e8fc) SHA1(4ab0c2cce1de2616044a9bfb9bf17f95a49baffd) )

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with extermination except d9 */
	ROM_LOAD( "b06-101.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

ROM_START( chukataij )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "b44-10", 0x00000, 0x10000, CRC(8c69e008) SHA1(7825965f517f3562a508345b7c0d32b8a57bd38a) )
	ROM_LOAD( "b44-11", 0x10000, 0x10000, CRC(32484094) SHA1(f320fea2910816b5085ca9aa37e30af665fb6be1) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "b44-12", 0x00000, 0x10000, CRC(0600ace6) SHA1(3d5767b91ea63128bfbff3527ddcf90fcf43af2e) )

	ROM_REGION( 0x10000, "mcu", 0 ) /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b44-8742.mcu", 0x0000, 0x0800, CRC(7dff3f9f) SHA1(bbf4e036d025fe8179b053d639f9b8ad401e6e68) ) /* B44 // 09 is the label? what is the mask number under the label? maybe Taito M-011? last digit is definitely 1 */

	ROM_REGION( 0x100000, "gfx1", 0 )
	ROM_LOAD( "b44-01.a13", 0x00000, 0x20000, CRC(aae7b3d5) SHA1(52809ea22d98811ece2fb27e80db6ddf4fbacb07) )
	ROM_LOAD( "b44-02.a12", 0x20000, 0x20000, CRC(7f0b9568) SHA1(415d2638d1b0eb36b2e2f63219cbc0dbebe02dc6) )
	ROM_LOAD( "b44-03.a10", 0x40000, 0x20000, CRC(5a54a3b9) SHA1(6b219f1c3570f16eb4a06221d7e527c735437bac) )
	ROM_LOAD( "b44-04.a08", 0x60000, 0x20000, CRC(3c5f544b) SHA1(d3b0ee18f1027483a36ef02757b62f42a086a8e2) )
	ROM_LOAD( "b44-05.a07", 0x80000, 0x20000, CRC(d1b7e314) SHA1(8b4181caa32955b4274614a4238bb24d67ecb729) )
	ROM_LOAD( "b44-06.a05", 0xa0000, 0x20000, CRC(269978a8) SHA1(aef7b8d3d00dcc4201e0a1e28026f6f1bdafd0b7) )
	ROM_LOAD( "b44-07.a04", 0xc0000, 0x20000, CRC(3e0e737e) SHA1(f8d62c7b69c79da9df7ef5ce454060d3645e5884) )
	ROM_LOAD( "b44-08.a02", 0xe0000, 0x20000, CRC(6cb1e8fc) SHA1(4ab0c2cce1de2616044a9bfb9bf17f95a49baffd) )

	ROM_REGION( 0x10000, "pal", 0 ) /* these are shared with extermination except d9 */
	ROM_LOAD( "b06-101.pal16l8a.d9.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-11.pal16l8a.d6.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-12.pal16l8a.c3.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.c2.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

/*An undumped set of chuka taisen exists with all roms in the b44-14
thru b44-25 number range and 27c1000d gfx roms, AND COLOR PROMS, using the older p0-025-a pcb
This set, unlike the others, does use the b06-10.pal16l8a.d9.jed "older" pal rather than the 'newer' b06-101.pal16l8a.d9.jed pal.
*/


/*
The New Zealand Story
Taito, 1988

PCB Layout ("New style PCB" with 3x z80 and no M-chip, and a daughterboard w/roms and z80)
----------
The tnzs PCB has a sticker label which says "M6100409A // N.Z.LAND STORY"

M6100356A (on PCB)
P0-043A (Seta number; on PCB)
|---------------------------------------------------|
|     VOL  HA17408           B53-26.U34  DSWB DSWA  |
|      4558       YM2203 Z80  62256              Z80|
|      4558 YM3014                                  |
|                                                   |
|                                  B06-13           |
|                                   (PAL)           |
|                                                   |
|                                                   |
|                               6264       B53-25.U3|
|J     TESTSW                                       |
|A                                                  |
|M                                                  |
|M                                 B06-101          |
|A                                  (PAL)           |
|---------------------------------------------------| <- ROM Board above
|                                              DIP40|    main PCB
|               X1-001A                             |
|                                                   |
|    X1-004                                         |
|               X1-002A       12MHz                 |
|                                                   |
|                                                   |
|   X1-006                              6264        |
|X1-007     DIP40  DIP40  DIP40  DIP40     B53-24.U1|
|---------------------------------------------------|
Notes:
      All Z80 CPU's running at 6.000MHz (12/2)
      YM2203 running at 3.000MHz (12/4)
      VSync 60Hz
      DIP40 - Empty sockets used for connection of ROM board
      Seta Custom IC's -
                        X1-001A
                        X1-002A
                        X1-004
                        X1-006
                        X1-007


ROM Board
---------
K9100209A
J9100159A SUB PCB
K9100209A N. Z. LAND STORY (sticker)
|---------------------------------------------------|
|                                  PAL(B53-15)      |
|     B53_20     B53_18      B53_16     Z80B   DIP40|
|B53_21    B53_19     B53_17                        |
|                                                   |
|                                                   |
|                                                   |
|B53_23                                             |
|    B53_22                                         |
|           DIP40  DIP40  DIP40  DIP40     62256    |
|---------------------------------------------------|
Notes:
      Z80 clock 6.000MHz
      DIP40 - connection pins joining to Main PCB DIP40 sockets
      All ROMs are 27010 (DIP32)
*/

/* tnzs - new style PCB sets */
ROM_START( tnzs )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "b53-24.u1",   0x00000, 0x20000, CRC(d66824c6) SHA1(fd381ac0dc52ce670c3fde320ea60a209e288a52) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "b53-25.u3",   0x00000, 0x10000, CRC(d6ac4e71) SHA1(f3e71624a8a5e4e4c8a6aa01711ed26bdd5abf5a) )

	ROM_REGION( 0x10000, "audiocpu", 0 )    /* 64k for the third CPU */
	ROM_LOAD( "b53-26.u34",  0x00000, 0x10000, CRC(cfd5649c) SHA1(4f6afccd535d39b41661dc3ccd17af125bfac015) )

	ROM_REGION( 0x100000, "gfx1", 0 ) /* the newer PCBs have updated GFX rom labels, content is the same. Located on a SUB PCB */
	ROM_LOAD( "b53-16.ic7",   0x00000, 0x20000, CRC(c3519c2a) SHA1(30fe7946fbc95ab6b3ccb6944fb24bf47bf3d743) ) /* Also labeled as U35L */
	ROM_LOAD( "b53-17.ic8",   0x20000, 0x20000, CRC(2bf199e8) SHA1(4ed73e4f00ae2f5f4028a0ea5ae3cd238863a370) ) /* Also labeled as U35U */
	ROM_LOAD( "b53-18.ic9",   0x40000, 0x20000, CRC(92f35ed9) SHA1(5fdd8d6ddbb7be9887af3c8dea9ad3b58c4e86f9) ) /* Also labeled as U39L */
	ROM_LOAD( "b53-19.ic10",  0x60000, 0x20000, CRC(edbb9581) SHA1(539396a01ca0b69455f000d446759b232530b542) ) /* Also labeled as U39U */
	ROM_LOAD( "b53-22.ic11",  0x80000, 0x20000, CRC(59d2aef6) SHA1(b657b7603c3eb5f169000d38497ebb93f26f7832) ) /* Also labeled as U43L */
	ROM_LOAD( "b53-23.ic13",  0xa0000, 0x20000, CRC(74acfb9b) SHA1(90b544ed7ede7565660bdd13c94c15c54423cda9) ) /* Also labeled as U43U */
	ROM_LOAD( "b53-20.ic12",  0xc0000, 0x20000, CRC(095d0dc0) SHA1(ced2937d0594fa00ae344a4e3a3cba23772dc160) ) /* Also labeled as U46L */
	ROM_LOAD( "b53-21.ic14",  0xe0000, 0x20000, CRC(9800c54d) SHA1(761647177d621ac2cdd8b009876eed35809f3c92) ) /* Also labeled as U46U */

	ROM_REGION( 0x10000, "pal", 0 )
	/* these are shared with extermination except for the subpcb pal */
	ROM_LOAD( "b06-13.pal16l8a.f2.jed",  0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-101.pal16l8a.i2.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b53-15.pal16l8a.subpcb.ic6.jed", 0x03000, 0x01000, NO_DUMP) // on sub pcb
ROM_END

ROM_START( tnzsj )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "b53-24.u1",   0x00000, 0x20000, CRC(d66824c6) SHA1(fd381ac0dc52ce670c3fde320ea60a209e288a52) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "b53-27.u3",   0x00000, 0x10000, CRC(b3415fc3) SHA1(a12b1788509e2ac2b05a083f432eecdce00769f6) )

	ROM_REGION( 0x10000, "audiocpu", 0 )    /* 64k for the third CPU */
	ROM_LOAD( "b53-26.u34",  0x00000, 0x10000, CRC(cfd5649c) SHA1(4f6afccd535d39b41661dc3ccd17af125bfac015) )

	ROM_REGION( 0x100000, "gfx1", 0 ) /* the newer PCBs have updated GFX rom labels, content is the same. Located on a SUB PCB */
	ROM_LOAD( "b53-16.ic7",   0x00000, 0x20000, CRC(c3519c2a) SHA1(30fe7946fbc95ab6b3ccb6944fb24bf47bf3d743) ) /* Also labeled as U35L */
	ROM_LOAD( "b53-17.ic8",   0x20000, 0x20000, CRC(2bf199e8) SHA1(4ed73e4f00ae2f5f4028a0ea5ae3cd238863a370) ) /* Also labeled as U35U */
	ROM_LOAD( "b53-18.ic9",   0x40000, 0x20000, CRC(92f35ed9) SHA1(5fdd8d6ddbb7be9887af3c8dea9ad3b58c4e86f9) ) /* Also labeled as U39L */
	ROM_LOAD( "b53-19.ic10",  0x60000, 0x20000, CRC(edbb9581) SHA1(539396a01ca0b69455f000d446759b232530b542) ) /* Also labeled as U39U */
	ROM_LOAD( "b53-22.ic11",  0x80000, 0x20000, CRC(59d2aef6) SHA1(b657b7603c3eb5f169000d38497ebb93f26f7832) ) /* Also labeled as U43L */
	ROM_LOAD( "b53-23.ic13",  0xa0000, 0x20000, CRC(74acfb9b) SHA1(90b544ed7ede7565660bdd13c94c15c54423cda9) ) /* Also labeled as U43U */
	ROM_LOAD( "b53-20.ic12",  0xc0000, 0x20000, CRC(095d0dc0) SHA1(ced2937d0594fa00ae344a4e3a3cba23772dc160) ) /* Also labeled as U46L */
	ROM_LOAD( "b53-21.ic14",  0xe0000, 0x20000, CRC(9800c54d) SHA1(761647177d621ac2cdd8b009876eed35809f3c92) ) /* Also labeled as U46U */

	ROM_REGION( 0x10000, "pal", 0 )
	/* these are shared with extermination except for the subpcb pal */
	ROM_LOAD( "b06-13.pal16l8a.f2.jed",  0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-101.pal16l8a.i2.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b53-15.pal16l8a.subpcb.ic6.jed", 0x03000, 0x01000, NO_DUMP) // on sub pcb
ROM_END

/* tnzs - old style PCB sets
The New Zealand Story
Taito, 1988

Taito ID: K1100356A
          J1100156A
          MAIN PCB
Seta ID: P0-041A
*/
/* This pcb is derived from the Chuka Taisen,DrToppel and Arkanoid 2 pcbs, replacing the two color proms with color ram;
   There is an M-chip i8x42 (with Taito silkscreen) and no 3rd z80.
   There is no daughter-pcb like the later TNZS pcb has.
   GFX Roms on the pcb are 28 pin 23C1000/TC531000 128K mask roms */

ROM_START( tnzso )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "b53-10.27c1001d.u32", 0x00000, 0x20000, CRC(a73745c6) SHA1(73eb38e75e08312d752332f988dc655084b4a86d) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "b53-14.u38", 0x00000, 0x10000, CRC(f269c5f1) SHA1(15e00e5bc6394f55fc6c591754e24842708f49f4) ) // World version old style PCB

	ROM_REGION( 0x10000, "mcu", 0 ) /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b53-09.u46", 0x0000, 0x0800, CRC(a4bfce19) SHA1(9340862d5bdc1ad4799dc92cae9bce1428b47478) )

	ROM_REGION( 0x100000, "gfx1", 0 )
	ROM_LOAD( "b53-08.u8",   0x00000, 0x20000, CRC(c3519c2a) SHA1(30fe7946fbc95ab6b3ccb6944fb24bf47bf3d743) )
	ROM_LOAD( "b53-07.u7",   0x20000, 0x20000, CRC(2bf199e8) SHA1(4ed73e4f00ae2f5f4028a0ea5ae3cd238863a370) )
	ROM_LOAD( "b53-06.u6",   0x40000, 0x20000, CRC(92f35ed9) SHA1(5fdd8d6ddbb7be9887af3c8dea9ad3b58c4e86f9) )
	ROM_LOAD( "b53-05.u5",   0x60000, 0x20000, CRC(edbb9581) SHA1(539396a01ca0b69455f000d446759b232530b542) )
	ROM_LOAD( "b53-04.u4",   0x80000, 0x20000, CRC(59d2aef6) SHA1(b657b7603c3eb5f169000d38497ebb93f26f7832) )
	ROM_LOAD( "b53-03.u3",   0xa0000, 0x20000, CRC(74acfb9b) SHA1(90b544ed7ede7565660bdd13c94c15c54423cda9) )
	ROM_LOAD( "b53-02.u2",   0xc0000, 0x20000, CRC(095d0dc0) SHA1(ced2937d0594fa00ae344a4e3a3cba23772dc160) )
	ROM_LOAD( "b53-01.u1",   0xe0000, 0x20000, CRC(9800c54d) SHA1(761647177d621ac2cdd8b009876eed35809f3c92) )

	ROM_REGION( 0x10000, "pal", 0 ) /* these are probably shared with extermination except for u35 */
	ROM_LOAD( "b06-12.pal16l8a.u26.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.u25.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b53-12.pal16l8a.u35.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-101.pal16l8a.u36.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

ROM_START( tnzsjo )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "b53-10.27c1001d.u32", 0x00000, 0x20000, CRC(a73745c6) SHA1(73eb38e75e08312d752332f988dc655084b4a86d) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "b53-11.27c512.u38", 0x00000, 0x10000, CRC(9784d443) SHA1(bc3647aac9974031dbe4898417fbaa99841f9548) ) // Japan version old style PCB

	ROM_REGION( 0x10000, "mcu", 0 ) /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b53-09.u46", 0x0000, 0x0800, CRC(a4bfce19) SHA1(9340862d5bdc1ad4799dc92cae9bce1428b47478) )

	ROM_REGION( 0x100000, "gfx1", 0 )
	/* ROMs taken from another set (the ones from this set were read incorrectly) */
	ROM_LOAD( "b53-08.u8",   0x00000, 0x20000, CRC(c3519c2a) SHA1(30fe7946fbc95ab6b3ccb6944fb24bf47bf3d743) )
	ROM_LOAD( "b53-07.u7",   0x20000, 0x20000, CRC(2bf199e8) SHA1(4ed73e4f00ae2f5f4028a0ea5ae3cd238863a370) )
	ROM_LOAD( "b53-06.u6",   0x40000, 0x20000, CRC(92f35ed9) SHA1(5fdd8d6ddbb7be9887af3c8dea9ad3b58c4e86f9) )
	ROM_LOAD( "b53-05.u5",   0x60000, 0x20000, CRC(edbb9581) SHA1(539396a01ca0b69455f000d446759b232530b542) )
	ROM_LOAD( "b53-04.u4",   0x80000, 0x20000, CRC(59d2aef6) SHA1(b657b7603c3eb5f169000d38497ebb93f26f7832) )
	ROM_LOAD( "b53-03.u3",   0xa0000, 0x20000, CRC(74acfb9b) SHA1(90b544ed7ede7565660bdd13c94c15c54423cda9) )
	ROM_LOAD( "b53-02.u2",   0xc0000, 0x20000, CRC(095d0dc0) SHA1(ced2937d0594fa00ae344a4e3a3cba23772dc160) )
	ROM_LOAD( "b53-01.u1",   0xe0000, 0x20000, CRC(9800c54d) SHA1(761647177d621ac2cdd8b009876eed35809f3c92) )

	ROM_REGION( 0x10000, "pal", 0 ) /* these are probably shared with extermination except for u35 */
	ROM_LOAD( "b06-12.pal16l8a.u26.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.u25.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b53-12.pal16l8a.u35.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-101.pal16l8a.u36.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

ROM_START( tnzsuo )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "b53-10.27c1001d.u32", 0x00000, 0x20000, CRC(a73745c6) SHA1(73eb38e75e08312d752332f988dc655084b4a86d) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "b53-13.27c512.u38", 0x00000, 0x10000, CRC(c09f4d28) SHA1(f1fd3202869738e17abcbb757f9ce7260707dd3d) ) // US version old style PCB

	ROM_REGION( 0x10000, "mcu", 0 ) /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b53-09.u46", 0x0000, 0x0800, CRC(a4bfce19) SHA1(9340862d5bdc1ad4799dc92cae9bce1428b47478) )

	ROM_REGION( 0x100000, "gfx1", 0 )
	ROM_LOAD( "b53-08.u8",   0x00000, 0x20000, CRC(c3519c2a) SHA1(30fe7946fbc95ab6b3ccb6944fb24bf47bf3d743) )
	ROM_LOAD( "b53-07.u7",   0x20000, 0x20000, CRC(2bf199e8) SHA1(4ed73e4f00ae2f5f4028a0ea5ae3cd238863a370) )
	ROM_LOAD( "b53-06.u6",   0x40000, 0x20000, CRC(92f35ed9) SHA1(5fdd8d6ddbb7be9887af3c8dea9ad3b58c4e86f9) )
	ROM_LOAD( "b53-05.u5",   0x60000, 0x20000, CRC(edbb9581) SHA1(539396a01ca0b69455f000d446759b232530b542) )
	ROM_LOAD( "b53-04.u4",   0x80000, 0x20000, CRC(59d2aef6) SHA1(b657b7603c3eb5f169000d38497ebb93f26f7832) )
	ROM_LOAD( "b53-03.u3",   0xa0000, 0x20000, CRC(74acfb9b) SHA1(90b544ed7ede7565660bdd13c94c15c54423cda9) )
	ROM_LOAD( "b53-02.u2",   0xc0000, 0x20000, CRC(095d0dc0) SHA1(ced2937d0594fa00ae344a4e3a3cba23772dc160) )
	ROM_LOAD( "b53-01.u1",   0xe0000, 0x20000, CRC(9800c54d) SHA1(761647177d621ac2cdd8b009876eed35809f3c92) )

	ROM_REGION( 0x10000, "pal", 0 ) /* these are probably shared with extermination except for u35 */
	ROM_LOAD( "b06-12.pal16l8a.u26.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.u25.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b53-12.pal16l8a.u35.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-101.pal16l8a.u36.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

ROM_START( tnzsoa ) // is this a legit set, or a hack, or a near-final (later than tnzsop below) prototype?
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "b53-unknown.27c1001d.u32", 0x00000, 0x20000, CRC(edf3b39e) SHA1(be221c99e50795d569611dba454c3954a259a859) ) // ROM LABEL FOR THIS SET IS UNKNOWN

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "b53-unknown.27c512.u38", 0x00000, 0x10000, CRC(60340d63) SHA1(12a26d19dc8e407e502f25617a5a4c9cea131ce2) ) // ROM LABEL FOR THIS SET IS UNKNOWN

	ROM_REGION( 0x10000, "mcu", 0 ) /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b53-09.u46", 0x0000, 0x0800, CRC(a4bfce19) SHA1(9340862d5bdc1ad4799dc92cae9bce1428b47478) )

	ROM_REGION( 0x100000, "gfx1", 0 )
	/* ROMs taken from another set (the ones from this set were read incorrectly) */
	ROM_LOAD( "b53-08.u8",   0x00000, 0x20000, CRC(c3519c2a) SHA1(30fe7946fbc95ab6b3ccb6944fb24bf47bf3d743) )
	ROM_LOAD( "b53-07.u7",   0x20000, 0x20000, CRC(2bf199e8) SHA1(4ed73e4f00ae2f5f4028a0ea5ae3cd238863a370) )
	ROM_LOAD( "b53-06.u6",   0x40000, 0x20000, CRC(92f35ed9) SHA1(5fdd8d6ddbb7be9887af3c8dea9ad3b58c4e86f9) )
	ROM_LOAD( "b53-05.u5",   0x60000, 0x20000, CRC(edbb9581) SHA1(539396a01ca0b69455f000d446759b232530b542) )
	ROM_LOAD( "b53-04.u4",   0x80000, 0x20000, CRC(59d2aef6) SHA1(b657b7603c3eb5f169000d38497ebb93f26f7832) )
	ROM_LOAD( "b53-03.u3",   0xa0000, 0x20000, CRC(74acfb9b) SHA1(90b544ed7ede7565660bdd13c94c15c54423cda9) )
	ROM_LOAD( "b53-02.u2",   0xc0000, 0x20000, CRC(095d0dc0) SHA1(ced2937d0594fa00ae344a4e3a3cba23772dc160) )
	ROM_LOAD( "b53-01.u1",   0xe0000, 0x20000, CRC(9800c54d) SHA1(761647177d621ac2cdd8b009876eed35809f3c92) )

	ROM_REGION( 0x10000, "pal", 0 ) /* PALS not directly observed on this board but assumed to exist */
	/* these are probably shared with extermination except for u35 */
	ROM_LOAD( "b06-12.pal16l8a.u26.jed", 0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.u25.jed", 0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "b53-12.pal16l8a.u35.jed", 0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-101.pal16l8a.u36.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

/* This is a prototype CA403001A PCB (Seta: P0-041-1), and is ALMOST but not exactly the same as the K1100356A/J1100156A (Seta: P0-041A) 'tnzsuo/tnzsjo/arkanoid2/etc' pcb above:
This pcb uses 32-pin 27c1000d eproms for the 8 gfx roms, and the final K1100356A/J1100156A pcb uses 28 pin 23c1000 mask roms instead. Some capacitors near the jamma connector were moved as well.
No other obviously evident routing/wiring changes are present.
This type of pcb might have been used for in-house testing of all the games on this hardware.
*/
ROM_START( tnzsop ) // prototype (location test?) version; has different rom labels, and the Seta X1-001 chip has prototype markings revealing it was fabbed by Yamaha, as 'YM3906'
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "c-11__6-24__1959h.d27c1000d-15.u32", 0x00000, 0x20000, CRC(3c1dae7b) SHA1(0004fccc171714c80565326f8690f9662c5b75d9) ) // Labeled as PCB location, date of 6/24 & checksum - NEC D271000d  eprom

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "e-3__6-24__c4ach.tmm27512d-20.u38", 0x00000, 0x10000, CRC(c7662e96) SHA1(be28298bfde4e3867cfe75633ffb0f8611dbbd8b) ) // Labeled as PCB location, date of 6/24 & checksum - TMM27512D  eprom

	ROM_REGION( 0x10000, "mcu", 0 ) /* M-Chip (i8x42 internal ROM) */
	ROM_LOAD( "b8042h__'88-6-22__0fcc.d8742.u46", 0x0000, 0x0800, CRC(a4bfce19) SHA1(9340862d5bdc1ad4799dc92cae9bce1428b47478) ) // Dated  '88/6/22 with checksum - Intel D8742 MCU

	ROM_REGION( 0x100000, "gfx1", 0 )
	ROM_LOAD( "a13__03e8.d27c1000d-15.a13",  0x00000, 0x20000, CRC(7e0bd5bb) SHA1(95dfb00ec915778e02d8bfa996735ab817191adc) ) // labels contain the PCB location & checksum
	ROM_LOAD( "a12__f4ec.d27c1000d-15.a12",  0x20000, 0x20000, CRC(95880726) SHA1(f4fdedd23e80a6ccf32f737ab4bc57f9fc0925be) ) // PCB CA403001A did NOT have silkscreened U1 - U8 labels
	ROM_LOAD( "a10__f2b5.d27c1000d-15.a10",  0x40000, 0x20000, CRC(2bc4c053) SHA1(cd7668a7733e5e80c2c566d0cf63c4310e5743b4) ) // PCB also labeled as P0-041-1
	ROM_LOAD( "a08__bd49.d27c1000d-15.a8",   0x60000, 0x20000, CRC(8ff8d88c) SHA1(31977e39ad048a077e9b5bd712ff66b14a466d27) )
	ROM_LOAD( "a07__d5f3.d27c1000d-15.a7",   0x80000, 0x20000, CRC(291bcaca) SHA1(4f659a0cd2ff6b4ec04ab95ee8a670222c402c2b) )
	ROM_LOAD( "a05__662a.d27c1000d-15.a5",   0xa0000, 0x20000, CRC(6e762e20) SHA1(66731fe4053b9c09bc9c95d10aba212db08b4636) )
	ROM_LOAD( "a04__0c21.d27c1000d-15.a4",   0xc0000, 0x20000, CRC(e1fd1b9d) SHA1(6027491b927c2ab9c77fbf8895da1abcfbe32d62) )
	ROM_LOAD( "a02__904f.d27c1000d-15.a2",   0xe0000, 0x20000, CRC(2ab06bda) SHA1(2b208b564e55c258665e1f66b26fe14a6c68eb96) )

	ROM_REGION( 0x10000, "pal", 0 )
	/* these are probably shared with extermination except for u35 */
	ROM_LOAD( "b06-12.pal16l8a.u26.jed",  0x00000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-13.pal16l8a.u25.jed",  0x01000, 0x01000, NO_DUMP)
	ROM_LOAD( "st-6.pal16l8a.u35.jed",    0x02000, 0x01000, NO_DUMP)
	ROM_LOAD( "b06-101.pal16l8a.u36.jed", 0x03000, 0x01000, NO_DUMP)
ROM_END

/*
Kabuki Z
Taito, 1988

This PCB runs on Taito/Seta hardware and the exact same newer PCB as The New Zealand Story.
As such, everything here also applies to The New Zealand Story.
Unlike the newer The New Zealand Story pcb, Kabuki Z lacks the daughterboard with the 3rd z80.

PCB Layout
----------

M6100356A (on PCB)
M6100356A (on sticker)
P0-043A (Seta number; on PCB)
|---------------------------------------------------|
|     VOL  HA17408           B50-07.U34  DSWB DSWA  |
|      4558       YM2203 Z80  62256              Z80|
|      4558 YM3014                                  |
|                                                   |
|                                  B06-13           |
|                                   (PAL)           |
|                                                   |
|                                                   |
|                               6264       B50-06.U3|
|J     TESTSW                                       |
|A                                                  |
|M                                                  |
|M                                                  |
|A                                 B06-101          |
|                                    (PAL)          |
|                                                Z80|
|               X1-001A                             |
|                                                   |
|    X1-004                                         |
|               X1-002A       12MHz                 |
|                                                   |
|         B50-01.U46    B50-03.U39                  |
|   X1-006                         6264             |
|X1-007        B50-02.U43   B50-04.U35     B50-05.U1|
|---------------------------------------------------|
Notes:
      All Z80 CPU's running at 6.000MHz (12/2)
      YM2203 running at 3.000Mz (12/4)
      VSync 60Hz
*/

ROM_START( kabukiz )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "b50-05.u1",  0x00000, 0x20000, CRC(9cccb129) SHA1(054faf7657bad7237182e36bcc4388b1748af935) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "b50-08.1e",  0x00000, 0x10000, CRC(cb92d34c) SHA1(3a666f0e3ff9d3daa599123edee228d94eeae754) )

	ROM_REGION( 0x20000, "audiocpu", 0 )    /* 64k + bankswitch areas for the third CPU */
	ROM_LOAD( "b50-07.u34", 0x00000, 0x20000, CRC(bf7fc2ed) SHA1(77008d12d9bdbfa100dcd87cd6ca7de3748408c5) )

	ROM_REGION( 0x200000, "gfx1", 0 )
	ROM_LOAD( "b50-04.u35", 0x000000, 0x80000, CRC(04829aa9) SHA1(a501ec7c802478fc41ec8ef4270b1a6872bcbf34) )
	ROM_LOAD( "b50-03.u39", 0x080000, 0x80000, CRC(31489a4c) SHA1(a4b7e00e2074287b47c7e16add963c1470534376) )
	ROM_LOAD( "b50-02.u43", 0x100000, 0x80000, CRC(90b8a8e7) SHA1(a55e327307606142fbb9d500e757655b35e1f252) )
	ROM_LOAD( "b50-01.u46", 0x180000, 0x80000, CRC(f4277751) SHA1(8f50f843f0eda30d639ba397889236ff0a3edce5) )
ROM_END

ROM_START( kabukizj )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "b50-05.u1",  0x00000, 0x20000, CRC(9cccb129) SHA1(054faf7657bad7237182e36bcc4388b1748af935) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "b50-06.u3",  0x00000, 0x10000, CRC(45650aab) SHA1(00d1fc6044a6ad1e82476ccbe730907b4d780cb9) )

	ROM_REGION( 0x20000, "audiocpu", 0 )    /* 64k + bankswitch areas for the third CPU */
	ROM_LOAD( "b50-07.u34", 0x00000, 0x20000, CRC(bf7fc2ed) SHA1(77008d12d9bdbfa100dcd87cd6ca7de3748408c5) )

	ROM_REGION( 0x200000, "gfx1", 0 )
	ROM_LOAD( "b50-04.u35", 0x000000, 0x80000, CRC(04829aa9) SHA1(a501ec7c802478fc41ec8ef4270b1a6872bcbf34) )
	ROM_LOAD( "b50-03.u39", 0x080000, 0x80000, CRC(31489a4c) SHA1(a4b7e00e2074287b47c7e16add963c1470534376) )
	ROM_LOAD( "b50-02.u43", 0x100000, 0x80000, CRC(90b8a8e7) SHA1(a55e327307606142fbb9d500e757655b35e1f252) )
	ROM_LOAD( "b50-01.u46", 0x180000, 0x80000, CRC(f4277751) SHA1(8f50f843f0eda30d639ba397889236ff0a3edce5) )
ROM_END

ROM_START( insectx )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "b97__03.u32", 0x00000, 0x20000, CRC(18eef387) SHA1(b22633930d39be1e72fbd5b080972122da3cb3ef) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "b97__07.u38", 0x00000, 0x10000, CRC(324b28c9) SHA1(db77a4ac60196d0f0f35dbc5c951ec29d6392463) ) /* Label is B97 07* with an asterisk */

	ROM_REGION( 0x100000, "gfx1", 0 ) /* Mask roms */
	ROM_LOAD( "b97__01.u1", 0x00000, 0x80000, CRC(d00294b1) SHA1(f43a4f7d13193ddbbcdef71a5085c1db0fc062d4) )
	ROM_LOAD( "b97__02.u2", 0x80000, 0x80000, CRC(db5a7434) SHA1(71fac872b19a13a7ad25c8ad895c322ec9573fdc) )
ROM_END

ROM_START( insectxj )
	ROM_REGION( 0x20000, "maincpu", 0 ) /* 64k + bankswitch areas for the first CPU */
	ROM_LOAD( "b97__03.u32", 0x00000, 0x20000, CRC(18eef387) SHA1(b22633930d39be1e72fbd5b080972122da3cb3ef) )

	ROM_REGION( 0x10000, "sub", 0 ) /* 64k for the second CPU */
	ROM_LOAD( "b97__04.u38", 0x00000, 0x10000, CRC(dc4549e5) SHA1(9920f7c12e047ee165418d33b3add51ea615df7e) ) /* Label is B97 04* with an asterisk */

	ROM_REGION( 0x100000, "gfx1", 0 ) /* Mask roms */
	ROM_LOAD( "b97__01.u1", 0x00000, 0x80000, CRC(d00294b1) SHA1(f43a4f7d13193ddbbcdef71a5085c1db0fc062d4) )
	ROM_LOAD( "b97__02.u2", 0x80000, 0x80000, CRC(db5a7434) SHA1(71fac872b19a13a7ad25c8ad895c322ec9573fdc) )
ROM_END


//    YEAR, NAME,      PARENT,   MACHINE,  INPUT,    INIT,     MONITOR,COMPANY,FULLNAME,FLAGS
GAME( 1987, plumppop,  0,        plumppop, plumppop, extrmatn_state, 0, ROT0,   "Taito Corporation", "Plump Pop (Japan)", MACHINE_SUPPORTS_SAVE )
GAME( 1992, jpopnics,  0,        jpopnics, jpopnics, jpopnics_state, 0, ROT0,   "Nics",              "Jumping Pop (Nics, Korean hack of Plump Pop)", MACHINE_IMPERFECT_GRAPHICS | MACHINE_SUPPORTS_SAVE )

GAME( 1987, extrmatn,  0,        extrmatn, extrmatn, extrmatn_state, 0, ROT270, "Taito Corporation Japan",                         "Extermination (World)", MACHINE_SUPPORTS_SAVE )
GAME( 1987, extrmatnu, extrmatn, extrmatn, extrmatn, extrmatn_state, 0, ROT270, "Taito (World Games license)",                     "Extermination (US, World Games)", MACHINE_SUPPORTS_SAVE )
GAME( 1987, extrmatnur,extrmatn, extrmatn, extrmatn, extrmatn_state, 0, ROT270, "Taito America Corporation (Romstar license)",     "Extermination (US, Romstar)", MACHINE_SUPPORTS_SAVE )
GAME( 1987, extrmatnj, extrmatn, extrmatn, extrmatn, extrmatn_state, 0, ROT270, "Taito Corporation",                               "Extermination (Japan)", MACHINE_SUPPORTS_SAVE )

GAME( 1987, arknoid2,  0,        arknoid2, arknoid2, arknoid2_state, 0, ROT270, "Taito Corporation Japan",                     "Arkanoid - Revenge of DOH (World)", MACHINE_SUPPORTS_SAVE )
GAME( 1987, arknoid2u, arknoid2, arknoid2, arknid2u, arknoid2_state, 0, ROT270, "Taito America Corporation (Romstar license)", "Arkanoid - Revenge of DOH (US)", MACHINE_SUPPORTS_SAVE )
GAME( 1987, arknoid2j, arknoid2, arknoid2, arknid2u, arknoid2_state, 0, ROT270, "Taito Corporation",                           "Arkanoid - Revenge of DOH (Japan)", MACHINE_SUPPORTS_SAVE )
GAME( 1987, arknoid2b, arknoid2, arknoid2, arknid2u, arknoid2_state, 0, ROT270, "bootleg",                                     "Arkanoid - Revenge of DOH (Japan bootleg)", MACHINE_SUPPORTS_SAVE )

GAME( 1987, drtoppel,  0,        extrmatn, drtoppel, extrmatn_state, 0, ROT90,  "Kaneko / Taito Corporation Japan",   "Dr. Toppel's Adventure (World)", MACHINE_SUPPORTS_SAVE ) /* Possible region hack */
GAME( 1987, drtoppelu, drtoppel, extrmatn, drtopplu, extrmatn_state, 0, ROT90,  "Kaneko / Taito America Corporation", "Dr. Toppel's Adventure (US)", MACHINE_SUPPORTS_SAVE ) /* Possible region hack */
GAME( 1987, drtoppelj, drtoppel, extrmatn, drtopplu, extrmatn_state, 0, ROT90,  "Kaneko / Taito Corporation",         "Dr. Toppel's Tankentai (Japan)", MACHINE_SUPPORTS_SAVE )

GAME( 1988, kageki,    0,        kageki,   kageki,   kageki_state,   0, ROT90,  "Kaneko / Taito Corporation",                           "Kageki (World)", MACHINE_SUPPORTS_SAVE )
GAME( 1988, kagekiu,   kageki,   kageki,   kagekiu,  kageki_state,   0, ROT90,  "Kaneko / Taito America Corporation (Romstar license)", "Kageki (US)", MACHINE_SUPPORTS_SAVE )
GAME( 1988, kagekij,   kageki,   kageki,   kagekij,  kageki_state,   0, ROT90,  "Kaneko / Taito Corporation",                           "Kageki (Japan)", MACHINE_SUPPORTS_SAVE )
GAME( 1992, kagekih,   kageki,   kageki,   kageki,   kageki_state,   0, ROT90,  "hack",                                                 "Kageki (hack)", MACHINE_SUPPORTS_SAVE ) // date is hacked at least, might also be a Japan set hacked to show english

GAME( 1988, chukatai,  0,        tnzs,     chukatai, tnzs_state,     0, ROT0,   "Taito Corporation Japan",   "Chuka Taisen (World)", MACHINE_SUPPORTS_SAVE ) /* Possible region hack */
GAME( 1988, chukataiu, chukatai, tnzs,     chukatau, tnzs_state,     0, ROT0,   "Taito America Corporation", "Chuka Taisen (US)", MACHINE_SUPPORTS_SAVE ) /* Possible region hack */
GAME( 1988, chukataij, chukatai, tnzs,     chukatau, tnzs_state,     0, ROT0,   "Taito Corporation",         "Chuka Taisen (Japan)", MACHINE_SUPPORTS_SAVE )

GAME( 1988, tnzs,      0,        tnzsb,    tnzs,     tnzsb_state,    0, ROT0,   "Taito Corporation Japan",   "The NewZealand Story (World, new version) (newer PCB)", MACHINE_SUPPORTS_SAVE )
GAME( 1988, tnzsj,     tnzs,     tnzsb,    tnzsj,    tnzsb_state,    0, ROT0,   "Taito Corporation",         "The NewZealand Story (Japan, new version) (newer PCB)", MACHINE_SUPPORTS_SAVE )
GAME( 1988, tnzso,     tnzs,     tnzs,     tnzsop,   tnzs_state,     0, ROT0,   "Taito Corporation Japan",   "The NewZealand Story (World, old version) (older PCB)", MACHINE_SUPPORTS_SAVE )
GAME( 1988, tnzsjo,    tnzs,     tnzs,     tnzsjo,   tnzs_state,     0, ROT0,   "Taito Corporation",         "The NewZealand Story (Japan, old version) (older PCB)", MACHINE_SUPPORTS_SAVE )
GAME( 1988, tnzsuo,    tnzs,     tnzs,     tnzsjo,   tnzs_state,     0, ROT0,   "Taito America Corporation", "The NewZealand Story (US, old version) (older PCB)",    MACHINE_SUPPORTS_SAVE )
GAME( 1988, tnzsoa,    tnzs,     tnzs,     tnzsop,   tnzs_state,     0, ROT0,   "Taito Corporation Japan",   "The NewZealand Story (World, unknown version) (older PCB)", MACHINE_SUPPORTS_SAVE )
GAME( 1988, tnzsop,    tnzs,     tnzs,     tnzsop,   tnzs_state,     0, ROT0,   "Taito Corporation Japan",   "The NewZealand Story (World, prototype) (older PCB)", MACHINE_SUPPORTS_SAVE )

GAME( 1988, kabukiz,   0,        kabukiz,  kabukiz,  kabukiz_state,  0, ROT0,   "Kaneko / Taito Corporation Japan", "Kabuki-Z (World)", MACHINE_SUPPORTS_SAVE )
GAME( 1988, kabukizj,  kabukiz,  kabukiz,  kabukizj, kabukiz_state,  0, ROT0,   "Kaneko / Taito Corporation",       "Kabuki-Z (Japan)", MACHINE_SUPPORTS_SAVE )

GAME( 1989, insectx,   0,        insectx,  insectx,  insectx_state,  0, ROT0,   "Taito Corporation Japan", "Insector X (World)", MACHINE_SUPPORTS_SAVE )
GAME( 1989, insectxj,  insectx,  insectx,  insectxj, insectx_state,  0, ROT0,   "Taito Corporation",       "Insector X (Japan)", MACHINE_SUPPORTS_SAVE )
