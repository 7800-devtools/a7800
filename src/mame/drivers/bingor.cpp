// license:BSD-3-Clause
// copyright-holders:Angelo Salese, Roberto Fresca
/***********************************************************************

  Bingo Roll / Bell Star


************************************************************************


  PCB Layout (Bell Star V3):

                        ___________________________________________
   ____________________| | | | | | | | | | | | | | | | | | | | | | |____________________
  |                                  22x2 edge connector                                |
  |                     ___________   ___________   ___________                         |
  |                    |  9CANH1M  | |  9CANH1M  | |  9CANH1M  |                        |
  |                    | ULN2003AN | | ULN2003AN | | ULN2003AN |                        |
  |                    |___________| |___________| |___________|                        |
  |                                                                                     |
  |       _______       ______________________________                                  |
  |      |JM04AB |     |                              |                                 |
  |      |LM/386N|     |      P8256AH  L6140860       |                                 |
  |      |_______|     |                              |                                 |
  |                    |         Intel 1975           |                                 |
  |                    |______________________________|                                 |
  |                                                                                     |
  |         ____________    ___________     ____________    ____________                |
  |        |  9BARTCK   |  | YMZ284-D  |   | 898-3-R10K |  | 898-3-R10K |               |
  |        | SN74HCTD0N |  | 9814QAAA  |   |            |  |            |               |
  |        |____________|  |___________|   |____________|  |____________|               |
  |                                                                                     |
  | _____   _______________                                                             |
  ||     | |               |                _____________   _____________               |
  ||     | | MHS E         |               |   86AEY4K   | |   86AEY4K   |              |
  ||     | | S-82716-4     |               | SN74HCT540N | | SN74HCT540N |              |
  || (1) | | 8944          |               |_____________| |_____________|              |
  ||     | |               |                _____________   _____________    _________  |
  ||     | |               |               |   83C931K   | |   83C931K   |  | 88A59LM | |
  ||     | |               |               | SN74HCT573N | | SN74HCT573N |  | LM339N  | |
  ||     | |               |               |_____________| |_____________|  |_________| |
  ||_____| |_______________|                                                            |
  |                                                                                     |
  |         _______________                 ________________    ________________        |
  |        |               |               |  Bell Star V3  |  |  Bell Star V3  |       |
  |        | N800186-12    |               |  V.3.6.02 L    |  |  V.3.6.02 H    |       |
  |        | 9941GPA BR    |               |  TMS27C256-10  |  |  TMS27C256-10  |       |
  |        | 8944          |               |________________|  |________________|       |
  |        | AMD           |                ________________    ________________        |
  |        | INTEL '78 '82 |               |  B.S. Grafik   |  |  B.S. Grafik   |       |
  |        |               |               |  11.10.02 L    |  |  11.10.02 H    |       |
  |        |               |               |   M27C256B     |  |   M27C256B     |       |
  |        |_______________|               |________________|  |________________|       |
  |                                                                                     |
  |                                         ________________    ________________        |
  |           _____________                |    HY62256B    |  |   HY62256B     |       |
  |          |   83C931K   |               |     LP-70      |  |    LP-70       |       |
  |          | SN74HCT573N |               |  9846B KOREA   |  |   9846B KOREA  |       |
  |          |_____________|               |________________|  |________________|       |
  |                                                                                     |
  |         _____________________                                                       |
  |        |      74HCT540N      |        __________   ________   __________   _______  |
  |        | C5944ME / Hnn9931 D |       |  86A4CY  | |74HCT08N| | 06F6L9K  | |ADM690 | |
  |        |_____________________|       |SN74HCT02N| |D0945PS | |SN74HCT32N| |AN/9927| |
  |                                      |__________| |________| |__________| |_______| |
  |                                                                                     |
  |            ______________             __________    ________________                |
  |           |  PIC 16C54C  |           |          |  |                |               |
  |           |    04/P      |           |  24C04A  |  | 3.6 V. BATTERY |               |
  |           |______________|           |__________|  |________________|               |
  |                                                                                     |
  |_____________________________________________________________________________________|


  (1) Siemens HYB5141758J-60 (Germany).

  - 1x Xtal 16 MHz.
  - 1x Xtal ?? MHz.


  Etched in copper:

  "VIP-3 PALOMA ELEKTRONIK"
  "gruber electronic gmbh"
  "KMG-Layout 007051"
  "Rev 1.0"
  "0103 VIP 3"


************************************************************************

  Pinout:

  ROLLING JOKER+ / BELL STAR+

  +--------------------+----------------------+
  |        Solder      |       Component      |
  +----------------+---+----+-----------------+
  |           GND  | A |  1 |  GND            |
  |           GND  | B |  2 |  GND            |
  |           GND  | C |  3 |  GND            |
  |       +5 Volt  | D |  4 |  +5 Volt        |
  |      +12 Volt  | E |  5 |  +12 Volt       |
  |    Lamp Hold1  | F |  6 |  Lamp Start     |
  |    Lamp Hold2  | H |  7 |  Lamp Hold5     |
  |    Lamp Clear  | J |  8 |  Lamp Hold4     |
  |        Coin 1  | K |  9 |  Lamp Hold3     |
  |    Counter In  | L | 10 |  Remote         |
  |   Counter Out  | M | 11 |  Button Hold1   |
  |Counter Remote  | N | 12 |  Button Clear   |
  |  Button Hold5  | P | 13 |  Button Start   |
  |  Owner Bookk.  | R | 14 |  Rental Bookk.  |
  |  Button Hold2  | S | 15 |  Button Hold4   |
  |        Coin 2  | T | 16 |  Button Hold3   |
  |            nc  | U | 17 |  Hopper Count   |
  |            nc  | V | 18 |  Hopper Drive   |
  |   Video Green  | W | 19 |  Video Red      |
  |    Video Sync  | X | 20 |  Video Blue     |
  |  Speaker 1(+)  | Y | 21 |  Video Ground   |
  |  Credit Clear  | Z | 22 |  Speaker 1(-)   |
  +----------------+---+----+-----------------+

************************************************************************

  PCB Layout (BINGOR1):

              +--------------------------+
              ||||||||||||||||||||||||||||
  +-----------+    22x2 edge connector   +-----------+
  |                                                  |
  |                      +-----+ +-----+             |
  |                      |  M  | |  M  |             |
  |                      +-----+ +-----+             |
  |                    +-----------------+           |
  |                    | P825GAH         |           |
  |                    | L4203043        |           |
  |                    | INTEL 1975      |           |
  |                    +-----------------+           |
  |     +-----+ +-----+                              |
  |     |  L  | |  L  |                              |
  |     +-----+ +-----+                              |
  |     +-----+ +-----+                              |
  |     |  K  | |  K  |                              |
  |     +-----+ +-----+                              |
  |      +---------------------------------------+   |
  |      |   +-----+                    XTAL     |   |
  |      |   |  I  |                   .----.    |   |
  |      |   +-----+                             |   |
  |      | +--------------+                      |   |
  |      | |BINGO         |         +----------+ |   |
  |      | |V.29.4.95 H   |         |N80C186-12| |   |
  |      | |              |         |BR 9709CPA| |   |
  |      | +--------------+ +------+|AMD       | |   |
  |      | +--------------+ |  K   ||INTEL78 82| |   |
  | +--+ | |BINGO         | +------+|          | |   |
  | |  | | |V.29.4.99 L   | +------+|          | |   |
  | |J | | |              | |  I   |+----------+ |   |
  | |  | | +--------------+ +------+             |   |
  | |  | |   +-----+      +------+ +---+ +-----+ |   |
  | +--+ |   |  I  |      |  H   | | G | |  F  | |   |
  |      |   +-----+      +------+ +---+ +-----+ |   |
  |      +---------------------------------------+   |
  |                        +------+      +------+    |
  |                        |  D   |      |  E   |    |
  |                        +------+      +------+    |
  |    +----------+                                  |
  |    |   MHS    |  +------------+ +------------+   |
  |    | S-82716-4|  |  HYUNDAI   | |  HYUNDAI   |   |
  |    |          |  |HY6116ALP-10| |HY6116ALP-10|   |
  |    |     9210 |  |9306A  KOREA| |9306A  KOREA|   |
  |    |          |  +------------+ +------------+   |
  |    |          |  +------------+ +------------+   |
  |    +----------+  |BINGO       | |BINGO       |   |
  |                  |Turbo  L    | |Turbo  H    |   |
  |                  |     M27C512| |     M27C512|   |
  | +--++--++--++--+ +------------+ +------------+   |
  | |  ||  ||  ||  |          +-----+ +-------+      |
  | |C ||C ||C ||C |   ----   |  B  | |       |      |
  | |  ||  ||  ||  |  /Batt\  +-----+ +-------+      |
  | |  ||  ||  ||  | [======]           +---+        |
  | +--++--++--++--+  \ery /            | A |        |
  |                    ----             +---+        |
  +--------------------------------------------------+

  1x XTAL = 16 MHz.

  A = 9423 / LTC690 / CN8
  B = P9242 / MM74HCT08N
  C = NEC USA / D41464C-10
  D = 46COJFK / SN74HCT373N
  E = PC74HCT32P / 751180T / Hnn9145PB
  F = SN74HCT02N
  G = 24C04A / /P CFW / 9512
  H = PIC16C54B / 04/P /98245AJ
  I = SN74HCT573N
  J = SAA1099P / 232300 / DSG9336M2
  K = SN74HCT540N
  L = 898-3-R10K / 9343
  M = ULN2003A / WB91K9409 / PHIL


************************************************************************

  PCB Layout (BINGOR2):

              +--------------------------+
              ||||||||||||||||||||||||||||
  +-----------+    22x2 edge connector   +-----------+
  |                                                  |
  |                      +-----+ +-----+             |
  |                      |  M  | |  M  |             |
  |                      +-----+ +-----+             |
  |                    +-----------------+           |
  |                    | P825GAH         |           |
  |                    | L4203043        |           |
  |                    | INTEL 1975      |           |
  |                    +-----------------+           |
  |     +-----+ +-----+                              |
  |     |  L  | |  L  |                              |
  |     +-----+ +-----+                              |
  |     +-----+ +-----+                              |
  |     |  K  | |  K  |                              |
  |     +-----+ +-----+                              |
  |      +---------------------------------------+   |
  |      |   +-----+                    XTAL     |   |
  |      |   |  I  |                   .----.    |   |
  |      |   +-----+                             |   |
  |      | +--------------+                      |   |
  |      | |BINGO ROLL    |  +----+ +----------+ |   |
  |      | |Vip2 v.26.2.02|  | N  | |N80C186-12| |   |
  |      | |             H|  +----+ |BR 9709CPA| |   |
  |      | +--------------+ +------+|AMD       | |   |
  |      | +--------------+ |  K   ||INTEL78 82| |   |
  | +--+ | |BINGO ROLL    | +------+|          | |   |
  | |  | | |Vip2 v.26.2.02| +------+|          | |   |
  | |J | | |             L| |  I   |+----------+ |   |
  | |  | | +--------------+ +------+             |   |
  | |  | |   +-----+      +------+ +---+ +-----+ |   |
  | +--+ |   |  I  |      |  H   | | G | |  F  | |   |
  |      |   +-----+      +------+ +---+ +-----+ |   |
  |      +---------------------------------------+   |
  |                        +------+      +------+    |
  |                        |  D   |      |  E   |    |
  |                        +------+      +------+    |
  |    +----------+                                  |
  |    |   MHS    |  +------------+ +------------+   |
  |    | S-82716-4|  |HM6116L-70  | |HM6116L-70  |   |
  |    |          |  |9914H C9423 | |9914H C9423 |   |
  |    |     9210 |  |            | |            |   |
  |    |          |  +------------+ +------------+   |
  |    |          |  +------------+ +------------+   |
  |    +----------+  |BINGO ROLL  | |BINGO ROLL  |   |
  |                  |GRAFIK  L   | |GRAFIK  H   |   |
  |                  |            | |            |   |
  | +--++--++--++--+ +------------+ +------------+   |
  | |  ||  ||  ||  |          +-----+ +-------+      |
  | |C ||C ||C ||C |   ----   |  B  | |       |      |
  | |  ||  ||  ||  |  /Batt\  +-----+ +-------+      |
  | |  ||  ||  ||  | [======]           +---+        |
  | +--++--++--++--+  \ery /            | A |        |
  |                    ----             +---+        |
  +--------------------------------------------------+

  1x XTAL = 16 MHz.

  A = ACM690AN / 9903
  B = P9242 / MM74HCT08N
  C = NEC USA / D41464C-10
  D = 46COJFK / SN74HCT373N
  E = SN74HC132N
  F = SN74HCT02N
  G = 24C04A / /P CFW / 9512
  H = PIC16C54C / 04/P /98245AJ
  I = SN74HCT573N
  J = SAA1099P / 218080 / DSG9249M2
  K = SN74HCT540N
  L = 898-3-R10K / 9836
  M = ULN2003A / WB91K9409 / PHIL
  N = LW339N / CPB6D143


************************************************************************

  PCB Layout (BINGOR3):

              +--------------------------+
              ||||||||||||||||||||||||||||
  +-----------+    22x2 edge connector   +-----------+
  |                                                  |
  |                      +-----+ +-----+             |
  |                      |  M  | |  M  |             |
  |                      +-----+ +-----+             |
  |                    +-----------------+           |
  |                    | P825GAH         |           |
  |                    | L4203043        |           |
  |                    | INTEL 1975      |           |
  |                    +-----------------+           |
  |     +-----+ +-----+                              |
  |     |  L  | |  L  |                              |
  |     +-----+ +-----+                              |
  |     +-----+ +-----+                              |
  |     |  K  | |  K  |                              |
  |     +-----+ +-----+                              |
  |      +---------------------------------------+   |
  |      |   +-----+                    XTAL     |   |
  |      |   |  I  |                   .----.    |   |
  |      |   +-----+                             |   |
  |      | +--------------+                      |   |
  |      | |BELL STAR     |         +----------+ |   |
  |      | |VIP2H 27.07   |         |N80C186-12| |   |
  |      | |              |         |BR 9709CPA| |   |
  |      | +--------------+ +------+|AMD       | |   |
  |      | +--------------+ |  K   ||INTEL78 82| |   |
  | +--+ | |BELL STAR     | +------+|          | |   |
  | |  | | |VIP2L 27.07   | +------+|          | |   |
  | |J | | |              | |  I   |+----------+ |   |
  | |  | | +--------------+ +------+             |   |
  | |  | |   +-----+      +------+ +---+ +-----+ |   |
  | +--+ |   |  I  |      |  H   | | G | |  F  | |   |
  |      |   +-----+      +------+ +---+ +-----+ |   |
  |      +---------------------------------------+   |
  |                        +------+      +------+    |
  |                        |  D   |      |  E   |    |
  |                        +------+      +------+    |
  |    +----------+                                  |
  |    |   MHS    |  +------------+ +------------+   |
  |    | S-82716-4|  |HM6116L-70  | |HM6116L-70  |   |
  |    |          |  |9914H C9423 | |9914H C9423 |   |
  |    |     9210 |  |            | |            |   |
  |    |          |  +------------+ +------------+   |
  |    |          |  +------------+ +------------+   |
  |    +----------+  |B.S.GRAFIK  | |B.S.GRAFIK  |   |
  |                  |11.10.02  L | |11.10.02  H |   |
  |                  |            | |            |   |
  | +--++--++--++--+ +------------+ +------------+   |
  | |  ||  ||  ||  |          +-----+ +-------+      |
  | |C ||C ||C ||C |   ----   |  B  | |       |      |
  | |  ||  ||  ||  |  /Batt\  +-----+ +-------+      |
  | |  ||  ||  ||  | [======]           +---+        |
  | +--++--++--++--+  \ery /            | A |        |
  |                    ----             +---+        |
  +--------------------------------------------------+

  1x XTAL = 16 MHz.

  A = ACM690AN / 9903
  B = P9242 / MM74HCT08N
  C = NEC USA / D41464C-10
  D = 46COJFK / SN74HCT373N
  E = SN74HC132N
  F = SN74HCT02N
  G = 24C04A / /P CFW / 9512
  H = PIC16C54A / 04/P /98245AJ
  I = SN74HCT573N
  J = SAA1093P / 218080 / DSG9249M2
  K = SN74HCT540N
  L = 898-3-R10K / 9921
  M = ULN2003A / WB91K9409 / PHIL


************************************************************************

  PCB Layout (BINGOR4):

              +--------------------------+
              ||||||||||||||||||||||||||||
  +-----------+    22x2 edge connector   +-----------+
  |                                                  |
  |                      +-----+ +-----+             |
  |                      |  M  | |  M  |             |
  |                      +-----+ +-----+             |
  |                    +-----------------+           |
  |                    | P825GAH         |           |
  |                    | L4203043        |           |
  |                    | INTEL 1975      |           |
  |                    +-----------------+           |
  |     +-----+ +-----+                              |
  |     |  L  | |  L  |                              |
  |     +-----+ +-----+                              |
  |     +-----+ +-----+                              |
  |     |  K  | |  K  |                              |
  |     +-----+ +-----+                              |
  |      +---------------------------------------+   |
  |      |   +-----+                    XTAL     |   |
  |      |   |  I  |                   .----.    |   |
  |      |   +-----+                             |   |
  |      | +--------------+                      |   |
  |      | |BELL STAR     |         +----------+ |   |
  |      | |VIP2H 23.07   |         |N80C186-12| |   |
  |      | |              |         |BR 9709CPA| |   |
  |      | +--------------+ +------+|AMD       | |   |
  |      | +--------------+ |  K   ||INTEL78 82| |   |
  | +--+ | |BELL STAR     | +------+|          | |   |
  | |  | | |VIP2L 23.07   | +------+|          | |   |
  | |J | | |              | |  I   |+----------+ |   |
  | |  | | +--------------+ +------+             |   |
  | |  | |   +-----+      +------+ +---+ +-----+ |   |
  | +--+ |   |  I  |      |  H   | | G | |  F  | |   |
  |      |   +-----+      +------+ +---+ +-----+ |   |
  |      +---------------------------------------+   |
  |                        +------+      +------+    |
  |                        |  D   |      |  E   |    |
  |                        +------+      +------+    |
  |    +----------+                                  |
  |    |   MHS    |  +------------+ +------------+   |
  |    | S-82716-4|  |HM6116L-70  | |HM6116L-70  |   |
  |    |          |  |9914H C9423 | |9914H C9423 |   |
  |    |     9210 |  |            | |            |   |
  |    |          |  +------------+ +------------+   |
  |    |          |  +------------+ +------------+   |
  |    +----------+  |B.S.GRAFIK  | |B.S.GRAFIK  |   |
  |                  |11.10.02  L | |11.10.02  H |   |
  |                  |            | |            |   |
  | +--++--++--++--+ +------------+ +------------+   |
  | |  ||  ||  ||  |          +-----+ +-------+      |
  | |C ||C ||C ||C | +------+ |  B  | |       |      |
  | |  ||  ||  ||  | |      | +-----+ +-------+      |
  | |  ||  ||  ||  | |Batter|           +---+        |
  | +--++--++--++--+ |y     |           | A |        |
  |                  +------+           +---+        |
  +--------------------------------------------------+

  1x XTAL = 16 MHz.

  A = ACM690AN / 9903
  B = P9242 / MM74HCT08N
  C = NEC USA / D41464C-10
  D = 46COJFK / SN74HCT373N
  E = SN74HC132N
  F = SN74HCT02N
  G = 24C04A / /P CFW / 9512
  H = PIC16C54A / 04/P /98245AJ
  I = SN74HCT573N
  J = SAA1099P / 232230 / DSG9249M2
  K = SN74HCT540N
  L = 898-3-R10K / 9921
  M = ULN2003A / WB91K9409 / PHIL


************************************************************************

  PCB Layout (VIP2000):

     +---------------------------------------------------------------------------------+
     |                                            +--------+                           |
     |                                            |74HCT32N|                  BATTERY  |
     |                           +----+  +----+   +--------+                 3.6V 80mAh|
     |                           |HY62|  |VIP |                        +-+             |
     |              +-+ +-+ +-+  |256B|  |BIOS|    +-----+   +-+  +-+  |D|             |
     |   +-+   +-+  | | | | | |  |    |  |8 H |    |AT49F|   | |  |E|  +-+         +---+
  +--+   | |   | |  | | | | | |  |    |  |    |    |4096-|   | |  |M|  +-+   +-+   |   +-+
  |--    |K|   |A|  |B| |C| |C|  |    |  |    |    |12RC |   |C|  |P|  | |   | |   |RS232|
  |--    | |   | |  | | | | | |  |    |  |M272|    |     |   | |  |T|  |I|   |L|   |     |
  |--    | |   | |  | | | | | |  |    |  |56  |    |     |   | |  |Y|  | |   | |   |   +-+
  |--    +-+   +-+  +-+ +-+ +-+  +----+  +----+    +-----+   +-+  +-+  +-+   +-+   +---+
  |--                                                                                  |
  |--2              +-+ +-+ +-+  +----+  +----+   +-+ +-+  +-+ +----+ +-+ +----+ +----+|
  |--2   +-+   +-+  | | | | | |  |HY62|  |VIP |   | | | |  | | |    | | | |HY62| |SLAV||
  |--P   | |   | |  | | | | | |  |256B|  |BIOS|   | | | |  |M| |P80C| | | |256B| |E   ||
  |--I   |K|   |A|  |B| |C| |C|  |    |  |8 H |   |C| |C|  | | |31BH| |C| |    | |19.0||
  |--N   | |   | |  | | | | | |  |    |  |    |   | | | |  +-+ |I   | | | |    | |9.01||
  |--    | |   | |  | | | | | |  |    |  |    |   | | | |  +-+ |    | | | |    | |    ||
  |--C   +-+   +-+  +-+ +-+ +-+  |    |  |M272|   +-+ +-+  | | |    | +-+ |    | |2725||
  |--O                           |    |  |56  |            |N| |    |     |    | |6   ||
  |--N   +-+        +-+          +----+  +----+            | | |    |     +----+ +----+|
  |--N   |:|   +-+  | | +-+                                +-+ |    |                  |
  |--E   |:|   | |  | | | |                                  X |    |                  |
  |--C   |:|   |A|  |B| |H|                                  T |    |                  |
  |--T   |:|   | |  | | | |  +----------+    +----------+    A |    |                  |
  |--O   |:|   | |  | | | |  |  MHS E   | X  |N80C186-12|    L +----+    +-+           |
  |--R   |:|   +-+  +-+ +-+  | S 82716-4| T  |0037CXC BR|    3           | |  +-+  +-+ |
  |--    |:|                 |          | A  |          |                |G|  |:|  |:| |
  |--    |:|            +-+  | 8941     | L  |(c) AMD   |X     +-+  +-+  | |  |:|  |:| |
  +--+   +-+            | |  |          | 1  |          |T     |E|  |F|  | |  +-+  +-+ |
     |         +-+      |I|  |          |    |          |A     +-+  +-+  +-+           |
     |         |J|      | |  +----------+    +----------+L                    +-+ +--+ |
     |         +-+      +-+                              2     +-+  +-+  +-+  |:| |::| |
     |                       +----------+                      | |  | |  | |  |:| |::| |
     |                       |HYB514175B|                      |O|  |O|  |K|  |:| |::| |
     |                       |J-60      |                      | |  | |  | |  |:| +--+ |
     |                       +----------+       VIP 2000       +-+  +-+  +-+  +-+      |
     |                                   (c)PALOMA-ELEKTRONIK                          |
     +---------------------------------------------------------------------------------+

  XTAL1: 10.0000M
  XTAL2: R240VB40
  XTAL3: 11.0592M

  A: 898-3-R10K / 0035
  B: 74HCT540N
  C: 74HCT573N
  D: ADM690
  E: 24C04A
  F: EMPTY SOCKET
  G: HEF40106BP
  H: YMZ284
  I: 74HCT00N
  J: EMI14AB LM 386N-4
  K: ULN2003JN
  L: MAX232N
  M: 74HCT32N
  N: 74HCT02N
  O: PIC16F627-04 (protected)


************************************************************************/

#include "emu.h"
#include "cpu/i86/i186.h"
#include "cpu/mcs51/mcs51.h"
#include "cpu/pic16c5x/pic16c5x.h"
#include "machine/intelfsh.h"
#include "machine/msm6242.h"
#include "sound/ay8910.h"
#include "sound/saa1099.h"
#include "screen.h"
#include "speaker.h"


class bingor_state : public driver_device
{
public:
	bingor_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_blit_ram(*this, "blit_ram"),
		m_maincpu(*this, "maincpu"),
		m_slavecpu(*this, "slavecpu"),
		m_palette(*this, "palette")  { }

	required_shared_ptr<uint16_t> m_blit_ram;
	virtual void video_start() override;
	uint32_t screen_update_bingor(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect);
	required_device<cpu_device> m_maincpu;
	optional_device<cpu_device> m_slavecpu;
	required_device<palette_device> m_palette;

	DECLARE_WRITE8_MEMBER(toslave_w);
	DECLARE_READ8_MEMBER(toslave_r);
	DECLARE_WRITE8_MEMBER(fromslave_w);
	DECLARE_READ8_MEMBER(fromslave_r);
	DECLARE_WRITE16_MEMBER(vip2000_outputs_w);
	u8 m_toslave;
	u8 m_fromslave;
};


void bingor_state::video_start()
{
}

uint32_t bingor_state::screen_update_bingor(screen_device &screen, bitmap_rgb32 &bitmap, const rectangle &cliprect)
{
	int x,y,count;

	bitmap.fill(m_palette->black_pen(), cliprect);

	count = (0x2000/2);

	for(y=0;y<256;y++)
	{
		for(x=0;x<286;x+=4)
		{
			uint32_t color;

			color = (m_blit_ram[count] & 0xf000)>>12;

			if(cliprect.contains(x+3, y))
				bitmap.pix32(y, x+3) = m_palette->pen(color);

			color = (m_blit_ram[count] & 0x0f00)>>8;

			if(cliprect.contains(x+2, y))
				bitmap.pix32(y, x+2) = m_palette->pen(color);

			color = (m_blit_ram[count] & 0x00f0)>>4;

			if(cliprect.contains(x+1, y))
				bitmap.pix32(y, x+1) = m_palette->pen(color);

			color = (m_blit_ram[count] & 0x000f)>>0;

			if(cliprect.contains(x+0, y))
				bitmap.pix32(y, x+0) = m_palette->pen(color);

			count++;
		}
	}

	return 0;
}


static ADDRESS_MAP_START( bingor_map, AS_PROGRAM, 16, bingor_state )
	AM_RANGE(0x00000, 0x0ffff) AM_RAM
	AM_RANGE(0x90000, 0x9ffff) AM_ROM AM_REGION("gfx", 0)
	AM_RANGE(0xa0300, 0xa031f) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette") //wrong
	AM_RANGE(0xa0000, 0xaffff) AM_RAM AM_SHARE("blit_ram")
	AM_RANGE(0xf0000, 0xfffff) AM_ROM AM_REGION("boot_prg", 0)
ADDRESS_MAP_END

static ADDRESS_MAP_START( bingor2_map, AS_PROGRAM, 16, bingor_state )
	AM_RANGE(0xe0000, 0xfffff) AM_ROM AM_REGION("boot_prg", 0) // banked?
	AM_IMPORT_FROM(bingor_map)
ADDRESS_MAP_END

static ADDRESS_MAP_START( bingor_io, AS_IO, 16, bingor_state )
	AM_RANGE(0x0100, 0x0103) AM_DEVWRITE8("saa", saa1099_device, write, 0x00ff)
ADDRESS_MAP_END


static INPUT_PORTS_START( bingor )
	PORT_START("IN0")
	PORT_DIPNAME( 0x0001, 0x0001, "IN0" )
	PORT_DIPSETTING(      0x0001, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0002, 0x0002, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0002, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0004, 0x0004, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0004, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0008, 0x0008, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0008, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0010, 0x0010, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0010, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0020, 0x0020, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0020, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0040, 0x0040, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0040, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0080, 0x0080, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0080, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0100, 0x0100, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0100, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0200, 0x0200, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0200, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0400, 0x0400, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0400, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x0800, 0x0800, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x0800, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x1000, 0x1000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x1000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x2000, 0x2000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x2000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x4000, 0x4000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x4000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
	PORT_DIPNAME( 0x8000, 0x8000, DEF_STR( Unknown ) )
	PORT_DIPSETTING(      0x8000, DEF_STR( Off ) )
	PORT_DIPSETTING(      0x0000, DEF_STR( On ) )
INPUT_PORTS_END


static const gfx_layout bingor_layout =
{
	8,8,
	RGN_FRAC(1,1),
	4,
	{ 0,1,2,3 },
	{ 0,4,8,12,16,20,24,28 },
	{ 0*32, 1*32, 2*32, 3*32, 4*32, 5*32, 6*32, 7*32 },
	8*32
};

static GFXDECODE_START( bingor )
	GFXDECODE_ENTRY( "gfx", 0, bingor_layout,   0x0, 2  )
GFXDECODE_END


static MACHINE_CONFIG_START( bingor )
	MCFG_CPU_ADD("maincpu", I80186, XTAL_16MHz)
	MCFG_CPU_PROGRAM_MAP(bingor_map)
	MCFG_CPU_IO_MAP(bingor_io)
	MCFG_CPU_PERIODIC_INT_DRIVER(bingor_state, nmi_line_pulse, 30)
	// Other interrupts generated by internal timers

	MCFG_CPU_ADD("pic", PIC16C57, 12000000) //?? Mhz

	MCFG_GFXDECODE_ADD("gfxdecode", "palette", bingor)
	//MCFG_NVRAM_ADD_0FILL("nvram")

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(400, 300)
	MCFG_SCREEN_VISIBLE_AREA(0, 400-1, 0, 300-1)
	MCFG_SCREEN_UPDATE_DRIVER(bingor_state, screen_update_bingor)

	MCFG_PALETTE_ADD("palette", 0x100)
	MCFG_PALETTE_FORMAT(RRRRGGGGBBBBIIII)

	MCFG_SPEAKER_STANDARD_MONO("mono")
	MCFG_SAA1099_ADD("saa", 6000000 )
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)
MACHINE_CONFIG_END

static MACHINE_CONFIG_DERIVED( bingor2, bingor )
	MCFG_CPU_MODIFY("maincpu")
	MCFG_CPU_PROGRAM_MAP(bingor2_map)
MACHINE_CONFIG_END


static ADDRESS_MAP_START( vip2000_map, AS_PROGRAM, 16, bingor_state )
	AM_RANGE(0x00000, 0x0ffff) AM_RAM
	AM_RANGE(0x40300, 0x4031f) AM_RAM_DEVWRITE("palette", palette_device, write) AM_SHARE("palette") //wrong
	AM_RANGE(0x40000, 0x4ffff) AM_RAM AM_SHARE("blit_ram")
	//AM_RANGE(0x50000, 0x5ffff) AM_ROM AM_REGION("gfx", 0)
	AM_RANGE(0x60000, 0x60003) AM_DEVWRITE8("ymz", ymz284_device, address_data_w, 0x00ff)
	AM_RANGE(0x80000, 0xeffff) AM_DEVREADWRITE("flash", intelfsh16_device, read, write)
	AM_RANGE(0xf0000, 0xfffff) AM_ROM AM_REGION("boot_prg", 0)
ADDRESS_MAP_END

static ADDRESS_MAP_START( vip2000_io, AS_IO, 16, bingor_state )
	AM_RANGE(0x0000, 0x0001) AM_READNOP // watchdog
	AM_RANGE(0x0080, 0x009f) AM_DEVREADWRITE8("rtc", msm6242_device, read, write, 0x00ff)
	AM_RANGE(0x0100, 0x0101) AM_READWRITE8(fromslave_r, toslave_w, 0x00ff)
	AM_RANGE(0x0280, 0x0281) AM_WRITE(vip2000_outputs_w)
ADDRESS_MAP_END

WRITE8_MEMBER(bingor_state::toslave_w)
{
	m_toslave = data;
}

READ8_MEMBER(bingor_state::toslave_r)
{
	return m_toslave;
}

WRITE8_MEMBER(bingor_state::fromslave_w)
{
	m_fromslave = data;
}

READ8_MEMBER(bingor_state::fromslave_r)
{
	return m_fromslave;
}

WRITE16_MEMBER(bingor_state::vip2000_outputs_w)
{
	m_slavecpu->set_input_line(MCS51_INT0_LINE, BIT(data, 15) ? CLEAR_LINE : ASSERT_LINE);
}

static ADDRESS_MAP_START( slave_map, AS_PROGRAM, 8, bingor_state )
	AM_RANGE(0x0000, 0x7fff) AM_ROM
ADDRESS_MAP_END

static ADDRESS_MAP_START( slave_io, AS_IO, 8, bingor_state)
	AM_RANGE(0x0000, 0x0000) AM_READWRITE(toslave_r, fromslave_w)
	AM_RANGE(0xc000, 0xcfff) AM_RAM
ADDRESS_MAP_END

static MACHINE_CONFIG_START( vip2000 )
	MCFG_CPU_ADD("maincpu", I80186, XTAL_10MHz)
	MCFG_CPU_PROGRAM_MAP(vip2000_map)
	MCFG_CPU_IO_MAP(vip2000_io)
	MCFG_CPU_PERIODIC_INT_DRIVER(bingor_state, nmi_line_pulse, 30)
	// Other interrupts generated by internal timers

	MCFG_ATMEL_49F4096_ADD("flash")

	MCFG_CPU_ADD("slavecpu", I80C31, XTAL_11_0592MHz)
	MCFG_CPU_PROGRAM_MAP(slave_map)
	MCFG_CPU_IO_MAP(slave_io)

	MCFG_DEVICE_ADD("rtc", MSM6242, XTAL_32_768kHz)

	MCFG_SCREEN_ADD("screen", RASTER)
	MCFG_SCREEN_REFRESH_RATE(60)
	MCFG_SCREEN_VBLANK_TIME(ATTOSECONDS_IN_USEC(0))
	MCFG_SCREEN_SIZE(400, 300)
	MCFG_SCREEN_VISIBLE_AREA(0, 400-1, 0, 300-1)
	MCFG_SCREEN_UPDATE_DRIVER(bingor_state, screen_update_bingor)

	MCFG_PALETTE_ADD("palette", 0x100)
	MCFG_PALETTE_FORMAT(RRRRGGGGBBBBIIII)

	MCFG_SPEAKER_STANDARD_MONO("mono")

	MCFG_SOUND_ADD("ymz", YMZ284, 1250000) // probably clocked by square wave output of 80186 timer 0
	MCFG_SOUND_ROUTE(ALL_OUTPUTS, "mono", 0.50)
MACHINE_CONFIG_END

// I doubt we need to load the eeproms

ROM_START( bingor1 )
	ROM_REGION( 0x10000, "boot_prg", ROMREGION_ERASE00 ) /* i186 code */
	ROM_LOAD16_BYTE( "bingo v 29.4.99 l.bin", 0x00000, 0x08000, CRC(b6773bff) SHA1(74e375662730e002e05186bd77098fa0d8e43ade) )
	ROM_LOAD16_BYTE( "bingo v 29.4.99 h.bin", 0x00001, 0x08000, CRC(0e18f90a) SHA1(0743302e675f01f8ad42ac2e67ecb1c1bf870ae7) )

	// gfx roms on this one are twice the size of the others
	ROM_REGION( 0x20000, "gfx", 0 ) /* blitter data? */
	ROM_LOAD16_BYTE( "bingo turbo l.bin", 0x000000, 0x10000, CRC(86b10566) SHA1(5f74b250ced3574feffdc40b6ed013ec5a0c2c97) )
	ROM_LOAD16_BYTE( "bingo turbo h.bin", 0x000001, 0x10000, CRC(7e18f9d7) SHA1(519b65d6812a3762e3215f4918c834d5a444b28a) )

	ROM_REGION( 0x20000, "pic", 0 ) /* protection? */
	ROM_LOAD( "pic16c54b.bin", 0x000, 0x200, CRC(21e8a699) SHA1(8a22292fa3669105d52a9d681d5be345fcfe6607) )

	ROM_REGION( 0x20000, "eeprom", 0 ) /* eeprom */
	ROM_LOAD( "bingor1_24c04a.bin", 0x000000, 0x200, CRC(b169df46) SHA1(ebafc81c6918aae9daa6b90df16161751cfd2590) )
ROM_END

ROM_START( bingor2 )
	ROM_REGION( 0x20000, "boot_prg", ROMREGION_ERASE00 ) /* i186 code */
	ROM_LOAD16_BYTE( "bingo roll vip2 v26.02.02_l.bin", 0x00000, 0x10000, CRC(40df6ee8) SHA1(1e90ef906e47f95ebde85b6dd32fdfe50c0564fc) )
	ROM_LOAD16_BYTE( "bingo roll vip2 v26.02.02_h.bin", 0x00001, 0x10000, CRC(9154c183) SHA1(a4060294295a9b8df07ce9fcfeefcf009e129817) )

	ROM_REGION( 0x20000, "gfx", 0 ) /* blitter data? */
	ROM_LOAD16_BYTE( "bingo roll grafik l.bin", 0x000000, 0x10000, CRC(3e753e13) SHA1(011b5f530e54332be194830c0a1d2ec31425017a) )
	ROM_LOAD16_BYTE( "bingo roll grafik h.bin", 0x000001, 0x10000, CRC(4eec39ad) SHA1(4201d5ec207d30dcac9813dd6866d2b61c168e75) )

	ROM_REGION( 0x20000, "pic", 0 ) /* protection? */
	ROM_LOAD( "pic16c54c.bin", 0x000, 0x200, CRC(21e8a699) SHA1(8a22292fa3669105d52a9d681d5be345fcfe6607) )

	ROM_REGION( 0x20000, "eeprom", 0 ) /* eeprom */
	ROM_LOAD( "bingor2_24c04a.bin", 0x000000, 0x200, CRC(a7c87036) SHA1(f7d6161bbfdcdc50212f6b71eb2cbbbb18548cc6) )
ROM_END

ROM_START( bingor3 )
	ROM_REGION( 0x10000, "boot_prg", ROMREGION_ERASE00 ) /* i186 code */
	ROM_LOAD16_BYTE( "bellstar vip2l 27.07_1.bin", 0x00000, 0x08000, CRC(0115bca7) SHA1(0b692b46bc6641296861666f00ec0475dc7296a1) )
	ROM_LOAD16_BYTE( "bellstar vip2l 27.07_2.bin", 0x00001, 0x08000, CRC(c689aa69) SHA1(fb1f477654909f156c30a6be29f84962f4edb1c3) )

	ROM_REGION( 0x20000, "gfx", ROMREGION_ERASE00 ) /* blitter data? */
	ROM_LOAD16_BYTE( "bsg-11.10.02_l.bin", 0x000000, 0x08000, CRC(a8b22477) SHA1(92d638f0f188a43f14487989cf42195311fb2c35) ) //half size?
	ROM_LOAD16_BYTE( "bsg-11.10.02_h.bin", 0x000001, 0x08000, CRC(969d201c) SHA1(7705ceb383ef122538ebf8046041d1c24ec9b9a4) )

	ROM_REGION( 0x20000, "pic", 0 ) /* protection? */
	ROM_LOAD( "pic16c54c.bin", 0x000, 0x400, CRC(5a507be6) SHA1(f4fbfb7e7516eecab32d96b3a34ad88395edac9e) )

	ROM_REGION( 0x20000, "eeprom", 0 ) /* eeprom */
	ROM_LOAD( "bingor3_24c04a.bin", 0x000000, 0x200,  CRC(7a5eb172) SHA1(12d2fc96049427ef1a8acf47242b41b2095d28b6) )
	ROM_LOAD( "bingor3_24c04a_alt.bin", 0x000000, 0x200,  CRC(fcff2d26) SHA1(aec1ddd38149404741a057c74bf84bfb4a8e4aa1) )
ROM_END

// this is a mix of 2 of the other sets.. I don't know if it's correct
ROM_START( bingor4 )
	ROM_REGION( 0x10000, "boot_prg", ROMREGION_ERASE00 ) /* i186 code */
	ROM_LOAD16_BYTE( "01.bin", 0x00000, 0x08000, CRC(0115bca7) SHA1(0b692b46bc6641296861666f00ec0475dc7296a1) )
	ROM_LOAD16_BYTE( "02.bin", 0x00001, 0x08000, CRC(c689aa69) SHA1(fb1f477654909f156c30a6be29f84962f4edb1c3) )

	ROM_REGION( 0x20000, "gfx", 0 ) /* blitter data? */
	ROM_LOAD16_BYTE( "bingo roll grafik l.bin", 0x000000, 0x10000, CRC(3e753e13) SHA1(011b5f530e54332be194830c0a1d2ec31425017a) )
	ROM_LOAD16_BYTE( "bingo roll grafik h.bin", 0x000001, 0x10000, CRC(4eec39ad) SHA1(4201d5ec207d30dcac9813dd6866d2b61c168e75) )

	ROM_REGION( 0x20000, "pic", 0 ) /* protection? */
	ROM_LOAD( "pic16c54c.bin", 0x000, 0x200, CRC(21e8a699) SHA1(8a22292fa3669105d52a9d681d5be345fcfe6607) )

	ROM_REGION( 0x20000, "eeprom", 0 ) /* eeprom */
	ROM_LOAD( "bingor4_24c04a.bin", 0x000000, 0x200,  CRC(38cf70a9) SHA1(ba9a1640200963e2d58d761edc13a24fa5ef44c2) )
ROM_END

/*

  BellStar V3
  Same as bingor3, but with different program ROMs

*/

ROM_START( bingor5 )    /* BellStar V3 */
	ROM_REGION( 0x10000, "boot_prg", ROMREGION_ERASE00 ) /* i186 code */
	ROM_LOAD16_BYTE( "bellstar_v3_v.3.6.02_l.bin", 0x00000, 0x08000, CRC(56b84a5d) SHA1(1bda4fb972b4f0f0575089b545bf15dfea859948) )
	ROM_LOAD16_BYTE( "bellstar_v3_v.3.6.02_h.bin", 0x00001, 0x08000, CRC(d6945bb8) SHA1(b620f1b547be03c4609bff8d06111d0ea425bae8) )

	ROM_REGION( 0x20000, "gfx", ROMREGION_ERASE00 ) /* blitter data? */
	ROM_LOAD16_BYTE( "b.s.grafik_11.10.02_l.bin", 0x000000, 0x08000, CRC(a8b22477) SHA1(92d638f0f188a43f14487989cf42195311fb2c35) ) //half size?
	ROM_LOAD16_BYTE( "b.s.grafik_11.10.02_h.bin", 0x000001, 0x08000, CRC(969d201c) SHA1(7705ceb383ef122538ebf8046041d1c24ec9b9a4) )

	ROM_REGION( 0x20000, "pic", 0 ) /* protection? */
	ROM_LOAD( "pic16c54c.bin", 0x000, 0x400, CRC(5a507be6) SHA1(f4fbfb7e7516eecab32d96b3a34ad88395edac9e) )

	ROM_REGION( 0x20000, "eeprom", 0 ) /* eeprom */
	ROM_LOAD( "bingor5_24c04a.bin", 0x000000, 0x200,  CRC(fcff2d26) SHA1(aec1ddd38149404741a057c74bf84bfb4a8e4aa1) )
ROM_END


ROM_START( vip2000 )
	ROM_REGION( 0x10000, "boot_prg", ROMREGION_ERASE00 )
	ROM_LOAD16_BYTE( "VIPBIOS8L.bin", 0x00000, 0x08000, CRC(a4c2b351) SHA1(bb718584bfe32b9ed27fadfd89b4094d4bbd6d7f) )
	ROM_LOAD16_BYTE( "VIPBIOS8H.bin", 0x00001, 0x08000, CRC(7c42c5ee) SHA1(c419a834ddb245363bacfe70d31cff7c2d4a2d03) )

	ROM_REGION16_BE( 0x80000, "flash", ROMREGION_ERASE00 )
	ROM_LOAD( "AT49F4096.bin", 0x00000, 0x80000, CRC(1d0fd3cf) SHA1(0ad76ea7efa31049a73cc336130cb5ca15480edd) )

	ROM_REGION( 0x08000, "slavecpu", 0 )
	ROM_LOAD( "SLAVE190991.bin", 0x0000, 0x8000, CRC(67feb297) SHA1(442b62e62b614bda2d277e4b827cb89677d6fbce) )

	ROM_REGION( 0x00800, "pic1", 0 )
	ROM_LOAD( "PIC16F627_A.bin", 0x000, 0x800, NO_DUMP )

	ROM_REGION( 0x00800, "pic2", 0 )
	ROM_LOAD( "PIC16F627_B.bin", 0x000, 0x800, NO_DUMP )

	ROM_REGION( 0x20000, "eeprom", 0 )
	ROM_LOAD( "24C04A.bin", 0x000, 0x200, CRC(4e231420) SHA1(24dcfc90ef9903692030be7de0f04fc9370021fd) )
ROM_END



GAME( 2002, bingor1,    0,      bingor,   bingor, bingor_state,   0,       ROT0,  "<unknown>",          "Bingo Roll / Bell Star? (set 1)",     MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
GAME( 2002, bingor2,    0,      bingor2,  bingor, bingor_state,   0,       ROT0,  "<unknown>",          "Bingo Roll / Bell Star? (set 2)",     MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
GAME( 2002, bingor3,    0,      bingor,   bingor, bingor_state,   0,       ROT0,  "<unknown>",          "Bingo Roll / Bell Star? (set 3)",     MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
GAME( 2002, bingor4,    0,      bingor,   bingor, bingor_state,   0,       ROT0,  "<unknown>",          "Bingo Roll / Bell Star? (set 4)",     MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
GAME( 2002, bingor5,    0,      bingor,   bingor, bingor_state,   0,       ROT0,  "<unknown>",          "Bingo Roll / Bell Star V3? (set 5)",  MACHINE_NOT_WORKING | MACHINE_NO_SOUND )

GAME( 2001, vip2000,    0,      vip2000,  bingor, bingor_state,   0,       ROT0,  "Paloma-Elektronik?", "Unknown 'VIP 2000' game",             MACHINE_NOT_WORKING | MACHINE_NO_SOUND )
