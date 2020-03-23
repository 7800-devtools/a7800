// license:BSD-3-Clause
// copyright-holders:Couriersud
/*
 * Couriersud, July 2014:
 *
 * This documents recent work on the AY8910. A YM2149 is now on it's way from
 * Hong Kong as well.
 *
 * TODO:
 *
 * - Create a true sound device nAY8910 driver.
 * - implement approach outlined below in this driver.
 *
 * For years I had a AY8910 in my drawer. Arduinos were around as well.
 * Using the approach documented in this blog post
 *    http://www.986-studio.com/2014/05/18/another-ay-entry/#more-476
 * I measured the output voltages using a Extech 520.
 *
 * Measurement Setup
 *
 * Laptop <--> Arduino <---> AY8910
 *
 * AY8910 Registers:
 * 0x07: 3f
 * 0x08: RV
 * 0x09: RV
 * 0x0A: RV
 *
 * Output was measured on Analog Output B with a resistor RD to
 * ground.
 *
 * Measurement results:
 *
 * RD      983  9.830k   99.5k  1.001M    open
 *
 * RV        B       B       B       B       B
 *  0   0.0000  0.0000  0.0001  0.0011  0.0616
 *  1   0.0106  0.0998  0.6680  1.8150  2.7260
 *  2   0.0150  0.1377  0.8320  1.9890  2.8120
 *  3   0.0222  0.1960  1.0260  2.1740  2.9000
 *  4   0.0320  0.2708  1.2320  2.3360  2.9760
 *  5   0.0466  0.3719  1.4530  2.4880  3.0440
 *  6   0.0665  0.4938  1.6680  2.6280  3.1130
 *  7   0.1039  0.6910  1.9500  2.7900  3.1860
 *  8   0.1237  0.7790  2.0500  2.8590  3.2340
 *  9   0.1986  1.0660  2.3320  3.0090  3.3090
 * 10   0.2803  1.3010  2.5050  3.0850  3.3380
 * 11   0.3548  1.4740  2.6170  3.1340  3.3590
 * 12   0.4702  1.6870  2.7340  3.1800  3.3730
 * 13   0.6030  1.8870  2.8410  3.2300  3.4050
 * 14   0.7530  2.0740  2.9280  3.2580  3.4170
 * 15   0.9250  2.2510  3.0040  3.2940  3.4380
 *
 * Using an equivalent model approach with two resistors
 *
 *      5V
 *       |
 *       Z
 *       Z Resistor Value for RV
 *       Z
 *       |
 *       +---> Output signal
 *       |
 *       Z
 *       Z External RD
 *       Z
 *       |
 *      GND
 *
 * will NOT work out of the box since RV = RV(RD).
 *
 * The following approach will be used going forward based on die pictures
 * of the AY8910 done by Dr. Stack van Hay:
 *
 *
 *              5V
 *             _| D
 *          G |      NMOS
 *     Vg ---||               Kn depends on volume selected
 *            |_  S Vs
 *               |
 *               |
 *               +---> VO Output signal
 *               |
 *               Z
 *               Z External RD
 *               Z
 *               |
 *              GND
 *
 *  Whilst conducting, the FET operates in saturation mode:
 *
 *  Id = Kn * (Vgs - Vth)^2
 *
 *  Using Id = Vs / RD
 *
 *  Vs = Kn * RD  * (Vg - Vs - Vth)^2
 *
 *  finally using Vg' = Vg - Vth
 *
 *  Vs = Vg' + 1 / (2 * Kn * RD) - sqrt((Vg' + 1 / (2 * Kn * RD))^2 - Vg'^2)
 *
 *  and finally
 *
 *  VO = Vs
 *
 *  and this can be used to re-Thenevin to 5V
 *
 *  RVequiv = RD * ( 5V / VO - 1)
 *
 *  The RV and Kn parameter are derived using least squares to match
 *  calculation results with measurements.
 *
 *  FIXME:
 *  There is voltage of 60 mV measured with the EX520 (Ri ~ 10M). This may
 *  be induced by cutoff currents from the 15 FETs.
 *
 */


/***************************************************************************

  ay8910.c

  Emulation of the AY-3-8910 / YM2149 sound chip.

  Based on various code snippets by Ville Hallik, Michael Cuddy,
  Tatsuyuki Satoh, Fabrice Frances, Nicola Salmoria.

  Mostly rewritten by couriersud in 2008

  Public documentation:

  - http://privatfrickler.de/blick-auf-den-chip-soundchip-general-instruments-ay-3-8910/
    Die pictures of the AY8910

  - US Patent 4933980

  Games using ADSR: gyruss

  A list with more games using ADSR can be found here:
        http://mametesters.org/view.php?id=3043

  TODO:
  * The AY8930 has an extended mode which is currently
    not emulated.
  * YM2610 & YM2608 will need a separate flag in their config structures
    to distinguish between legacy and discrete mode.

  The rewrite also introduces a generic model for the DAC. This model is
  not perfect, but allows channel mixing based on a parametrized approach.
  This model also allows to factor in different loads on individual channels.
  If a better model is developped in the future or better measurements are
  available, the driver should be easy to change. The model is described
  later.

  In order to not break hundreds of existing drivers by default the flag
  AY8910_LEGACY_OUTPUT is used by drivers not changed to take into account the
  new model. All outputs are normalized to the old output range (i.e. 0 .. 7ffff).
  In the case of channel mixing, output range is 0...3 * 7fff.

  The main difference between the AY-3-8910 and the YM2149 is, that the
  AY-3-8910 datasheet mentions, that fixed volume level 0, which is set by
  registers 8 to 10 is "channel off". The YM2149 mentions, that the generated
  signal has a 2V DC component. This is confirmed by measurements. The approach
  taken here is to assume the 2V DC offset for all outputs for the YM2149.
  For the AY-3-8910, an offset is used if envelope is active for a channel.
  This is backed by oscilloscope pictures from the datasheet. If a fixed volume
  is set, i.e. envelope is disabled, the output voltage is set to 0V. Recordings
  I found on the web for gyruss indicate, that the AY-3-8910 offset should
  be around 0.2V. This will also make sound levels more compatible with
  user observations for scramble.

  The Model:
                     5V     5V
                      |      |
                      /      |
  Volume Level x >---|       Z
                      >      Z Pullup Resistor RU
                       |     Z
                       Z     |
                    Rx Z     |
                       Z     |
                       |     |
                       '-----+-------->  >---+----> Output signal
                             |               |
                             Z               Z
               Pulldown RD   Z               Z Load RL
                             Z               Z
                             |               |
                            GND             GND

Each Volume level x will select a different resistor Rx. Measurements from fpgaarcade.com
where used to calibrate channel mixing for the YM2149. This was done using
a least square approach using a fixed RL of 1K Ohm.

For the AY measurements cited in e.g. openmsx as "Hacker Kay" for a single
channel were taken. These were normalized to 0 ... 65535 and consequently
adapted to an offset of 0.2V and a VPP of 1.3V. These measurements are in
line e.g. with the formula used by pcmenc for the volume: vol(i) = exp(i/2-7.5).

The following is documentation from the code moved here and amended to reflect
the changes done:

Careful studies of the chip output prove that the chip counts up from 0
until the counter becomes greater or equal to the period. This is an
important difference when the program is rapidly changing the period to
modulate the sound. This is worthwhile noting, since the datasheets
say, that the chip counts down.
Also, note that period = 0 is the same as period = 1. This is mentioned
in the YM2203 data sheets. However, this does NOT apply to the Envelope
period. In that case, period = 0 is half as period = 1.

Envelope shapes:
    C AtAlH
    0 0 x x  \___
    0 1 x x  /___
    1 0 0 0  \\\\
    1 0 0 1  \___
    1 0 1 0  \/\/
    1 0 1 1  \```
    1 1 0 0  ////
    1 1 0 1  /```
    1 1 1 0  /\/\
    1 1 1 1  /___

The envelope counter on the AY-3-8910 has 16 steps. On the YM2149 it
has twice the steps, happening twice as fast.

****************************************************************************

    The bus control and chip selection signals of the AY PSGs and their
    pin-compatible clones such as YM2149 are somewhat unconventional and
    redundant, having been designed for compatibility with GI's CP1610
    series of microprocessors. Much of the redundancy can be finessed by
    tying BC2 to Vcc; AY-3-8913 and AY8930 do this internally.

                            /A9   A8    /CS   BDIR  BC2   BC1
                AY-3-8910   24    25    n/a   27    28    29
                AY-3-8912   n/a   17    n/a   18    19    20
                AY-3-8913   22    23    24    2     n/a   3
                            ------------------------------------
                Inactive            NACT      0     0     0
                Latch address       ADAR      0     0     1
                Inactive            IAB       0     1     0
                Read from PSG       DTB       0     1     1
                Latch address       BAR       1     0     0
                Inactive            DW        1     0     1
                Write to PSG        DWS       1     1     0
                Latch address       INTAK     1     1     1

***************************************************************************/

/**
AY-3-8910(A)/8914/8916/8917/8930/YM2149 (others?):
                        _______    _______
                      _|       \__/       |_
   [4] VSS (GND) --  |_|1  *            40|_|  -- VCC (+5v)
                      _|                  |_
             [5] NC  |_|2               39|_|  <- TEST 1 [1]
                      _|                  |_
ANALOG CHANNEL B <-  |_|3               38|_|  -> ANALOG CHANNEL C
                      _|                  |_
ANALOG CHANNEL A <-  |_|4               37|_|  <> DA0
                      _|                  |_
             [5] NC  |_|5               36|_|  <> DA1
                      _|                  |_
            IOB7 <>  |_|6               35|_|  <> DA2
                      _|                  |_
            IOB6 <>  |_|7               34|_|  <> DA3
                      _|   /---\          |_
            IOB5 <>  |_|8  \-/ |   A    33|_|  <> DA4
                      _|   .   .   Y      |_
            IOB4 <>  |_|9  |---|   - S  32|_|  <> DA5
                      _|   '   '   3 O    |_
            IOB3 <>  |_|10   8     - U  31|_|  <> DA6
                      _|     3     8 N    |_
            IOB2 <>  |_|11   0     9 D  30|_|  <> DA7
                      _|     8     1      |_
            IOB1 <>  |_|12         0    29|_|  <- BC1
                      _|     P            |_
            IOB0 <>  |_|13              28|_|  <- BC2
                      _|                  |_
            IOA7 <>  |_|14              27|_|  <- BDIR
                      _|                  |_                     Prelim. DS:   YM2149/8930:
            IOA6 <>  |_|15              26|_|  <- TEST 2 [2,3]   CS2           /SEL
                      _|                  |_
            IOA5 <>  |_|16              25|_|  <- A8 [3]         CS1
                      _|                  |_
            IOA4 <>  |_|17              24|_|  <- /A9 [3]        /CS0
                      _|                  |_
            IOA3 <>  |_|18              23|_|  <- /RESET
                      _|                  |_
            IOA2 <>  |_|19              22|_|  == CLOCK
                      _|                  |_
            IOA1 <>  |_|20              21|_|  <> IOA0
                       |__________________|

[1] Based on the decap, TEST 1 connects to the Envelope Generator and/or the
    frequency divider somehow. Is this an input or an output?
[2] The TEST 2 input connects to the same selector as A8 and /A9 do on the 8910
    and acts as just another active high enable like A8(pin 25).
    The preliminary datasheet calls this pin CS2.
    On the 8914, it performs the same above function but additionally ?disables?
    the DA0-7 bus if pulled low/active. This additional function was removed
    on the 8910.
    This pin has an internal pullup.
    On the AY8930 and YM2149, this pin is /SEL; if low, clock input is halved.
[3] These 3 pins are technically enables, and have pullups/pulldowns such that
    if the pins are left floating, the chip remains enabled.
[4] On the AY-3-8910 the bond wire for the VSS pin goes to the substrate frame,
    and then a separate bond wire connects it to a pad between pins 21 and 22.
[5] These pins lack internal bond wires entirely.


AY-3-8912(A):
                        _______    _______
                      _|       \__/       |_
ANALOG CHANNEL C <-  |_|1  *            28|_|  <> DA0
                      _|                  |_
          TEST 1 ->  |_|2               27|_|  <> DA1
                      _|                  |_
       VCC (+5V) --  |_|3               26|_|  <> DA2
                      _|                  |_
ANALOG CHANNEL B <-  |_|4               25|_|  <> DA3
                      _|    /---\         |_
ANALOG CHANNEL A <-  |_|5   \-/ |   A   24|_|  <> DA4
                      _|    .   .   Y     |_
       VSS (GND) --  |_|6   |---|   - S 23|_|  <> DA5
                      _|    '   '   3 O   |_
            IOA7 <>  |_|7    T 8    - U 22|_|  <> DA6
                      _|     A 3    8 N   |_
            IOA6 <>  |_|8    I 1    9 D 21|_|  <> DA7
                      _|     W 1    1     |_
            IOA5 <>  |_|9    A  C   2   20|_|  <- BC1
                      _|     N  D         |_
            IOA4 <>  |_|10      A       19|_|  <- BC2
                      _|                  |_
            IOA3 <>  |_|11              18|_|  <- BDIR
                      _|                  |_
            IOA2 <>  |_|12              17|_|  <- A8
                      _|                  |_
            IOA1 <>  |_|13              16|_|  <- /RESET
                      _|                  |_
            IOA0 <>  |_|14              15|_|  == CLOCK
                       |__________________|


AY-3-8913:
                        _______    _______
                      _|       \__/       |_
   [1] VSS (GND) --  |_|1  *            24|_|  <- /CHIP SELECT [2]
                      _|                  |_
            BDIR ->  |_|2               23|_|  <- A8
                      _|                  |_
             BC1 ->  |_|3               22|_|  <- /A9
                      _|    /---\         |_
             DA7 <>  |_|4   \-/ |   A   21|_|  <- /RESET
                      _|    .   .   Y     |_
             DA6 <>  |_|5   |---|   -   20|_|  == CLOCK
                      _|    '   '   3     |_
             DA5 <>  |_|6    T 8    -   19|_|  -- VSS (GND) [1]
                      _|     A 3    8     |_
             DA4 <>  |_|7    I 3    9   18|_|  -> ANALOG CHANNEL C
                      _|     W 2    1     |_
             DA3 <>  |_|8    A      3   17|_|  -> ANALOG CHANNEL A
                      _|     N C          |_
             DA2 <>  |_|9      -        16|_|  NC(?)
                      _|       A          |_
             DA1 <>  |_|10              15|_|  -> ANALOG CHANNEL B
                      _|                  |_
             DA0 <>  |_|11              14|_|  ?? TEST IN [3]
                      _|                  |_
    [4] TEST OUT ??  |_|12              13|_|  -- VCC (+5V)
                       |__________________|

[1] Both of these are ground, they are probably connected together internally. Grounding either one should work.
[2] This is effectively another enable, much like TEST 2 is on the AY-3-8910 and 8914, but active low
[3] This is claimed to be equivalent to TEST 1 on the datasheet
[4] This is claimed to be equivalent to TEST 2 on the datasheet


GI AY-3-8910/A Programmable Sound Generator (PSG): 2 I/O ports
  A7 thru A4 enable state for selecting a register can be changed with a
    factory mask adjustment but default was 0000 for the "common" part shipped
    (probably die "-100").
  Pins 24, 25, and 26 are /A9, A8, and TEST2, which are an active low, high
    and high chip enable, respectively.
  AY-3-8910:  Unused bits in registers have unknown behavior.
  AY-3-8910A: Unused bits in registers have unknown behavior.
  I/O current source/sink behavior is unknown.
  AY-3-8910 die is labeled "90-32033" with a 1979 copyright and a "-100" die
    code.
  AY-3-8910A die is labeled "90-32128" with a 1983 copyright.
GI AY-3-8912/A: 1 I/O port
  /A9 pin doesn't exist and is considered pulled low.
  TEST2 pin doesn't exist and is considered pulled high.
  IOB pins do not exist and have unknown behavior if driven high/low and read
    back.
  A7 thru A4 enable state for selecting a register can be changed with a
    factory mask adjustment but default was 0000 for the "common" part shipped
  AY-3-8912:  Unused bits in registers have unknown behavior.
  AY-3-8912A: Unused bits in registers have unknown behavior.
  I/O current source/sink behavior is unknown.
  AY-3-8912 die is unknown.
  AY-3-8912A or A/P die is unknown.
AY-3-8913: 0 I/O ports
  BC2 pin doesn't exist and is considered pulled high.
  IOA/B pins do not exist and have unknown behavior if driven high/low and read back.
  A7 thru A4 enable state for selecting a register can be changed with a
    factory mask adjustment but default was 0000 for the "common" part shipped
  AY-3-8913:  Unused bits in registers have unknown behavior.
  AY-3-8913 die is unknown.
GI AY-3-8914/A: 2 I/O ports
  A7 thru A4 enable state for selecting a register can be changed with a
    factory mask adjustment but was 0000 for the part shipped with the
    Intellivision.
  Pins 24, 25, and 26 are /A9, A8, and TEST2, which are an active low, high
    and high chip enable, respectively.
  TEST2 additionally ?disables? the data bus if pulled low.
  The register mapping is different from the AY-3-8910, the AY-3-8914 register
    mapping matches the "preliminary" 1978 AY-3-8910 datasheet.
  The Envelope/Volume control register is 6 bits wide instead of 5 bits, and
    the additional bit combines with the M bit to form a bit pair C0 and C1,
    which shift the volume output of the Envelope generator right by 0, 1 or 2
    bits on a given channel, or allow the low 4 bits to drive the channel
    volume.
  AY-3-8914:  Unused bits in registers have unknown behavior.
  AY-3-8914A: Unused bits in registers have unknown behavior.
  I/O current source/sink behavior is unknown.
  AY-3-8914 die is labeled "90-32022" with a 1978 copyright.
  AY-3-8914A die is unknown.
GI AY-3-8916: 2 I/O ports
  A7 thru A4 enable state for selecting a register can be changed with a
    factory mask adjustment; its mask is unknown. This chip was shipped
    with certain later Intellivision II systems.
  Pins 24, 25, and 26 are /A9, /A8(!), and TEST2, which are an active low,
    low(!) and high chip enable, respectively.
    NOTE: the /A8 enable polarity may be mixed up with AY-3-8917 below.
  AY-3-8916: Unused bits in registers have unknown behavior.
  I/O current source/sink behavior is unknown.
  AY-3-8916 die is unknown.
GI AY-3-8917: 2 I/O ports
  A7 thru A4 enable state for selecting a register can be changed with a
    factory mask adjustment but was 1111 for the part shipped with the
    Intellivision ECS module.
  Pins 24, 25, and 26 are /A9, A8, and TEST2, which are an active low, high
    and high chip enable, respectively.
    NOTE: the A8 enable polarity may be mixed up with AY-3-8916 above.
  AY-3-8917: Unused bits in registers have unknown behavior.
  I/O current source/sink behavior is unknown.
  AY-3-8917 die is unknown.
Microchip AY8930 Enhanced Programmable Sound Generator (EPSG): 2 I/O ports
  BC2 pin exists but is always considered pulled high. The pin might have no
    bond wire at all.
  Pins 2 and 5 might be additional test pins rather than being NC.
  A7 thru A4 enable state for selecting a register are 0000 for all? parts
    shipped.
  Pins 24 and 25 are /A9, A8 which are an active low and high chip enable.
  Pin 26 is /SELECT which if driven low divides the input clock by 2.
  Writing 0xAn or 0xBn to register 0x0D turns on extended mode, which enables
    an additional 16 registers (banked using 0x0D bit 0), and clears the
    contents of all of the registers except the high 3 bits of register 0x0D
    (according to the datasheet).
  If the AY8930's extended mode is enabled, it gains higher resolution
    frequency and volume control, separate volume per-channel, and the duty
    cycle can be adjusted for the 3 channels.
  If the mode is not enabled, it behaves almost exactly like an AY-3-8910(A?),
    barring the BC2 and /SELECT differences.
  AY8930: Unused bits in registers have unknown behavior, but the datasheet
    explicitly states that unused bits always read as 0.
  I/O current source/sink behavior is unknown.
  AY8930 die is unknown.
Yamaha YM2149 Software-Controlled Sound Generator (SSG): 2 I/O ports
  A7 thru A4 enable state for selecting a register are 0000 for all? parts
    shipped.
  Pins 24 and 25 are /A9, A8 which are an active low and high chip enable.
  Pin 26 is /SEL which if driven low divides the input clock by 2.
  The YM2149 envelope register has 5 bits of resolution internally, allowing
  for smoother volume ramping, though the register for setting its direct
  value remains 4 bits wide.
  YM2149: Unused bits in registers have unknown behavior.
  I/O current source/sink behavior is unknown.
  YM2149 die is unknown; only one die revision, 'G', has been observed
    from Yamaha chip/datecode silkscreen surface markings.
Yamaha YM2203: 2 I/O ports
  The pinout of this chip is completely different from the AY-3-8910.
  The entire way this chip is accessed is completely different from the usual
    AY-3-8910 selection of chips, so there is a /CS and a /RD and a /WR and
    an A0 pin; The chip status can be read back by reading the register
    select address.
  The chip has a 3-channel, 4-op FM synthesis sound core in it, not discussed
    in this source file.
  The first 16 registers are the same(?) as the YM2149.
  YM2203: Unused bits in registers have unknown behavior.
  I/O current source/sink behavior is unknown.
  YM2203 die is unknown; two die revisions, 'F' and 'H', have been observed
    from Yamaha chip/datecode silkscreen surface markings. It is unknown
    what behavioral differences exist between these two revisions.
    The 'F' revision only appears during the first year of production.
Yamaha YM3439: limited info: CMOS version of YM2149?
Yamaha YMZ284: limited info: 0 I/O port, different clock divider
  The chip selection logic is again simplified here: pin 1 is /WR, pin 2 is
    /CS and pin 3 is A0.
  D0-D7 are conveniently all on one side of the 16-pin package.
  Pin 8 is /IC (initial clear), with an internal pullup.
Yamaha YMZ294: limited info: 0 I/O port
  Pinout is identical to YMZ284 except for two additions: pin 8 selects
    between 4MHz (H) and 6MHz (L), while pin 10 is /TEST.
OKI M5255, Winbond WF19054, JFC 95101, File KC89C72, Toshiba T7766A : differences to be listed

Decaps:
AY-3-8914 - http://siliconpr0n.org/map/gi/ay-3-8914/mz_mit20x/
AY-3-8910 - http://privatfrickler.de/blick-auf-den-chip-soundchip-general-instruments-ay-3-8910/
AY-3-8910A - https://seanriddledecap.blogspot.com/2017/01/gi-ay-3-8910-ay-3-8910a-gi-8705-cba.html (TODO: update this link when it has its own page at seanriddle.com)

Links:
AY-3-8910 'preliminary' datasheet (which actually describes the AY-3-8914) from 1978:
  http://spatula-city.org/~im14u2c/intv/gi_micro_programmable_tv_games/page_7_100.png
  http://spatula-city.org/~im14u2c/intv/gi_micro_programmable_tv_games/page_7_101.png
  http://spatula-city.org/~im14u2c/intv/gi_micro_programmable_tv_games/page_7_102.png
  http://spatula-city.org/~im14u2c/intv/gi_micro_programmable_tv_games/page_7_103.png
  http://spatula-city.org/~im14u2c/intv/gi_micro_programmable_tv_games/page_7_104.png
  http://spatula-city.org/~im14u2c/intv/gi_micro_programmable_tv_games/page_7_105.png
AY-3-8910/8912 Feb 1979 manual: http://dev-docs.atariforge.org/files/GI_AY-3-8910_Feb-1979.pdf
AY-3-8910/8912/8913 post-1983 manual: http://map.grauw.nl/resources/sound/generalinstrument_ay-3-8910.pdf or http://www.ym2149.com/ay8910.pdf
AY-8930 datasheet: http://www.ym2149.com/ay8930.pdf
YM2149 datasheet: http://www.ym2149.com/ym2149.pdf
YM2203 English datasheet: http://www.appleii-box.de/APPLE2/JonasCard/YM2203%20datasheet.pdf
YM2203 Japanese datasheet contents, translated: http://www.larwe.com/technical/chip_ym2203.html
*/

#include "emu.h"
#include "ay8910.h"

/*************************************
 *
 *  Defines
 *
 *************************************/

#define ENABLE_REGISTER_TEST        (0)     /* Enable preprogrammed registers */
#define LOG_IGNORED_WRITES          (0)

#define MAX_OUTPUT 0x7fff

/* register id's */
#define AY_AFINE    (0)
#define AY_ACOARSE  (1)
#define AY_BFINE    (2)
#define AY_BCOARSE  (3)
#define AY_CFINE    (4)
#define AY_CCOARSE  (5)
#define AY_NOISEPER (6)
#define AY_ENABLE   (7)
#define AY_AVOL     (8)
#define AY_BVOL     (9)
#define AY_CVOL     (10)
#define AY_EFINE    (11)
#define AY_ECOARSE  (12)
#define AY_ESHAPE   (13)

#define AY_PORTA    (14)
#define AY_PORTB    (15)

#define NOISE_ENABLEQ(_chan)  ((m_regs[AY_ENABLE] >> (3 + _chan)) & 1)
#define TONE_ENABLEQ(_chan)   ((m_regs[AY_ENABLE] >> (_chan)) & 1)
#define TONE_PERIOD(_chan)    ( m_regs[(_chan) << 1] | ((m_regs[((_chan) << 1) | 1] & 0x0f) << 8) )
#define NOISE_PERIOD()          ( m_regs[AY_NOISEPER] & 0x1f)
#define TONE_VOLUME(_chan)    ( m_regs[AY_AVOL + (_chan)] & 0x0f)
#define TONE_ENVELOPE(_chan)  ((m_regs[AY_AVOL + (_chan)] >> 4) & ((type() == AY8914) ? 3 : 1))
#define ENVELOPE_PERIOD()       ((m_regs[AY_EFINE] | (m_regs[AY_ECOARSE]<<8)))
#define NOISE_OUTPUT()          (m_rng & 1)

/*************************************
 *
 *  Type definitions
 *
 *************************************/


/*************************************
 *
 *  Static
 *
 *************************************/

static const ay8910_device::ay_ym_param ym2149_param =
{
	630, 801,
	16,
	{ 73770, 37586, 27458, 21451, 15864, 12371, 8922,  6796,
		4763,  3521,  2403,  1737,  1123,   762,  438,   251 },
};

static const ay8910_device::ay_ym_param ym2149_param_env =
{
	630, 801,
	32,
	{ 103350, 73770, 52657, 37586, 32125, 27458, 24269, 21451,
		18447, 15864, 14009, 12371, 10506,  8922,  7787,  6796,
		5689,  4763,  4095,  3521,  2909,  2403,  2043,  1737,
		1397,  1123,   925,   762,   578,   438,   332,   251 },
};

#if 0
/* RL = 1000, Hacker Kay normalized, 2.1V to 3.2V */
static const ay8910_device::ay_ym_param ay8910_param =
{
	664, 913,
	16,
	{ 85785, 34227, 26986, 20398, 14886, 10588,  7810,  4856,
		4120,  2512,  1737,  1335,  1005,   747,   586,    451 },
};

/*
 * RL = 3000, Hacker Kay normalized pattern, 1.5V to 2.8V
 * These values correspond with guesses based on Gyruss schematics
 * They work well with scramble as well.
 */
static const ay8910_device::ay_ym_param ay8910_param =
{
	930, 454,
	16,
	{ 85066, 34179, 27027, 20603, 15046, 10724, 7922, 4935,
		4189,  2557,  1772,  1363,  1028,  766,   602,  464 },
};

/*
 * RL = 1000, Hacker Kay normalized pattern, 0.75V to 2.05V
 * These values correspond with guesses based on Gyruss schematics
 * They work well with scramble as well.
 */
static const ay8910_device::ay_ym_param ay8910_param =
{
	1371, 313,
	16,
	{ 93399, 33289, 25808, 19285, 13940, 9846,  7237,  4493,
		3814,  2337,  1629,  1263,   962,  727,   580,   458 },
};

/*
 * RL = 1000, Hacker Kay normalized pattern, 0.2V to 1.5V
 */
static const ay8910_device::ay_ym_param ay8910_param =
{
	5806, 300,
	16,
	{ 118996, 42698, 33105, 24770, 17925, 12678,  9331,  5807,
		4936,  3038,  2129,  1658,  1271,   969,   781,   623 }
};
#endif

/*
 * RL = 2000, Based on Matthew Westcott's measurements from Dec 2001.
 * -------------------------------------------------------------------
 *
 * http://groups.google.com/group/comp.sys.sinclair/browse_thread/thread/fb3091da4c4caf26/d5959a800cda0b5e?lnk=gst&q=Matthew+Westcott#d5959a800cda0b5e
 * After what Russell mentioned a couple of weeks back about the lack of
 * publicised measurements of AY chip volumes - I've finally got round to
 * making these readings, and I'm placing them in the public domain - so
 * anyone's welcome to use them in emulators or anything else.

 * To make the readings, I set up the chip to produce a constant voltage on
 * channel C (setting bits 2 and 5 of register 6), and varied the amplitude
 * (the low 4 bits of register 10). The voltages were measured between the
 * channel C output (pin 1) and ground (pin 6).
 *
 * Level  Voltage
 *  0     1.147
 *  1     1.162
 *  2     1.169
 *  3     1.178
 *  4     1.192
 *  5     1.213
 *  6     1.238
 *  7     1.299
 *  8     1.336
 *  9     1.457
 * 10     1.573
 * 11     1.707
 * 12     1.882
 * 13     2.06
 * 14     2.32
 * 15     2.58
 * -------------------------------------------------------------------
 *
 * The ZX spectrum output circuit was modelled in SwitcherCAD and
 * the resistor values below create the voltage levels above.
 * RD was measured on a real chip to be 8m Ohm, RU was 0.8m Ohm.
 */


static const ay8910_device::ay_ym_param ay8910_param =
{
	800000, 8000000,
	16,
	{ 15950, 15350, 15090, 14760, 14275, 13620, 12890, 11370,
		10600,  8590,  7190,  5985,  4820,  3945,  3017,  2345 }
};

static const ay8910_device::mosfet_param ay8910_mosfet_param =
{
	1.465385778,
	4.9,
	16,
	{
		0.00076,
		0.80536,
		1.13106,
		1.65952,
		2.42261,
		3.60536,
		5.34893,
		8.96871,
		10.97202,
		19.32370,
		29.01935,
		38.82026,
		55.50539,
		78.44395,
		109.49257,
		153.72985,
	}
};




/*************************************
 *
 *  Inline
 *
 *************************************/

static inline void build_3D_table(double rl, const ay8910_device::ay_ym_param *par, const ay8910_device::ay_ym_param *par_env, int normalize, double factor, int zero_is_off, int32_t *tab)
{
	double min = 10.0,  max = 0.0;

	std::vector<double> temp(8*32*32*32, 0);

	for (int e=0; e < 8; e++)
	{
		const ay8910_device::ay_ym_param *par_ch1 = (e & 0x01) ? par_env : par;
		const ay8910_device::ay_ym_param *par_ch2 = (e & 0x02) ? par_env : par;
		const ay8910_device::ay_ym_param *par_ch3 = (e & 0x04) ? par_env : par;

		for (int j1=0; j1 < par_ch1->res_count; j1++)
			for (int j2=0; j2 < par_ch2->res_count; j2++)
				for (int j3=0; j3 < par_ch3->res_count; j3++)
				{
					double n;
					if (zero_is_off)
					{
						n  = (j1 != 0 || (e & 0x01)) ? 1 : 0;
						n += (j2 != 0 || (e & 0x02)) ? 1 : 0;
						n += (j3 != 0 || (e & 0x04)) ? 1 : 0;
					}
					else
						n = 3.0;

					double rt = n / par->r_up + 3.0 / par->r_down + 1.0 / rl;
					double rw = n / par->r_up;

					rw += 1.0 / par_ch1->res[j1];
					rt += 1.0 / par_ch1->res[j1];
					rw += 1.0 / par_ch2->res[j2];
					rt += 1.0 / par_ch2->res[j2];
					rw += 1.0 / par_ch3->res[j3];
					rt += 1.0 / par_ch3->res[j3];

					int indx = (e << 15) | (j3<<10) | (j2<<5) | j1;
					temp[indx] = rw / rt;
					if (temp[indx] < min)
						min = temp[indx];
					if (temp[indx] > max)
						max = temp[indx];
				}
	}

	if (normalize)
	{
		for (int j=0; j < 32*32*32*8; j++)
			tab[j] = MAX_OUTPUT * (((temp[j] - min)/(max-min))) * factor;
	}
	else
	{
		for (int j=0; j < 32*32*32*8; j++)
			tab[j] = MAX_OUTPUT * temp[j];
	}

	/* for (e=0;e<16;e++) printf("%d %d\n",e<<10, tab[e<<10]); */
}

static inline void build_single_table(double rl, const ay8910_device::ay_ym_param *par, int normalize, int32_t *tab, int zero_is_off)
{
	int j;
	double rt;
	double rw;
	double temp[32], min=10.0, max=0.0;

	for (j=0; j < par->res_count; j++)
	{
		rt = 1.0 / par->r_down + 1.0 / rl;

		rw = 1.0 / par->res[j];
		rt += 1.0 / par->res[j];

		if (!(zero_is_off && j == 0))
		{
			rw += 1.0 / par->r_up;
			rt += 1.0 / par->r_up;
		}

		temp[j] = rw / rt;
		if (temp[j] < min)
			min = temp[j];
		if (temp[j] > max)
			max = temp[j];
	}
	if (normalize)
	{
		for (j=0; j < par->res_count; j++)
			tab[j] = MAX_OUTPUT * (((temp[j] - min)/(max-min)) - 0.25) * 0.5;
	}
	else
	{
		for (j=0; j < par->res_count; j++)
			tab[j] = MAX_OUTPUT * temp[j];
	}

}

static inline void build_mosfet_resistor_table(const ay8910_device::mosfet_param &par, const double rd, int32_t *tab)
{
	int j;

	for (j=0; j < par.m_count; j++)
	{
		const double Vd = 5.0;
		const double Vg = par.m_Vg - par.m_Vth;
		const double kn = par.m_Kn[j] / 1.0e6;
		const double p2 = 1.0 / (2.0 * kn * rd) + Vg;
		const double Vs = p2 - sqrt(p2 * p2 - Vg * Vg);

		const double res = rd * ( Vd / Vs - 1.0);
		/* That's the biggest value we can stream on to netlist. */

		if (res > (1 << 28))
			tab[j] = (1 << 28);
		else
			tab[j] = res;
		//printf("%d %f %10d\n", j, rd / (res + rd) * 5.0, tab[j]);
	}
}


uint16_t ay8910_device::mix_3D()
{
	int indx = 0, chan;

	for (chan = 0; chan < NUM_CHANNELS; chan++)
		if (TONE_ENVELOPE(chan) != 0)
		{
			if (type() == AY8914) // AY8914 Has a two bit tone_envelope field
			{
				indx |= (1 << (chan+15)) | ( m_vol_enabled[chan] ? ((m_env_volume >> (3-TONE_ENVELOPE(chan))) << (chan*5)) : 0);
			}
			else
			{
				indx |= (1 << (chan+15)) | ( m_vol_enabled[chan] ? m_env_volume << (chan*5) : 0);
			}
		}
		else
		{
			indx |= (m_vol_enabled[chan] ? TONE_VOLUME(chan) << (chan*5) : 0);
		}
	return m_vol3d_table[indx];
}

/*************************************
 *
 * Static functions
 *
 *************************************/

void ay8910_device::ay8910_write_reg(int r, int v)
{
	//if (r >= 11 && r <= 13 ) printf("%d %x %02x\n", PSG->index, r, v);
	m_regs[r] = v;

	switch( r )
	{
		case AY_AFINE:
		case AY_ACOARSE:
		case AY_BFINE:
		case AY_BCOARSE:
		case AY_CFINE:
		case AY_CCOARSE:
		case AY_NOISEPER:
		case AY_AVOL:
		case AY_BVOL:
		case AY_CVOL:
		case AY_EFINE:
			/* No action required */
			break;
		case AY_ECOARSE:
			if ( (v & 0x0f) > 0)
				osd_printf_verbose("ECoarse\n");
			/* No action required */
			break;
		case AY_ENABLE:
			if ((m_last_enable == -1) ||
				((m_last_enable & 0x40) != (m_regs[AY_ENABLE] & 0x40)))
			{
				/* write out 0xff if port set to input */
				if (!m_port_a_write_cb.isnull())
					m_port_a_write_cb((offs_t)0, (m_regs[AY_ENABLE] & 0x40) ? m_regs[AY_PORTA] : 0xff);
			}

			if ((m_last_enable == -1) ||
				((m_last_enable & 0x80) != (m_regs[AY_ENABLE] & 0x80)))
			{
				/* write out 0xff if port set to input */
				if (!m_port_b_write_cb.isnull())
					m_port_b_write_cb((offs_t)0, (m_regs[AY_ENABLE] & 0x80) ? m_regs[AY_PORTB] : 0xff);
			}
			m_last_enable = m_regs[AY_ENABLE];
			break;
		case AY_ESHAPE:
			if ( (v & 0x0f) > 0)
				osd_printf_verbose("EShape\n");
			m_attack = (m_regs[AY_ESHAPE] & 0x04) ? m_env_step_mask : 0x00;
			if ((m_regs[AY_ESHAPE] & 0x08) == 0)
			{
				/* if Continue = 0, map the shape to the equivalent one which has Continue = 1 */
				m_hold = 1;
				m_alternate = m_attack;
			}
			else
			{
				m_hold = m_regs[AY_ESHAPE] & 0x01;
				m_alternate = m_regs[AY_ESHAPE] & 0x02;
			}
			m_env_step = m_env_step_mask;
			m_holding = 0;
			m_env_volume = (m_env_step ^ m_attack);
			break;
		case AY_PORTA:
			if (m_regs[AY_ENABLE] & 0x40)
			{
				if (!m_port_a_write_cb.isnull())
					m_port_a_write_cb((offs_t)0, m_regs[AY_PORTA]);
				else
					logerror("warning: unmapped write %02x to %s Port A\n", v, name());
			}
			else
			{
#if LOG_IGNORED_WRITES
				logerror("warning: write %02x to %s Port A set as input - ignored\n", v, name());
#endif
			}
			break;
		case AY_PORTB:
			if (m_regs[AY_ENABLE] & 0x80)
			{
				if (!m_port_b_write_cb.isnull())
					m_port_b_write_cb((offs_t)0, m_regs[AY_PORTB]);
				else
					logerror("warning: unmapped write %02x to %s Port B\n", v, name());
			}
			else
			{
#if LOG_IGNORED_WRITES
				logerror("warning: write %02x to %s Port B set as input - ignored\n", v, name());
#endif
			}
			break;
	}
}

//-------------------------------------------------
//  sound_stream_update - handle a stream update
//-------------------------------------------------

void ay8910_device::sound_stream_update(sound_stream &stream, stream_sample_t **inputs, stream_sample_t **outputs, int samples)
{
	stream_sample_t *buf[NUM_CHANNELS];
	int chan;

	buf[0] = outputs[0];
	buf[1] = nullptr;
	buf[2] = nullptr;
	if (m_streams == NUM_CHANNELS)
	{
		buf[1] = outputs[1];
		buf[2] = outputs[2];
	}

	/* hack to prevent us from hanging when starting filtered outputs */
	if (!m_ready)
	{
		for (chan = 0; chan < NUM_CHANNELS; chan++)
			if (buf[chan] != nullptr)
				memset(buf[chan], 0, samples * sizeof(*buf[chan]));
	}

	/* The 8910 has three outputs, each output is the mix of one of the three */
	/* tone generators and of the (single) noise generator. The two are mixed */
	/* BEFORE going into the DAC. The formula to mix each channel is: */
	/* (ToneOn | ToneDisable) & (NoiseOn | NoiseDisable). */
	/* Note that this means that if both tone and noise are disabled, the output */
	/* is 1, not 0, and can be modulated changing the volume. */

	/* buffering loop */
	while (samples)
	{
		for (chan = 0; chan < NUM_CHANNELS; chan++)
		{
			m_count[chan]++;
			if (m_count[chan] >= TONE_PERIOD(chan))
			{
				m_output[chan] ^= 1;
				m_count[chan] = 0;
			}
		}

		m_count_noise++;
		if (m_count_noise >= NOISE_PERIOD())
		{
			/* toggle the prescaler output. Noise is no different to
			 * channels.
			 */
			m_count_noise = 0;
			m_prescale_noise ^= 1;

			if ( m_prescale_noise)
			{
				/* The Random Number Generator of the 8910 is a 17-bit shift */
				/* register. The input to the shift register is bit0 XOR bit3 */
				/* (bit0 is the output). This was verified on AY-3-8910 and YM2149 chips. */

				m_rng ^= (((m_rng & 1) ^ ((m_rng >> 3) & 1)) << 17);
				m_rng >>= 1;
			}
		}

		for (chan = 0; chan < NUM_CHANNELS; chan++)
		{
			m_vol_enabled[chan] = (m_output[chan] | TONE_ENABLEQ(chan)) & (NOISE_OUTPUT() | NOISE_ENABLEQ(chan));
		}

		/* update envelope */
		if (m_holding == 0)
		{
			m_count_env++;
			if (m_count_env >= ENVELOPE_PERIOD() * m_step )
			{
				m_count_env = 0;
				m_env_step--;

				/* check envelope current position */
				if (m_env_step < 0)
				{
					if (m_hold)
					{
						if (m_alternate)
							m_attack ^= m_env_step_mask;
						m_holding = 1;
						m_env_step = 0;
					}
					else
					{
						/* if CountEnv has looped an odd number of times (usually 1), */
						/* invert the output. */
						if (m_alternate && (m_env_step & (m_env_step_mask + 1)))
							m_attack ^= m_env_step_mask;

						m_env_step &= m_env_step_mask;
					}
				}

			}
		}
		m_env_volume = (m_env_step ^ m_attack);

		if (m_streams == 3)
		{
			for (chan = 0; chan < NUM_CHANNELS; chan++)
				if (TONE_ENVELOPE(chan) != 0)
				{
					if (type() == AY8914) // AY8914 Has a two bit tone_envelope field
					{
						*(buf[chan]++) = m_env_table[chan][m_vol_enabled[chan] ? m_env_volume >> (3-TONE_ENVELOPE(chan)) : 0];
					}
					else
					{
						*(buf[chan]++) = m_env_table[chan][m_vol_enabled[chan] ? m_env_volume : 0];
					}
				}
				else
				{
					*(buf[chan]++) = m_vol_table[chan][m_vol_enabled[chan] ? TONE_VOLUME(chan) : 0];
				}
		}
		else
		{
			*(buf[0]++) = mix_3D();
		}
		samples--;
	}
}

void ay8910_device::build_mixer_table()
{
	int normalize = 0;
	int chan;

	if ((m_flags & AY8910_LEGACY_OUTPUT) != 0)
	{
		logerror("%s using legacy output levels!\n", name());
		normalize = 1;
	}

	if ((m_flags & AY8910_RESISTOR_OUTPUT) != 0)
	{
		if (m_type != PSG_TYPE_AY)
			fatalerror("AY8910_RESISTOR_OUTPUT currently only supported for AY8910 devices.");

		for (chan=0; chan < NUM_CHANNELS; chan++)
		{
			build_mosfet_resistor_table(ay8910_mosfet_param, m_res_load[chan], m_vol_table[chan]);
			build_mosfet_resistor_table(ay8910_mosfet_param, m_res_load[chan], m_env_table[chan]);
		}
	}
	else if (m_streams == NUM_CHANNELS)
	{
		for (chan=0; chan < NUM_CHANNELS; chan++)
		{
			build_single_table(m_res_load[chan], m_par, normalize, m_vol_table[chan], m_zero_is_off);
			build_single_table(m_res_load[chan], m_par_env, normalize, m_env_table[chan], 0);
		}
	}
	/*
	 * The previous implementation added all three channels up instead of averaging them.
	 * The factor of 3 will force the same levels if normalizing is used.
	 */
	else
	{
		build_3D_table(m_res_load[0], m_par, m_par_env, normalize, 3, m_zero_is_off, m_vol3d_table.get());
	}
}

void ay8910_device::ay8910_statesave()
{
	save_item(NAME(m_active));
	save_item(NAME(m_register_latch));
	save_item(NAME(m_regs));
	save_item(NAME(m_last_enable));

	save_item(NAME(m_count));
	save_item(NAME(m_count_noise));
	save_item(NAME(m_count_env));

	save_item(NAME(m_env_volume));

	save_item(NAME(m_output));
	save_item(NAME(m_prescale_noise));

	save_item(NAME(m_env_step));
	save_item(NAME(m_hold));
	save_item(NAME(m_alternate));
	save_item(NAME(m_attack));
	save_item(NAME(m_holding));
	save_item(NAME(m_rng));
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void ay8910_device::device_start()
{
	int master_clock = clock();

	if (m_ioports < 1 && !(m_port_a_read_cb.isnull() && m_port_a_write_cb.isnull()))
		fatalerror("Device '%s' is a %s and has no port A!", tag(), name());

	if (m_ioports < 2 && !(m_port_b_read_cb.isnull() && m_port_b_write_cb.isnull()))
		fatalerror("Device '%s' is a %s and has no port B!", tag(), name());

	m_port_a_read_cb.resolve();
	m_port_b_read_cb.resolve();
	m_port_a_write_cb.resolve();
	m_port_b_write_cb.resolve();

	if ((m_flags & AY8910_SINGLE_OUTPUT) != 0)
	{
		logerror("%s device using single output!\n", name());
		m_streams = 1;
	}

	m_vol3d_table = make_unique_clear<int32_t[]>(8*32*32*32);

	build_mixer_table();

	/* The envelope is pacing twice as fast for the YM2149 as for the AY-3-8910,    */
	/* This handled by the step parameter. Consequently we use a divider of 8 here. */
	m_channel = machine().sound().stream_alloc(*this, 0, m_streams, master_clock / 8);

	ay_set_clock(master_clock);
	ay8910_statesave();
}



void ay8910_device::ay8910_reset_ym()
{
	int i;

	m_active = false;
	m_register_latch = 0;
	m_rng = 1;
	m_output[0] = 0;
	m_output[1] = 0;
	m_output[2] = 0;
	m_count[0] = 0;
	m_count[1] = 0;
	m_count[2] = 0;
	m_count_noise = 0;
	m_count_env = 0;
	m_prescale_noise = 0;
	m_last_enable = -1;  /* force a write */
	for (i = 0;i < AY_PORTA;i++)
		ay8910_write_reg(i,0);
	m_ready = 1;
#if ENABLE_REGISTER_TEST
	ay8910_write_reg(AY_AFINE, 0);
	ay8910_write_reg(AY_ACOARSE, 1);
	ay8910_write_reg(AY_BFINE, 0);
	ay8910_write_reg(AY_BCOARSE, 2);
	ay8910_write_reg(AY_CFINE, 0);
	ay8910_write_reg(AY_CCOARSE, 4);
	//#define AY_NOISEPER   (6)
	ay8910_write_reg(AY_ENABLE, ~7);
	ay8910_write_reg(AY_AVOL, 10);
	ay8910_write_reg(AY_BVOL, 10);
	ay8910_write_reg(AY_CVOL, 10);
	//#define AY_EFINE  (11)
	//#define AY_ECOARSE    (12)
	//#define AY_ESHAPE (13)
#endif
}

void ay8910_device::set_volume(int channel,int volume)
{
	int ch;

	for (ch = 0; ch < m_streams; ch++)
		if (channel == ch || m_streams == 1 || channel == ALL_8910_CHANNELS)
			m_channel->set_output_gain(ch, volume / 100.0);
}

void ay8910_device::ay_set_clock(int clock)
{
	// FIXME: this doesn't belong here, it should be an input pin exposed via devcb
	if (type() == YM2149 && (m_flags & YM2149_PIN26_LOW))
	{
		clock /= 2;
	}
	m_channel->set_sample_rate( clock / 8 );
}

void ay8910_device::device_clock_changed()
{
	ay_set_clock(clock());
}

void ay8910_device::ay8910_write_ym(int addr, uint8_t data)
{
	if (addr & 1)
	{
		if (m_active)
		{
			/* Data port */
			if (m_register_latch == AY_ESHAPE || m_regs[m_register_latch] != data)
			{
				/* update the output buffer before changing the register */
				m_channel->update();
			}

			ay8910_write_reg(m_register_latch, data);
		}
	}
	else
	{
		m_active = (data >> 4) == 0; // mask programmed 4-bit code
		if (m_active)
		{
			/* Register port */
			m_register_latch = data & 0x0f;
		}
		else
		{
			logerror("%s: warning - %s upper address mismatch\n", machine().describe_context(), name());
		}
	}
}

uint8_t ay8910_device::ay8910_read_ym()
{
	device_type chip_type = type();
	int r = m_register_latch;

	if (!m_active) return 0xff; // high impedance

	/* There are no state dependent register in the AY8910! */
	/* m_channel->update(); */

	switch (r)
	{
	case AY_PORTA:
		if ((m_regs[AY_ENABLE] & 0x40) != 0)
			logerror("%s: warning - read from %s Port A set as output\n", machine().describe_context(), name());
		/*
		   even if the port is set as output, we still need to return the external
		   data. Some games, like kidniki, need this to work.

		   FIXME: The io ports are designed as open collector outputs. Bits 7 and 8 of AY_ENABLE
		   only enable (low) or disable (high) the pull up resistors. The YM2149 datasheet
		   specifies those pull up resistors as 60k to 600k (min / max).
		   We do need a callback for those two flags. Kid Niki (Irem m62) is one such
		   case were it makes a difference in comparison to a standard TTL output.
		 */
		if (!m_port_a_read_cb.isnull())
			m_regs[AY_PORTA] = m_port_a_read_cb(0);
		else
			logerror("%s: warning - read 8910 Port A\n", machine().describe_context());
		break;
	case AY_PORTB:
		if ((m_regs[AY_ENABLE] & 0x80) != 0)
			logerror("%s: warning - read from 8910 Port B set as output\n", machine().describe_context());
		if (!m_port_b_read_cb.isnull())
			m_regs[AY_PORTB] = m_port_b_read_cb(0);
		else
			logerror("%s: warning - read 8910 Port B\n", machine().describe_context());
		break;
	}

	/* Depending on chip type, unused bits in registers may or may not be accessible.
	Untested chips are assumed to regard them as 'ram'
	Tested and confirmed on hardware:
	- AY-3-8910: inaccessible bits (see masks below) read back as 0
	- AY-3-8914: same as 8910 except regs B,C,D (8,9,A below due to 8910->8914 remapping) are 0x3f
	- AY-3-8916/8917 (used on ECS INTV expansion): inaccessible bits mirror one of the i/o ports, needs further testing
	- YM2149: no anomaly
	*/
	if (chip_type == AY8910) {
		const uint8_t mask[0x10]={
			0xff,0x0f,0xff,0x0f,0xff,0x0f,0x1f,0xff,0x1f,0x1f,0x1f,0xff,0xff,0x0f,0xff,0xff
		};
		return m_regs[r] & mask[r];
	}
	else if (chip_type == AY8914) {
		const uint8_t mask[0x10]={
			0xff,0x0f,0xff,0x0f,0xff,0x0f,0x1f,0xff,0x3f,0x3f,0x3f,0xff,0xff,0x0f,0xff,0xff
		};
		return m_regs[r] & mask[r];
	}
	else return m_regs[r];
}

/*************************************
 *
 * Sound Interface
 *
 *************************************/


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void ay8910_device::device_reset()
{
	ay8910_reset_ym();
}

/*************************************
 *
 * Read/Write Handlers
 *
 *************************************/

READ8_MEMBER( ay8910_device::data_r )
{
	return ay8910_read_ym();
}

WRITE8_MEMBER( ay8910_device::data_address_w )
{
	/* note that directly connecting BC1 to A0 puts data on 0 and address on 1 */
	ay8910_write_ym(~offset & 1, data);
}

WRITE8_MEMBER( ay8910_device::address_data_w )
{
	ay8910_write_ym(offset & 1, data);
}

WRITE8_MEMBER( ay8910_device::address_w )
{
#if ENABLE_REGISTER_TEST
	return;
#else
	data_address_w(space, 1, data);
#endif
}

WRITE8_MEMBER( ay8910_device::data_w )
{
#if ENABLE_REGISTER_TEST
	return;
#else
	data_address_w(space, 0, data);
#endif
}

// here, BC1 is hooked up to A0 on the host and BC2 is hooked up to A1
WRITE8_MEMBER( ay8910_device::write_bc1_bc2 )
{
	switch (offset & 3)
	{
	case 0: // latch address
		address_w(space, 0, data);
		break;
	case 1: // inactive
		break;
	case 2: // write to psg
		data_w(space, 0, data);
		break;
	case 3: // latch address
		address_w(space, 0, data);
		break;
	}
}

WRITE8_MEMBER( ay8910_device::reset_w )
{
	ay8910_reset_ym();
}

static const int mapping8914to8910[16] = { 0, 2, 4, 11, 1, 3, 5, 12, 7, 6, 13, 8, 9, 10, 14, 15 };

READ8_MEMBER( ay8914_device::read )
{
	uint16_t rv;
	address_w(space, 0, mapping8914to8910[offset & 0xf]);
	rv = (uint16_t) data_r(space, 0);
	return rv;
}

WRITE8_MEMBER( ay8914_device::write )
{
	address_w(space, 0, mapping8914to8910[offset & 0xf]);
	data_w(space, 0, data & 0xff);
}



DEFINE_DEVICE_TYPE(AY8910, ay8910_device, "ay8910", "AY-3-8910A PSG")

ay8910_device::ay8910_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: ay8910_device(mconfig, AY8910, tag, owner, clock, PSG_TYPE_AY, 3, 2)
{
}

ay8910_device::ay8910_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock,
								psg_type_t psg_type, int streams, int ioports)
	: device_t(mconfig, type, tag, owner, clock),
		device_sound_interface(mconfig, *this),
		m_type(psg_type),
		m_streams(streams),
		m_ioports(ioports),
		m_ready(0),
		m_channel(nullptr),
		m_active(false),
		m_register_latch(0),
		m_last_enable(0),
		m_prescale_noise(0),
		m_count_noise(0),
		m_count_env(0),
		m_env_step(0),
		m_env_volume(0),
		m_hold(0),
		m_alternate(0),
		m_attack(0),
		m_holding(0),
		m_rng(0),
		m_env_step_mask(psg_type == PSG_TYPE_AY ? 0x0f : 0x1f),
		m_step(         psg_type == PSG_TYPE_AY ? 2 : 1),
		m_zero_is_off(  psg_type == PSG_TYPE_AY ? 1 : 0),
		m_par(          psg_type == PSG_TYPE_AY ? &ay8910_param : &ym2149_param),
		m_par_env(      psg_type == PSG_TYPE_AY ? &ay8910_param : &ym2149_param_env),
		m_flags(AY8910_LEGACY_OUTPUT),
		m_port_a_read_cb(*this),
		m_port_b_read_cb(*this),
		m_port_a_write_cb(*this),
		m_port_b_write_cb(*this)
{
	memset(&m_regs,0,sizeof(m_regs));
	memset(&m_count,0,sizeof(m_count));
	memset(&m_output,0,sizeof(m_output));
	memset(&m_vol_enabled,0,sizeof(m_vol_enabled));
	memset(&m_vol_table,0,sizeof(m_vol_table));
	memset(&m_env_table,0,sizeof(m_env_table));
	m_res_load[0] = m_res_load[1] = m_res_load[2] = 1000; //Default values for resistor loads

	set_type(psg_type);
}

void ay8910_device::set_type(psg_type_t psg_type)
{
	m_type = psg_type;
	if (psg_type == PSG_TYPE_AY)
	{
		m_env_step_mask = 0x0f;
		m_step = 2;
		m_zero_is_off = 1;
		m_par = &ay8910_param;
		m_par_env = &ay8910_param;
	}
	else
	{
		m_env_step_mask = 0x1f;
		m_step = 1;
		m_zero_is_off = 0;
		m_par = &ym2149_param;
		m_par_env = &ym2149_param_env;
	}
}

DEFINE_DEVICE_TYPE(AY8912, ay8912_device, "ay8912", "AY-3-8912A PSG")

ay8912_device::ay8912_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: ay8910_device(mconfig, AY8912, tag, owner, clock, PSG_TYPE_AY, 3, 1)
{
}


DEFINE_DEVICE_TYPE(AY8913, ay8913_device, "ay8913", "AY-3-8913 PSG")

ay8913_device::ay8913_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: ay8910_device(mconfig, AY8913, tag, owner, clock, PSG_TYPE_AY, 3, 0)
{
}


DEFINE_DEVICE_TYPE(AY8914, ay8914_device, "ay8914", "AY-3-8914A PSG")

ay8914_device::ay8914_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: ay8910_device(mconfig, AY8914, tag, owner, clock, PSG_TYPE_AY, 3, 2)
{
}


DEFINE_DEVICE_TYPE(AY8930, ay8930_device, "ay8930", "AY8930 EPSG")

ay8930_device::ay8930_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: ay8910_device(mconfig, AY8930, tag, owner, clock, PSG_TYPE_AY, 3, 2)
{
}


DEFINE_DEVICE_TYPE(YM2149, ym2149_device, "ym2149", "YM2149 SSG")

ym2149_device::ym2149_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: ay8910_device(mconfig, YM2149, tag, owner, clock, PSG_TYPE_YM, 3, 2)
{
}


DEFINE_DEVICE_TYPE(YM3439, ym3439_device, "ym3439", "YM3439 SSGC")

ym3439_device::ym3439_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: ay8910_device(mconfig, YM3439, tag, owner, clock, PSG_TYPE_YM, 3, 2)
{
}


DEFINE_DEVICE_TYPE(YMZ284, ymz284_device, "ymz284", "YMZ284 SSGL")

ymz284_device::ymz284_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: ay8910_device(mconfig, YMZ284, tag, owner, clock, PSG_TYPE_YM, 1, 0)
{
}


DEFINE_DEVICE_TYPE(YMZ294, ymz294_device, "ymz294", "YMZ294 SSGLP")

ymz294_device::ymz294_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: ay8910_device(mconfig, YMZ294, tag, owner, clock, PSG_TYPE_YM, 1, 0)
{
}
