// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
#include "emu.h"
#include "h8s2320.h"

DEFINE_DEVICE_TYPE(H8S2320, h8s2320_device, "h8s2320", "H8S/2320")
DEFINE_DEVICE_TYPE(H8S2321, h8s2321_device, "h8s2321", "H8S/2321")
DEFINE_DEVICE_TYPE(H8S2322, h8s2322_device, "h8s2322", "H8S/2322")
DEFINE_DEVICE_TYPE(H8S2323, h8s2323_device, "h8s2323", "H8S/2323")
DEFINE_DEVICE_TYPE(H8S2324, h8s2324_device, "h8s2324", "H8S/2324")
DEFINE_DEVICE_TYPE(H8S2326, h8s2326_device, "h8s2326", "H8S/2326")
DEFINE_DEVICE_TYPE(H8S2327, h8s2327_device, "h8s2327", "H8S/2327")
DEFINE_DEVICE_TYPE(H8S2328, h8s2328_device, "h8s2328", "H8S/2328")
DEFINE_DEVICE_TYPE(H8S2329, h8s2329_device, "h8s2329", "H8S/2329")


h8s2320_device::h8s2320_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, uint32_t start) :
	h8s2000_device(mconfig, type, tag, owner, clock, address_map_delegate(FUNC(h8s2320_device::map), this)),
	intc(*this, "intc"),
	adc(*this, "adc"),
	dma(*this, "dma"),
	dma0(*this, "dma:0"),
	dma1(*this, "dma:1"),
	dtc(*this, "dtc"),
	port1(*this, "port1"),
	port2(*this, "port2"),
	port3(*this, "port3"),
	port4(*this, "port4"),
	port5(*this, "port5"),
	port6(*this, "port6"),
	porta(*this, "porta"),
	portb(*this, "portb"),
	portc(*this, "portc"),
	portd(*this, "portd"),
	porte(*this, "porte"),
	portf(*this, "portf"),
	portg(*this, "portg"),
	timer8_0(*this, "timer8_0"),
	timer8_1(*this, "timer8_1"),
	timer16(*this, "timer16"),
	timer16_0(*this, "timer16:0"),
	timer16_1(*this, "timer16:1"),
	timer16_2(*this, "timer16:2"),
	timer16_3(*this, "timer16:3"),
	timer16_4(*this, "timer16:4"),
	timer16_5(*this, "timer16:5"),
	sci0(*this, "sci0"),
	sci1(*this, "sci1"),
	sci2(*this, "sci2"),
	watchdog(*this, "watchdog"),
	ram_start(start),
	syscr(0)
{
}

h8s2320_device::h8s2320_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8s2320_device(mconfig, H8S2320, tag, owner, clock, 0xffec00)
{
}

h8s2321_device::h8s2321_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8s2320_device(mconfig, H8S2321, tag, owner, clock, 0xffec00)
{
}

h8s2322_device::h8s2322_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8s2320_device(mconfig, H8S2322, tag, owner, clock, 0xffdc00)
{
}

h8s2323_device::h8s2323_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8s2320_device(mconfig, H8S2323, tag, owner, clock, 0xffdc00)
{
}

h8s2324_device::h8s2324_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8s2320_device(mconfig, H8S2324, tag, owner, clock, 0xff7c00)
{
}

h8s2326_device::h8s2326_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8s2320_device(mconfig, H8S2326, tag, owner, clock, 0xffdc00)
{
}

h8s2327_device::h8s2327_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8s2320_device(mconfig, H8S2327, tag, owner, clock, 0xffdc00)
{
}

h8s2328_device::h8s2328_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8s2320_device(mconfig, H8S2328, tag, owner, clock, 0xffdc00)
{
}

h8s2329_device::h8s2329_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h8s2320_device(mconfig, H8S2329, tag, owner, clock, 0xff7c00)
{
}

DEVICE_ADDRESS_MAP_START(map, 16, h8s2320_device)
	AM_RANGE(ram_start, 0xfffbff) AM_RAM

	AM_RANGE(0xfffe80, 0xfffe81) AM_DEVREADWRITE8("timer16:3", h8_timer16_channel_device, tcr_r,    tcr_w,    0xff00)
	AM_RANGE(0xfffe80, 0xfffe81) AM_DEVREADWRITE8("timer16:3", h8_timer16_channel_device, tmdr_r,   tmdr_w,   0x00ff)
	AM_RANGE(0xfffe82, 0xfffe83) AM_DEVREADWRITE8("timer16:3", h8_timer16_channel_device, tior_r,   tior_w,   0xffff)
	AM_RANGE(0xfffe84, 0xfffe85) AM_DEVREADWRITE8("timer16:3", h8_timer16_channel_device, tier_r,   tier_w,   0xff00)
	AM_RANGE(0xfffe84, 0xfffe85) AM_DEVREADWRITE8("timer16:3", h8_timer16_channel_device, tsr_r,    tsr_w,    0x00ff)
	AM_RANGE(0xfffe86, 0xfffe87) AM_DEVREADWRITE( "timer16:3", h8_timer16_channel_device, tcnt_r,   tcnt_w          )
	AM_RANGE(0xfffe88, 0xfffe8f) AM_DEVREADWRITE( "timer16:3", h8_timer16_channel_device, tgr_r,    tgr_w           )
	AM_RANGE(0xfffe90, 0xfffe91) AM_DEVREADWRITE8("timer16:4", h8_timer16_channel_device, tcr_r,    tcr_w,    0xff00)
	AM_RANGE(0xfffe90, 0xfffe91) AM_DEVREADWRITE8("timer16:4", h8_timer16_channel_device, tmdr_r,   tmdr_w,   0x00ff)
	AM_RANGE(0xfffe92, 0xfffe93) AM_DEVREADWRITE8("timer16:4", h8_timer16_channel_device, tior_r,   tior_w,   0xff00)
	AM_RANGE(0xfffe94, 0xfffe95) AM_DEVREADWRITE8("timer16:4", h8_timer16_channel_device, tier_r,   tier_w,   0xff00)
	AM_RANGE(0xfffe94, 0xfffe95) AM_DEVREADWRITE8("timer16:4", h8_timer16_channel_device, tsr_r,    tsr_w,    0x00ff)
	AM_RANGE(0xfffe96, 0xfffe97) AM_DEVREADWRITE( "timer16:4", h8_timer16_channel_device, tcnt_r,   tcnt_w          )
	AM_RANGE(0xfffe98, 0xfffe9b) AM_DEVREADWRITE( "timer16:4", h8_timer16_channel_device, tgr_r,    tgr_w           )
	AM_RANGE(0xfffea0, 0xfffea1) AM_DEVREADWRITE8("timer16:5", h8_timer16_channel_device, tcr_r,    tcr_w,    0xff00)
	AM_RANGE(0xfffea0, 0xfffea1) AM_DEVREADWRITE8("timer16:5", h8_timer16_channel_device, tmdr_r,   tmdr_w,   0x00ff)
	AM_RANGE(0xfffea2, 0xfffea3) AM_DEVREADWRITE8("timer16:5", h8_timer16_channel_device, tior_r,   tior_w,   0xff00)
	AM_RANGE(0xfffea4, 0xfffea5) AM_DEVREADWRITE8("timer16:5", h8_timer16_channel_device, tier_r,   tier_w,   0xff00)
	AM_RANGE(0xfffea4, 0xfffea5) AM_DEVREADWRITE8("timer16:5", h8_timer16_channel_device, tsr_r,    tsr_w,    0x00ff)
	AM_RANGE(0xfffea6, 0xfffea7) AM_DEVREADWRITE( "timer16:5", h8_timer16_channel_device, tcnt_r,   tcnt_w          )
	AM_RANGE(0xfffea8, 0xfffeab) AM_DEVREADWRITE( "timer16:5", h8_timer16_channel_device, tgr_r,    tgr_w           )
	AM_RANGE(0xfffeb0, 0xfffeb1) AM_DEVWRITE8(    "port1",     h8_port_device,                      ddr_w,    0xff00)
	AM_RANGE(0xfffeb0, 0xfffeb1) AM_DEVWRITE8(    "port2",     h8_port_device,                      ddr_w,    0x00ff)
	AM_RANGE(0xfffeb2, 0xfffeb3) AM_DEVWRITE8(    "port3",     h8_port_device,                      ddr_w,    0xff00)
	AM_RANGE(0xfffeb4, 0xfffeb5) AM_DEVWRITE8(    "port5",     h8_port_device,                      ddr_w,    0xff00)
	AM_RANGE(0xfffeb4, 0xfffeb5) AM_DEVWRITE8(    "port6",     h8_port_device,                      ddr_w,    0x00ff)
	AM_RANGE(0xfffeb8, 0xfffeb9) AM_DEVWRITE8(    "porta",     h8_port_device,                      ddr_w,    0x00ff)
	AM_RANGE(0xfffeba, 0xfffebb) AM_DEVWRITE8(    "portb",     h8_port_device,                      ddr_w,    0xff00)
	AM_RANGE(0xfffeba, 0xfffebb) AM_DEVWRITE8(    "portc",     h8_port_device,                      ddr_w,    0x00ff)
	AM_RANGE(0xfffebc, 0xfffebd) AM_DEVWRITE8(    "portd",     h8_port_device,                      ddr_w,    0xff00)
	AM_RANGE(0xfffebc, 0xfffebd) AM_DEVWRITE8(    "porte",     h8_port_device,                      ddr_w,    0x00ff)
	AM_RANGE(0xfffebe, 0xfffebf) AM_DEVWRITE8(    "portf",     h8_port_device,                      ddr_w,    0xff00)
	AM_RANGE(0xfffebe, 0xfffebf) AM_DEVWRITE8(    "portg",     h8_port_device,                      ddr_w,    0x00ff)
	AM_RANGE(0xfffec0, 0xfffec1) AM_DEVREADWRITE8("intc",      h8s_intc_device,           icr_r,    icr_w,    0xffff)
	AM_RANGE(0xfffec2, 0xfffec3) AM_DEVREADWRITE8("intc",      h8s_intc_device,           icrc_r,   icrc_w,   0xff00)
	AM_RANGE(0xfffec4, 0xfffecd) AM_DEVREADWRITE8("intc",      h8s_intc_device,           ipr_r,    ipr_w,    0xffff)
	AM_RANGE(0xfffece, 0xfffecf) AM_DEVREADWRITE8("intc",      h8s_intc_device,           iprk_r,   iprk_w,   0xff00)

	AM_RANGE(0xfffee0, 0xfffee1) AM_DEVREADWRITE( "dma:0",     h8_dma_channel_device,     marah_r,  marah_w         )
	AM_RANGE(0xfffee2, 0xfffee3) AM_DEVREADWRITE( "dma:0",     h8_dma_channel_device,     maral_r,  maral_w         )
	AM_RANGE(0xfffee4, 0xfffee5) AM_DEVREADWRITE( "dma:0",     h8_dma_channel_device,     ioara_r,  ioara_w         )
	AM_RANGE(0xfffee6, 0xfffee7) AM_DEVREADWRITE( "dma:0",     h8_dma_channel_device,     etcra_r,  etcra_w         )
	AM_RANGE(0xfffee8, 0xfffee9) AM_DEVREADWRITE( "dma:0",     h8_dma_channel_device,     marbh_r,  marbh_w         )
	AM_RANGE(0xfffeea, 0xfffeeb) AM_DEVREADWRITE( "dma:0",     h8_dma_channel_device,     marbl_r,  marbl_w         )
	AM_RANGE(0xfffeec, 0xfffeed) AM_DEVREADWRITE( "dma:0",     h8_dma_channel_device,     ioarb_r,  ioarb_w         )
	AM_RANGE(0xfffeee, 0xfffeef) AM_DEVREADWRITE( "dma:0",     h8_dma_channel_device,     etcrb_r,  etcrb_w         )
	AM_RANGE(0xfffef0, 0xfffef1) AM_DEVREADWRITE( "dma:1",     h8_dma_channel_device,     marah_r,  marah_w         )
	AM_RANGE(0xfffef2, 0xfffef3) AM_DEVREADWRITE( "dma:1",     h8_dma_channel_device,     maral_r,  maral_w         )
	AM_RANGE(0xfffef4, 0xfffef5) AM_DEVREADWRITE( "dma:1",     h8_dma_channel_device,     ioara_r,  ioara_w         )
	AM_RANGE(0xfffef6, 0xfffef7) AM_DEVREADWRITE( "dma:1",     h8_dma_channel_device,     etcra_r,  etcra_w         )
	AM_RANGE(0xfffef8, 0xfffef9) AM_DEVREADWRITE( "dma:1",     h8_dma_channel_device,     marbh_r,  marbh_w         )
	AM_RANGE(0xfffefa, 0xfffefb) AM_DEVREADWRITE( "dma:1",     h8_dma_channel_device,     marbl_r,  marbl_w         )
	AM_RANGE(0xfffefc, 0xfffefd) AM_DEVREADWRITE( "dma:1",     h8_dma_channel_device,     ioarb_r,  ioarb_w         )
	AM_RANGE(0xfffefe, 0xfffeff) AM_DEVREADWRITE( "dma:1",     h8_dma_channel_device,     etcrb_r,  etcrb_w         )
	AM_RANGE(0xffff00, 0xffff01) AM_DEVREADWRITE8("dma",       h8_dma_device,             dmawer_r, dmawer_w, 0xff00)
	AM_RANGE(0xffff00, 0xffff01) AM_DEVREADWRITE8("dma",       h8_dma_device,             dmatcr_r, dmatcr_w, 0x00ff)
	AM_RANGE(0xffff02, 0xffff03) AM_DEVREADWRITE( "dma:0",     h8_dma_channel_device,     dmacr_r,  dmacr_w         )
	AM_RANGE(0xffff04, 0xffff05) AM_DEVREADWRITE( "dma:1",     h8_dma_channel_device,     dmacr_r,  dmacr_w         )
	AM_RANGE(0xffff06, 0xffff07) AM_DEVREADWRITE( "dma",       h8_dma_device,             dmabcr_r, dmabcr_w        )
	AM_RANGE(0xffff2c, 0xffff2d) AM_DEVREADWRITE8("intc",      h8s_intc_device,           iscrh_r,  iscrh_w,  0xff00)
	AM_RANGE(0xffff2c, 0xffff2d) AM_DEVREADWRITE8("intc",      h8s_intc_device,           iscrl_r,  iscrl_w,  0x00ff)
	AM_RANGE(0xffff2e, 0xffff2f) AM_DEVREADWRITE8("intc",      h8s_intc_device,           ier_r,    ier_w,    0xff00)
	AM_RANGE(0xffff2e, 0xffff2f) AM_DEVREADWRITE8("intc",      h8s_intc_device,           isr_r,    isr_w,    0x00ff)
	AM_RANGE(0xffff30, 0xffff35) AM_DEVREADWRITE8("dtc",       h8_dtc_device,             dtcer_r,  dtcer_w,  0xffff)
	AM_RANGE(0xffff36, 0xffff37) AM_DEVREADWRITE8("dtc",       h8_dtc_device,             dtvecr_r, dtvecr_w, 0x00ff)
	AM_RANGE(0xffff38, 0xffff39) AM_READWRITE8(                                           syscr_r,  syscr_w,  0x00ff)

	AM_RANGE(0xffff50, 0xffff51) AM_DEVREAD8(     "port1",     h8_port_device,            port_r,             0xff00)
	AM_RANGE(0xffff50, 0xffff51) AM_DEVREAD8(     "port2",     h8_port_device,            port_r,             0x00ff)
	AM_RANGE(0xffff52, 0xffff53) AM_DEVREAD8(     "port3",     h8_port_device,            port_r,             0xff00)
	AM_RANGE(0xffff52, 0xffff53) AM_DEVREAD8(     "port4",     h8_port_device,            port_r,             0x00ff)
	AM_RANGE(0xffff54, 0xffff55) AM_DEVREAD8(     "port5",     h8_port_device,            port_r,             0xff00)
	AM_RANGE(0xffff54, 0xffff55) AM_DEVREAD8(     "port6",     h8_port_device,            port_r,             0x00ff)
	AM_RANGE(0xffff58, 0xffff59) AM_DEVREAD8(     "porta",     h8_port_device,            port_r,             0x00ff)
	AM_RANGE(0xffff5a, 0xffff5b) AM_DEVREAD8(     "portb",     h8_port_device,            port_r,             0xff00)
	AM_RANGE(0xffff5a, 0xffff5b) AM_DEVREAD8(     "portc",     h8_port_device,            port_r,             0x00ff)
	AM_RANGE(0xffff5c, 0xffff5d) AM_DEVREAD8(     "portd",     h8_port_device,            port_r,             0xff00)
	AM_RANGE(0xffff5c, 0xffff5d) AM_DEVREAD8(     "porte",     h8_port_device,            port_r,             0x00ff)
	AM_RANGE(0xffff5e, 0xffff5f) AM_DEVREAD8(     "portf",     h8_port_device,            port_r,             0xff00)
	AM_RANGE(0xffff5e, 0xffff5f) AM_DEVREAD8(     "portg",     h8_port_device,            port_r,             0x00ff)
	AM_RANGE(0xffff60, 0xffff61) AM_DEVREADWRITE8("port1",     h8_port_device,            dr_r,     dr_w,     0xff00)
	AM_RANGE(0xffff60, 0xffff61) AM_DEVREADWRITE8("port2",     h8_port_device,            dr_r,     dr_w,     0x00ff)
	AM_RANGE(0xffff62, 0xffff63) AM_DEVREADWRITE8("port3",     h8_port_device,            dr_r,     dr_w,     0xff00)
	AM_RANGE(0xffff64, 0xffff65) AM_DEVREADWRITE8("port5",     h8_port_device,            dr_r,     dr_w,     0xff00)
	AM_RANGE(0xffff64, 0xffff65) AM_DEVREADWRITE8("port6",     h8_port_device,            dr_r,     dr_w,     0x00ff)
	AM_RANGE(0xffff68, 0xffff69) AM_DEVREADWRITE8("porta",     h8_port_device,            dr_r,     dr_w,     0x00ff)
	AM_RANGE(0xffff6a, 0xffff6b) AM_DEVREADWRITE8("portb",     h8_port_device,            dr_r,     dr_w,     0xff00)
	AM_RANGE(0xffff6a, 0xffff6b) AM_DEVREADWRITE8("portc",     h8_port_device,            dr_r,     dr_w,     0x00ff)
	AM_RANGE(0xffff6c, 0xffff6d) AM_DEVREADWRITE8("portd",     h8_port_device,            dr_r,     dr_w,     0xff00)
	AM_RANGE(0xffff6c, 0xffff6d) AM_DEVREADWRITE8("porte",     h8_port_device,            dr_r,     dr_w,     0x00ff)
	AM_RANGE(0xffff6e, 0xffff6f) AM_DEVREADWRITE8("portf",     h8_port_device,            dr_r,     dr_w,     0xff00)
	AM_RANGE(0xffff6e, 0xffff6f) AM_DEVREADWRITE8("portg",     h8_port_device,            dr_r,     dr_w,     0x00ff)
	AM_RANGE(0xffff70, 0xffff71) AM_DEVREADWRITE8("porta",     h8_port_device,            pcr_r,    pcr_w,    0xff00)
	AM_RANGE(0xffff70, 0xffff71) AM_DEVREADWRITE8("portb",     h8_port_device,            pcr_r,    pcr_w,    0x00ff)
	AM_RANGE(0xffff72, 0xffff73) AM_DEVREADWRITE8("portc",     h8_port_device,            pcr_r,    pcr_w,    0xff00)
	AM_RANGE(0xffff72, 0xffff73) AM_DEVREADWRITE8("portd",     h8_port_device,            pcr_r,    pcr_w,    0x00ff)
	AM_RANGE(0xffff74, 0xffff75) AM_DEVREADWRITE8("porte",     h8_port_device,            pcr_r,    pcr_w,    0xff00)
	AM_RANGE(0xffff76, 0xffff77) AM_DEVREADWRITE8("port3",     h8_port_device,            odr_r,    odr_w,    0xff00)
	AM_RANGE(0xffff76, 0xffff77) AM_DEVREADWRITE8("porta",     h8_port_device,            odr_r,    odr_w,    0x00ff)
	AM_RANGE(0xffff78, 0xffff79) AM_DEVREADWRITE8("sci0",      h8_sci_device,             smr_r,    smr_w,    0xff00)
	AM_RANGE(0xffff78, 0xffff79) AM_DEVREADWRITE8("sci0",      h8_sci_device,             brr_r,    brr_w,    0x00ff)
	AM_RANGE(0xffff7a, 0xffff7b) AM_DEVREADWRITE8("sci0",      h8_sci_device,             scr_r,    scr_w,    0xff00)
	AM_RANGE(0xffff7a, 0xffff7b) AM_DEVREADWRITE8("sci0",      h8_sci_device,             tdr_r,    tdr_w,    0x00ff)
	AM_RANGE(0xffff7c, 0xffff7d) AM_DEVREADWRITE8("sci0",      h8_sci_device,             ssr_r,    ssr_w,    0xff00)
	AM_RANGE(0xffff7c, 0xffff7d) AM_DEVREAD8(     "sci0",      h8_sci_device,             rdr_r,              0x00ff)
	AM_RANGE(0xffff7e, 0xffff7f) AM_DEVREADWRITE8("sci0",      h8_sci_device,             scmr_r,   scmr_w,   0xff00)
	AM_RANGE(0xffff80, 0xffff81) AM_DEVREADWRITE8("sci1",      h8_sci_device,             smr_r,    smr_w,    0xff00)
	AM_RANGE(0xffff80, 0xffff81) AM_DEVREADWRITE8("sci1",      h8_sci_device,             brr_r,    brr_w,    0x00ff)
	AM_RANGE(0xffff82, 0xffff83) AM_DEVREADWRITE8("sci1",      h8_sci_device,             scr_r,    scr_w,    0xff00)
	AM_RANGE(0xffff82, 0xffff83) AM_DEVREADWRITE8("sci1",      h8_sci_device,             tdr_r,    tdr_w,    0x00ff)
	AM_RANGE(0xffff84, 0xffff85) AM_DEVREADWRITE8("sci1",      h8_sci_device,             ssr_r,    ssr_w,    0xff00)
	AM_RANGE(0xffff84, 0xffff85) AM_DEVREAD8(     "sci1",      h8_sci_device,             rdr_r,              0x00ff)
	AM_RANGE(0xffff86, 0xffff87) AM_DEVREADWRITE8("sci1",      h8_sci_device,             scmr_r,   scmr_w,   0xff00)
	AM_RANGE(0xffff88, 0xffff89) AM_DEVREADWRITE8("sci2",      h8_sci_device,             smr_r,    smr_w,    0xff00)
	AM_RANGE(0xffff88, 0xffff89) AM_DEVREADWRITE8("sci2",      h8_sci_device,             brr_r,    brr_w,    0x00ff)
	AM_RANGE(0xffff8a, 0xffff8b) AM_DEVREADWRITE8("sci2",      h8_sci_device,             scr_r,    scr_w,    0xff00)
	AM_RANGE(0xffff8a, 0xffff8b) AM_DEVREADWRITE8("sci2",      h8_sci_device,             tdr_r,    tdr_w,    0x00ff)
	AM_RANGE(0xffff8c, 0xffff8d) AM_DEVREADWRITE8("sci2",      h8_sci_device,             ssr_r,    ssr_w,    0xff00)
	AM_RANGE(0xffff8c, 0xffff8d) AM_DEVREAD8(     "sci2",      h8_sci_device,             rdr_r,              0x00ff)
	AM_RANGE(0xffff8e, 0xffff8f) AM_DEVREADWRITE8("sci2",      h8_sci_device,             scmr_r,   scmr_w,   0xff00)
	AM_RANGE(0xffff90, 0xffff97) AM_DEVREAD8(     "adc",       h8_adc_device,             addr8_r,            0xffff)
	AM_RANGE(0xffff98, 0xffff99) AM_DEVREADWRITE8("adc",       h8_adc_device,             adcsr_r,  adcsr_w,  0xff00)
	AM_RANGE(0xffff98, 0xffff99) AM_DEVREADWRITE8("adc",       h8_adc_device,             adcr_r,   adcr_w,   0x00ff)

	AM_RANGE(0xffffb0, 0xffffb1) AM_DEVREADWRITE8("timer8_0",  h8_timer8_channel_device,  tcr_r,    tcr_w,    0xff00)
	AM_RANGE(0xffffb0, 0xffffb1) AM_DEVREADWRITE8("timer8_1",  h8_timer8_channel_device,  tcr_r,    tcr_w,    0x00ff)
	AM_RANGE(0xffffb2, 0xffffb3) AM_DEVREADWRITE8("timer8_0",  h8_timer8_channel_device,  tcsr_r,   tcsr_w,   0xff00)
	AM_RANGE(0xffffb2, 0xffffb3) AM_DEVREADWRITE8("timer8_1",  h8_timer8_channel_device,  tcsr_r,   tcsr_w,   0x00ff)
	AM_RANGE(0xffffb4, 0xffffb7) AM_DEVREADWRITE8("timer8_0",  h8_timer8_channel_device,  tcor_r,   tcor_w,   0xff00)
	AM_RANGE(0xffffb4, 0xffffb7) AM_DEVREADWRITE8("timer8_1",  h8_timer8_channel_device,  tcor_r,   tcor_w,   0x00ff)
	AM_RANGE(0xffffb8, 0xffffb9) AM_DEVREADWRITE8("timer8_0",  h8_timer8_channel_device,  tcnt_r,   tcnt_w,   0xff00)
	AM_RANGE(0xffffb8, 0xffffb9) AM_DEVREADWRITE8("timer8_1",  h8_timer8_channel_device,  tcnt_r,   tcnt_w,   0x00ff)
	AM_RANGE(0xffffbc, 0xffffbd) AM_DEVREADWRITE( "watchdog",  h8_watchdog_device,        wd_r,     wd_w            )
	AM_RANGE(0xffffbe, 0xffffbf) AM_DEVREADWRITE( "watchdog",  h8_watchdog_device,        rst_r,    rst_w           )
	AM_RANGE(0xffffc0, 0xffffc1) AM_DEVREADWRITE8("timer16",   h8_timer16_device,         tstr_r,   tstr_w,   0xff00)
	AM_RANGE(0xffffc0, 0xffffc1) AM_DEVREADWRITE8("timer16",   h8_timer16_device,         tsyr_r,   tsyr_w,   0x00ff)

	AM_RANGE(0xffffd0, 0xffffd1) AM_DEVREADWRITE8("timer16:0", h8_timer16_channel_device, tcr_r,    tcr_w,    0xff00)
	AM_RANGE(0xffffd0, 0xffffd1) AM_DEVREADWRITE8("timer16:0", h8_timer16_channel_device, tmdr_r,   tmdr_w,   0x00ff)
	AM_RANGE(0xffffd2, 0xffffd3) AM_DEVREADWRITE8("timer16:0", h8_timer16_channel_device, tior_r,   tior_w,   0xffff)
	AM_RANGE(0xffffd4, 0xffffd5) AM_DEVREADWRITE8("timer16:0", h8_timer16_channel_device, tier_r,   tier_w,   0xff00)
	AM_RANGE(0xffffd4, 0xffffd5) AM_DEVREADWRITE8("timer16:0", h8_timer16_channel_device, tsr_r,    tsr_w,    0x00ff)
	AM_RANGE(0xffffd6, 0xffffd7) AM_DEVREADWRITE( "timer16:0", h8_timer16_channel_device, tcnt_r,   tcnt_w          )
	AM_RANGE(0xffffd8, 0xffffdf) AM_DEVREADWRITE( "timer16:0", h8_timer16_channel_device, tgr_r,    tgr_w           )
	AM_RANGE(0xffffe0, 0xffffe1) AM_DEVREADWRITE8("timer16:1", h8_timer16_channel_device, tcr_r,    tcr_w,    0xff00)
	AM_RANGE(0xffffe0, 0xffffe1) AM_DEVREADWRITE8("timer16:1", h8_timer16_channel_device, tmdr_r,   tmdr_w,   0x00ff)
	AM_RANGE(0xffffe2, 0xffffe3) AM_DEVREADWRITE8("timer16:1", h8_timer16_channel_device, tior_r,   tior_w,   0xff00)
	AM_RANGE(0xffffe4, 0xffffe5) AM_DEVREADWRITE8("timer16:1", h8_timer16_channel_device, tier_r,   tier_w,   0xff00)
	AM_RANGE(0xffffe4, 0xffffe5) AM_DEVREADWRITE8("timer16:1", h8_timer16_channel_device, tsr_r,    tsr_w,    0x00ff)
	AM_RANGE(0xffffe6, 0xffffe7) AM_DEVREADWRITE( "timer16:1", h8_timer16_channel_device, tcnt_r,   tcnt_w          )
	AM_RANGE(0xffffe8, 0xffffeb) AM_DEVREADWRITE( "timer16:1", h8_timer16_channel_device, tgr_r,    tgr_w           )
	AM_RANGE(0xfffff0, 0xfffff1) AM_DEVREADWRITE8("timer16:2", h8_timer16_channel_device, tcr_r,    tcr_w,    0xff00)
	AM_RANGE(0xfffff0, 0xfffff1) AM_DEVREADWRITE8("timer16:2", h8_timer16_channel_device, tmdr_r,   tmdr_w,   0x00ff)
	AM_RANGE(0xfffff2, 0xfffff3) AM_DEVREADWRITE8("timer16:2", h8_timer16_channel_device, tior_r,   tior_w,   0xff00)
	AM_RANGE(0xfffff4, 0xfffff5) AM_DEVREADWRITE8("timer16:2", h8_timer16_channel_device, tier_r,   tier_w,   0xff00)
	AM_RANGE(0xfffff4, 0xfffff5) AM_DEVREADWRITE8("timer16:2", h8_timer16_channel_device, tsr_r,    tsr_w,    0x00ff)
	AM_RANGE(0xfffff6, 0xfffff7) AM_DEVREADWRITE( "timer16:2", h8_timer16_channel_device, tcnt_r,   tcnt_w          )
	AM_RANGE(0xfffff8, 0xfffffb) AM_DEVREADWRITE( "timer16:2", h8_timer16_channel_device, tgr_r,    tgr_w           )
ADDRESS_MAP_END

// TODO: the 2321 doesn't have the dma subdevice

MACHINE_CONFIG_MEMBER(h8s2320_device::device_add_mconfig)
	MCFG_H8S_INTC_ADD("intc")
	MCFG_H8_ADC_2320_ADD("adc", "intc", 28)
	MCFG_H8_DMA_ADD("dma")
	MCFG_H8_DMA_CHANNEL_ADD("dma:0", "intc", 72, h8_dma_channel_device::NONE, 28, h8_dma_channel_device::NONE, h8_dma_channel_device::NONE, 82, 81, 86, 85, 32, 40, 44, 48, 56, 60, h8_dma_channel_device::NONE, h8_dma_channel_device::NONE)
	MCFG_H8_DMA_CHANNEL_ADD("dma:1", "intc", 74, h8_dma_channel_device::NONE, 28, h8_dma_channel_device::DREQ_EDGE, h8_dma_channel_device::DREQ_LEVEL, 82, 81, 86, 85, 32, 40, 44, 48, 56, 60, h8_dma_channel_device::NONE, h8_dma_channel_device::NONE)
	MCFG_H8_DTC_ADD("dtc", "intc", 24)
	MCFG_H8_PORT_ADD("port1", h8_device::PORT_1, 0x00, 0x00)
	MCFG_H8_PORT_ADD("port2", h8_device::PORT_2, 0x00, 0x00)
	MCFG_H8_PORT_ADD("port3", h8_device::PORT_3, 0xc0, 0xc0)
	MCFG_H8_PORT_ADD("port4", h8_device::PORT_4, 0x00, 0x00)
	MCFG_H8_PORT_ADD("port5", h8_device::PORT_5, 0xf0, 0xf0)
	MCFG_H8_PORT_ADD("port6", h8_device::PORT_6, 0x00, 0x00)
	MCFG_H8_PORT_ADD("porta", h8_device::PORT_A, 0x00, 0x00)
	MCFG_H8_PORT_ADD("portb", h8_device::PORT_B, 0x00, 0x00)
	MCFG_H8_PORT_ADD("portc", h8_device::PORT_C, 0x00, 0x00)
	MCFG_H8_PORT_ADD("portd", h8_device::PORT_D, 0x00, 0x00)
	MCFG_H8_PORT_ADD("porte", h8_device::PORT_E, 0x00, 0x00)
	MCFG_H8_PORT_ADD("portf", h8_device::PORT_F, 0x00, 0x00)
	MCFG_H8_PORT_ADD("portg", h8_device::PORT_G, 0xe0, 0xe0)
	MCFG_H8H_TIMER8_CHANNEL_ADD("timer8_0", "intc", 64, 65, 66, "timer8_1", h8_timer8_channel_device::CHAIN_OVERFLOW, true,  false)
	MCFG_H8H_TIMER8_CHANNEL_ADD("timer8_1", "intc", 68, 69, 70, "timer8_0", h8_timer8_channel_device::CHAIN_A,        false, false)
	MCFG_H8_TIMER16_ADD("timer16", 6, 0x00)
	MCFG_H8S_TIMER16_CHANNEL_ADD("timer16:0", 4, 0x60, "intc", 32,
									h8_timer16_channel_device::DIV_1,
									h8_timer16_channel_device::DIV_4,
									h8_timer16_channel_device::DIV_16,
									h8_timer16_channel_device::DIV_64,
									h8_timer16_channel_device::INPUT_A,
									h8_timer16_channel_device::INPUT_B,
									h8_timer16_channel_device::INPUT_C,
									h8_timer16_channel_device::INPUT_D)
	MCFG_H8S_TIMER16_CHANNEL_ADD("timer16:1", 2, 0x4c, "intc", 40,
									h8_timer16_channel_device::DIV_1,
									h8_timer16_channel_device::DIV_4,
									h8_timer16_channel_device::DIV_16,
									h8_timer16_channel_device::DIV_64,
									h8_timer16_channel_device::INPUT_A,
									h8_timer16_channel_device::INPUT_B,
									h8_timer16_channel_device::DIV_256,
									h8_timer16_channel_device::CHAIN)
	MCFG_H8S_TIMER16_CHANNEL_SET_CHAIN("timer16:2")
	MCFG_H8S_TIMER16_CHANNEL_ADD("timer16:2", 2, 0x4c, "intc", 44,
									h8_timer16_channel_device::DIV_1,
									h8_timer16_channel_device::DIV_4,
									h8_timer16_channel_device::DIV_16,
									h8_timer16_channel_device::DIV_64,
									h8_timer16_channel_device::INPUT_A,
									h8_timer16_channel_device::INPUT_B,
									h8_timer16_channel_device::INPUT_C,
									h8_timer16_channel_device::DIV_1024)
	MCFG_H8S_TIMER16_CHANNEL_ADD("timer16:3", 4, 0x60, "intc", 48,
									h8_timer16_channel_device::DIV_1,
									h8_timer16_channel_device::DIV_4,
									h8_timer16_channel_device::DIV_16,
									h8_timer16_channel_device::DIV_64,
									h8_timer16_channel_device::INPUT_A,
									h8_timer16_channel_device::DIV_1024,
									h8_timer16_channel_device::DIV_256,
									h8_timer16_channel_device::DIV_4096)
	MCFG_H8S_TIMER16_CHANNEL_ADD("timer16:4", 2, 0x4c, "intc", 56,
									h8_timer16_channel_device::DIV_1,
									h8_timer16_channel_device::DIV_4,
									h8_timer16_channel_device::DIV_16,
									h8_timer16_channel_device::DIV_64,
									h8_timer16_channel_device::INPUT_A,
									h8_timer16_channel_device::INPUT_C,
									h8_timer16_channel_device::DIV_1024,
									h8_timer16_channel_device::CHAIN)
	MCFG_H8S_TIMER16_CHANNEL_SET_CHAIN("timer16:5")
	MCFG_H8S_TIMER16_CHANNEL_ADD("timer16:5", 2, 0x4c, "intc", 60,
									h8_timer16_channel_device::DIV_1,
									h8_timer16_channel_device::DIV_4,
									h8_timer16_channel_device::DIV_16,
									h8_timer16_channel_device::DIV_64,
									h8_timer16_channel_device::INPUT_A,
									h8_timer16_channel_device::INPUT_C,
									h8_timer16_channel_device::DIV_256,
									h8_timer16_channel_device::INPUT_D)
	MCFG_H8_SCI_ADD("sci0", "intc", 80, 81, 82, 83)
	MCFG_H8_SCI_ADD("sci1", "intc", 84, 85, 86, 87)
	MCFG_H8_SCI_ADD("sci2", "intc", 88, 89, 90, 91)
	MCFG_H8_WATCHDOG_ADD("watchdog", "intc", 25, h8_watchdog_device::H)
MACHINE_CONFIG_END

void h8s2320_device::execute_set_input(int inputnum, int state)
{
	intc->set_input(inputnum, state);
}

bool h8s2320_device::exr_in_stack() const
{
	return syscr & 0x20;
}

int h8s2320_device::trace_setup()
{
	CCR |= F_I;
	EXR &= ~EXR_T;
	return 5;
}

int h8s2320_device::trapa_setup()
{
	CCR |= F_I;
	if(syscr & 0x20)
		EXR &= ~EXR_T;
	return 8;
}

void h8s2320_device::irq_setup()
{
	switch(syscr & 0x30) {
	case 0x00:
		CCR |= F_I;
		break;
	case 0x20:
		EXR = EXR & (EXR_NC);
		if(taken_irq_level == 8)
			EXR |= 7;
		else
			EXR |= taken_irq_level;
		break;
	}
}

void h8s2320_device::update_irq_filter()
{
	switch(syscr & 0x30) {
	case 0x00:
		if(CCR & F_I)
			intc->set_filter(2, -1);
		else
			intc->set_filter(0, -1);
		break;
	case 0x20:
		intc->set_filter(0, EXR & 7);
		break;
	}
}

void h8s2320_device::interrupt_taken()
{
	standard_irq_callback(intc->interrupt_taken(taken_irq_vector));
}

void h8s2320_device::internal_update(uint64_t current_time)
{
	uint64_t event_time = 0;

	add_event(event_time, adc->internal_update(current_time));
	add_event(event_time, sci0->internal_update(current_time));
	add_event(event_time, sci1->internal_update(current_time));
	add_event(event_time, sci2->internal_update(current_time));
	add_event(event_time, timer8_0->internal_update(current_time));
	add_event(event_time, timer8_1->internal_update(current_time));
	add_event(event_time, timer16_0->internal_update(current_time));
	add_event(event_time, timer16_1->internal_update(current_time));
	add_event(event_time, timer16_2->internal_update(current_time));
	add_event(event_time, timer16_3->internal_update(current_time));
	add_event(event_time, timer16_4->internal_update(current_time));
	add_event(event_time, timer16_5->internal_update(current_time));
	add_event(event_time, watchdog->internal_update(current_time));

	recompute_bcount(event_time);
}

void h8s2320_device::device_start()
{
	h8s2000_device::device_start();
	dma_device = dma;
	dtc_device = dtc;
}

void h8s2320_device::device_reset()
{
	h8s2000_device::device_reset();
	syscr = 0x01;
}

READ8_MEMBER(h8s2320_device::syscr_r)
{
	return syscr;
}

WRITE8_MEMBER(h8s2320_device::syscr_w)
{
	syscr = data;
	mac_saturating = syscr & 0x80;
	update_irq_filter();
	logerror("syscr = %02x\n", data);
}
