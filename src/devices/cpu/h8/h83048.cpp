// license:BSD-3-Clause
// copyright-holders:Olivier Galibert
#include "emu.h"
#include "h83048.h"

DEFINE_DEVICE_TYPE(H83044, h83044_device, "h83044", "H8/3044")
DEFINE_DEVICE_TYPE(H83045, h83045_device, "h83045", "H8/3045")
DEFINE_DEVICE_TYPE(H83047, h83047_device, "h83047", "H8/3047")
DEFINE_DEVICE_TYPE(H83048, h83048_device, "h83048", "H8/3048")

h83048_device::h83048_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, uint32_t start) :
	h8h_device(mconfig, type, tag, owner, clock, address_map_delegate(FUNC(h83048_device::map), this)),
	intc(*this, "intc"),
	adc(*this, "adc"),
	port1(*this, "port1"),
	port2(*this, "port2"),
	port3(*this, "port3"),
	port4(*this, "port4"),
	port5(*this, "port5"),
	port6(*this, "port6"),
	port7(*this, "port7"),
	port8(*this, "port8"),
	port9(*this, "port9"),
	porta(*this, "porta"),
	portb(*this, "portb"),
	timer16(*this, "timer16"),
	timer16_0(*this, "timer16:0"),
	timer16_1(*this, "timer16:1"),
	timer16_2(*this, "timer16:2"),
	timer16_3(*this, "timer16:3"),
	timer16_4(*this, "timer16:4"),
	sci0(*this, "sci0"),
	sci1(*this, "sci1"),
	watchdog(*this, "watchdog"),
	ram_start(start),
	syscr(0)
{
}

h83048_device::h83048_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h83048_device(mconfig, H83048, tag, owner, clock, 0xffef10)
{
}

h83044_device::h83044_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h83048_device(mconfig, H83044, tag, owner, clock, 0xfff710)
{
}

h83045_device::h83045_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h83048_device(mconfig, H83045, tag, owner, clock, 0xfff710)
{
}

h83047_device::h83047_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock) :
	h83048_device(mconfig, H83047, tag, owner, clock, 0xffef10)
{
}

DEVICE_ADDRESS_MAP_START(map, 16, h83048_device)
	AM_RANGE(ram_start, 0xffff0f) AM_RAM

	AM_RANGE(0xffff60, 0xffff61) AM_DEVREADWRITE8("timer16",   h8_timer16_device,         tstr_r,  tstr_w,  0xff00)
	AM_RANGE(0xffff60, 0xffff61) AM_DEVREADWRITE8("timer16",   h8_timer16_device,         tsyr_r,  tsyr_w,  0x00ff)
	AM_RANGE(0xffff62, 0xffff63) AM_DEVREADWRITE8("timer16",   h8_timer16_device,         tmdr_r,  tmdr_w,  0xff00)
	AM_RANGE(0xffff62, 0xffff63) AM_DEVREADWRITE8("timer16",   h8_timer16_device,         tfcr_r,  tfcr_w,  0x00ff)
	AM_RANGE(0xffff64, 0xffff65) AM_DEVREADWRITE8("timer16:0", h8_timer16_channel_device, tcr_r,   tcr_w,   0xff00)
	AM_RANGE(0xffff64, 0xffff65) AM_DEVREADWRITE8("timer16:0", h8_timer16_channel_device, tior_r,  tior_w,  0x00ff)
	AM_RANGE(0xffff66, 0xffff67) AM_DEVREADWRITE8("timer16:0", h8_timer16_channel_device, tier_r,  tier_w,  0xff00)
	AM_RANGE(0xffff66, 0xffff67) AM_DEVREADWRITE8("timer16:0", h8_timer16_channel_device, tsr_r,   tsr_w,   0x00ff)
	AM_RANGE(0xffff68, 0xffff69) AM_DEVREADWRITE( "timer16:0", h8_timer16_channel_device, tcnt_r,  tcnt_w         )
	AM_RANGE(0xffff6a, 0xffff6d) AM_DEVREADWRITE( "timer16:0", h8_timer16_channel_device, tgr_r,   tgr_w          )
	AM_RANGE(0xffff6e, 0xffff6f) AM_DEVREADWRITE8("timer16:1", h8_timer16_channel_device, tcr_r,   tcr_w,   0xff00)
	AM_RANGE(0xffff6e, 0xffff6f) AM_DEVREADWRITE8("timer16:1", h8_timer16_channel_device, tior_r,  tior_w,  0x00ff)
	AM_RANGE(0xffff70, 0xffff71) AM_DEVREADWRITE8("timer16:1", h8_timer16_channel_device, tier_r,  tier_w,  0xff00)
	AM_RANGE(0xffff70, 0xffff71) AM_DEVREADWRITE8("timer16:1", h8_timer16_channel_device, tsr_r,   tsr_w,   0x00ff)
	AM_RANGE(0xffff72, 0xffff73) AM_DEVREADWRITE( "timer16:1", h8_timer16_channel_device, tcnt_r,  tcnt_w         )
	AM_RANGE(0xffff74, 0xffff77) AM_DEVREADWRITE( "timer16:1", h8_timer16_channel_device, tgr_r,   tgr_w          )
	AM_RANGE(0xffff78, 0xffff79) AM_DEVREADWRITE8("timer16:2", h8_timer16_channel_device, tcr_r,   tcr_w,   0xff00)
	AM_RANGE(0xffff78, 0xffff79) AM_DEVREADWRITE8("timer16:2", h8_timer16_channel_device, tior_r,  tior_w,  0x00ff)
	AM_RANGE(0xffff7a, 0xffff7b) AM_DEVREADWRITE8("timer16:2", h8_timer16_channel_device, tier_r,  tier_w,  0xff00)
	AM_RANGE(0xffff7a, 0xffff7b) AM_DEVREADWRITE8("timer16:2", h8_timer16_channel_device, tsr_r,   tsr_w,   0x00ff)
	AM_RANGE(0xffff7c, 0xffff7d) AM_DEVREADWRITE( "timer16:2", h8_timer16_channel_device, tcnt_r,  tcnt_w         )
	AM_RANGE(0xffff7e, 0xffff81) AM_DEVREADWRITE( "timer16:2", h8_timer16_channel_device, tgr_r,   tgr_w          )
	AM_RANGE(0xffff82, 0xffff83) AM_DEVREADWRITE8("timer16:3", h8_timer16_channel_device, tcr_r,   tcr_w,   0xff00)
	AM_RANGE(0xffff82, 0xffff83) AM_DEVREADWRITE8("timer16:3", h8_timer16_channel_device, tior_r,  tior_w,  0x00ff)
	AM_RANGE(0xffff84, 0xffff85) AM_DEVREADWRITE8("timer16:3", h8_timer16_channel_device, tier_r,  tier_w,  0xff00)
	AM_RANGE(0xffff84, 0xffff85) AM_DEVREADWRITE8("timer16:3", h8_timer16_channel_device, tsr_r,   tsr_w,   0x00ff)
	AM_RANGE(0xffff86, 0xffff87) AM_DEVREADWRITE( "timer16:3", h8_timer16_channel_device, tcnt_r,  tcnt_w         )
	AM_RANGE(0xffff88, 0xffff8b) AM_DEVREADWRITE( "timer16:3", h8_timer16_channel_device, tgr_r,   tgr_w          )
	AM_RANGE(0xffff8c, 0xffff8f) AM_DEVREADWRITE( "timer16:3", h8_timer16_channel_device, tbr_r,   tbr_w          )
	AM_RANGE(0xffff90, 0xffff91) AM_DEVREADWRITE8("timer16",   h8_timer16_device,         toer_r,  toer_w,  0xff00)
	AM_RANGE(0xffff90, 0xffff91) AM_DEVREADWRITE8("timer16",   h8_timer16_device,         tocr_r,  tocr_w,  0x00ff)
	AM_RANGE(0xffff92, 0xffff93) AM_DEVREADWRITE8("timer16:4", h8_timer16_channel_device, tcr_r,   tcr_w,   0xff00)
	AM_RANGE(0xffff92, 0xffff93) AM_DEVREADWRITE8("timer16:4", h8_timer16_channel_device, tior_r,  tior_w,  0x00ff)
	AM_RANGE(0xffff94, 0xffff95) AM_DEVREADWRITE8("timer16:4", h8_timer16_channel_device, tier_r,  tier_w,  0xff00)
	AM_RANGE(0xffff94, 0xffff95) AM_DEVREADWRITE8("timer16:4", h8_timer16_channel_device, tsr_r,   tsr_w,   0x00ff)
	AM_RANGE(0xffff96, 0xffff97) AM_DEVREADWRITE( "timer16:4", h8_timer16_channel_device, tcnt_r,  tcnt_w         )
	AM_RANGE(0xffff98, 0xffff9b) AM_DEVREADWRITE( "timer16:4", h8_timer16_channel_device, tgr_r,   tgr_w          )
	AM_RANGE(0xffff9c, 0xffff9f) AM_DEVREADWRITE( "timer16:4", h8_timer16_channel_device, tbr_r,   tbr_w          )

	AM_RANGE(0xffffa8, 0xffffa9) AM_DEVREADWRITE( "watchdog",  h8_watchdog_device,        wd_r,    wd_w           )
	AM_RANGE(0xffffaa, 0xffffab) AM_DEVREADWRITE( "watchdog",  h8_watchdog_device,        rst_r,   rst_w          )

	AM_RANGE(0xffffb0, 0xffffb1) AM_DEVREADWRITE8("sci0",      h8_sci_device,             smr_r,   smr_w,   0xff00)
	AM_RANGE(0xffffb0, 0xffffb1) AM_DEVREADWRITE8("sci0",      h8_sci_device,             brr_r,   brr_w,   0x00ff)
	AM_RANGE(0xffffb2, 0xffffb3) AM_DEVREADWRITE8("sci0",      h8_sci_device,             scr_r,   scr_w,   0xff00)
	AM_RANGE(0xffffb2, 0xffffb3) AM_DEVREADWRITE8("sci0",      h8_sci_device,             tdr_r,   tdr_w,   0x00ff)
	AM_RANGE(0xffffb4, 0xffffb5) AM_DEVREADWRITE8("sci0",      h8_sci_device,             ssr_r,   ssr_w,   0xff00)
	AM_RANGE(0xffffb4, 0xffffb5) AM_DEVREAD8(     "sci0",      h8_sci_device,             rdr_r,            0x00ff)
	AM_RANGE(0xffffb8, 0xffffb9) AM_DEVREADWRITE8("sci1",      h8_sci_device,             smr_r,   smr_w,   0xff00)
	AM_RANGE(0xffffb8, 0xffffb9) AM_DEVREADWRITE8("sci1",      h8_sci_device,             brr_r,   brr_w,   0x00ff)
	AM_RANGE(0xffffba, 0xffffbb) AM_DEVREADWRITE8("sci1",      h8_sci_device,             scr_r,   scr_w,   0xff00)
	AM_RANGE(0xffffba, 0xffffbb) AM_DEVREADWRITE8("sci1",      h8_sci_device,             tdr_r,   tdr_w,   0x00ff)
	AM_RANGE(0xffffbc, 0xffffbd) AM_DEVREADWRITE8("sci1",      h8_sci_device,             ssr_r,   ssr_w,   0xff00)
	AM_RANGE(0xffffbc, 0xffffbd) AM_DEVREAD8(     "sci1",      h8_sci_device,             rdr_r,            0x00ff)
	AM_RANGE(0xffffc0, 0xffffc1) AM_DEVWRITE8(    "port1",     h8_port_device,                     ddr_w,   0xff00)
	AM_RANGE(0xffffc0, 0xffffc1) AM_DEVWRITE8(    "port2",     h8_port_device,                     ddr_w,   0x00ff)
	AM_RANGE(0xffffc2, 0xffffc3) AM_DEVREADWRITE8("port1",     h8_port_device,            port_r,  dr_w,    0xff00)
	AM_RANGE(0xffffc2, 0xffffc3) AM_DEVREADWRITE8("port2",     h8_port_device,            port_r,  dr_w,    0x00ff)
	AM_RANGE(0xffffc4, 0xffffc5) AM_DEVWRITE8(    "port3",     h8_port_device,                     ddr_w,   0xff00)
	AM_RANGE(0xffffc4, 0xffffc5) AM_DEVWRITE8(    "port4",     h8_port_device,                     ddr_w,   0x00ff)
	AM_RANGE(0xffffc6, 0xffffc7) AM_DEVREADWRITE8("port3",     h8_port_device,            port_r,  dr_w,    0xff00)
	AM_RANGE(0xffffc6, 0xffffc7) AM_DEVREADWRITE8("port4",     h8_port_device,            port_r,  dr_w,    0x00ff)
	AM_RANGE(0xffffc8, 0xffffc9) AM_DEVWRITE8(    "port5",     h8_port_device,                     ddr_w,   0xff00)
	AM_RANGE(0xffffc8, 0xffffc9) AM_DEVWRITE8(    "port6",     h8_port_device,                     ddr_w,   0x00ff)
	AM_RANGE(0xffffca, 0xffffcb) AM_DEVREADWRITE8("port5",     h8_port_device,            port_r,  dr_w,    0xff00)
	AM_RANGE(0xffffca, 0xffffcb) AM_DEVREADWRITE8("port6",     h8_port_device,            port_r,  dr_w,    0x00ff)
	AM_RANGE(0xffffcc, 0xffffcd) AM_DEVWRITE8(    "port8",     h8_port_device,                     ddr_w,   0x00ff)
	AM_RANGE(0xffffce, 0xffffcf) AM_DEVREADWRITE8("port7",     h8_port_device,            port_r,  dr_w,    0xff00)
	AM_RANGE(0xffffce, 0xffffcf) AM_DEVREADWRITE8("port8",     h8_port_device,            port_r,  dr_w,    0x00ff)
	AM_RANGE(0xffffd0, 0xffffd1) AM_DEVWRITE8(    "port9",     h8_port_device,                     ddr_w,   0xff00)
	AM_RANGE(0xffffd0, 0xffffd1) AM_DEVWRITE8(    "porta",     h8_port_device,                     ddr_w,   0x00ff)
	AM_RANGE(0xffffd2, 0xffffd3) AM_DEVREADWRITE8("port9",     h8_port_device,            port_r,  dr_w,    0xff00)
	AM_RANGE(0xffffd2, 0xffffd3) AM_DEVREADWRITE8("porta",     h8_port_device,            port_r,  dr_w,    0x00ff)
	AM_RANGE(0xffffd4, 0xffffd5) AM_DEVWRITE8(    "portb",     h8_port_device,                     ddr_w,   0xff00)
	AM_RANGE(0xffffd6, 0xffffd7) AM_DEVREADWRITE8("portb",     h8_port_device,            port_r,  dr_w,    0xff00)
	AM_RANGE(0xffffd8, 0xffffd9) AM_DEVREADWRITE8("port2",     h8_port_device,            pcr_r,   pcr_w,   0xff00)
	AM_RANGE(0xffffda, 0xffffdb) AM_DEVREADWRITE8("port4",     h8_port_device,            pcr_r,   pcr_w,   0xff00)
	AM_RANGE(0xffffda, 0xffffdb) AM_DEVREADWRITE8("port5",     h8_port_device,            pcr_r,   pcr_w,   0x00ff)

	AM_RANGE(0xffffe0, 0xffffe7) AM_DEVREAD8(     "adc",       h8_adc_device,             addr8_r,          0xffff)
	AM_RANGE(0xffffe8, 0xffffe9) AM_DEVREADWRITE8("adc",       h8_adc_device,             adcsr_r, adcsr_w, 0xff00)
	AM_RANGE(0xffffe8, 0xffffe9) AM_DEVREADWRITE8("adc",       h8_adc_device,             adcr_r,  adcr_w,  0x00ff)

	AM_RANGE(0xfffff2, 0xfffff3) AM_READWRITE8(                                           syscr_r, syscr_w, 0xff00)

	AM_RANGE(0xfffff4, 0xfffff5) AM_DEVREADWRITE8("intc",      h8h_intc_device,           iscr_r,  iscr_w,  0xff00)
	AM_RANGE(0xfffff4, 0xfffff5) AM_DEVREADWRITE8("intc",      h8h_intc_device,           ier_r,   ier_w,   0x00ff)
	AM_RANGE(0xfffff6, 0xfffff7) AM_DEVREADWRITE8("intc",      h8h_intc_device,           isr_r,   isr_w,   0xff00)
	AM_RANGE(0xfffff8, 0xfffff9) AM_DEVREADWRITE8("intc",      h8h_intc_device,           icr_r,   icr_w,   0xffff)
ADDRESS_MAP_END

MACHINE_CONFIG_MEMBER(h83048_device::device_add_mconfig)
	MCFG_H8H_INTC_ADD("intc")
	MCFG_H8_ADC_3337_ADD("adc", "intc", 60)
	MCFG_H8_PORT_ADD("port1", h8_device::PORT_1, 0x00, 0x00)
	MCFG_H8_PORT_ADD("port2", h8_device::PORT_2, 0x00, 0x00)
	MCFG_H8_PORT_ADD("port3", h8_device::PORT_3, 0x00, 0x00)
	MCFG_H8_PORT_ADD("port4", h8_device::PORT_4, 0x00, 0x00)
	MCFG_H8_PORT_ADD("port5", h8_device::PORT_5, 0xf0, 0xf0)
	MCFG_H8_PORT_ADD("port6", h8_device::PORT_6, 0x80, 0x80)
	MCFG_H8_PORT_ADD("port7", h8_device::PORT_7, 0x00, 0x00)
	MCFG_H8_PORT_ADD("port8", h8_device::PORT_8, 0xe0, 0xe0)
	MCFG_H8_PORT_ADD("port9", h8_device::PORT_9, 0xc0, 0xc0)
	MCFG_H8_PORT_ADD("porta", h8_device::PORT_A, 0x00, 0x00)
	MCFG_H8_PORT_ADD("portb", h8_device::PORT_B, 0x00, 0x00)
	MCFG_H8_TIMER16_ADD("timer16", 5, 0xe0)
	MCFG_H8H_TIMER16_CHANNEL_ADD("timer16:0", 2, 2, "intc", 24)
	MCFG_H8H_TIMER16_CHANNEL_ADD("timer16:1", 2, 2, "intc", 28)
	MCFG_H8H_TIMER16_CHANNEL_ADD("timer16:2", 2, 2, "intc", 32)
	MCFG_H8H_TIMER16_CHANNEL_ADD("timer16:3", 2, 2, "intc", 36)
	MCFG_H8H_TIMER16_CHANNEL_ADD("timer16:4", 2, 2, "intc", 40)
	MCFG_H8_SCI_ADD("sci0", "intc", 52, 53, 54, 55)
	MCFG_H8_SCI_ADD("sci1", "intc", 56, 57, 58, 59)
	MCFG_H8_WATCHDOG_ADD("watchdog", "intc", 20, h8_watchdog_device::H)
MACHINE_CONFIG_END

void h83048_device::execute_set_input(int inputnum, int state)
{
	intc->set_input(inputnum, state);
}

int h83048_device::trapa_setup()
{
	if(syscr & 0x08)
		CCR |= F_I;
	else
		CCR |= F_I|F_UI;
	return 8;
}

void h83048_device::irq_setup()
{
	if(syscr & 0x08)
		CCR |= F_I;
	else
		CCR |= F_I|F_UI;
}

void h83048_device::update_irq_filter()
{
	switch(syscr & 0x08) {
	case 0x00:
		if((CCR & (F_I|F_UI)) == (F_I|F_UI))
			intc->set_filter(2, -1);
		else if(CCR & F_I)
			intc->set_filter(1, -1);
		else
			intc->set_filter(0, -1);
		break;
	case 0x08:
		if(CCR & F_I)
			intc->set_filter(2, -1);
		else
			intc->set_filter(0, -1);
		break;
	}
}

void h83048_device::interrupt_taken()
{
	standard_irq_callback(intc->interrupt_taken(taken_irq_vector));
}

void h83048_device::internal_update(uint64_t current_time)
{
	uint64_t event_time = 0;

	add_event(event_time, adc->internal_update(current_time));
	add_event(event_time, sci0->internal_update(current_time));
	add_event(event_time, sci1->internal_update(current_time));
	add_event(event_time, timer16_0->internal_update(current_time));
	add_event(event_time, timer16_1->internal_update(current_time));
	add_event(event_time, timer16_2->internal_update(current_time));
	add_event(event_time, timer16_3->internal_update(current_time));
	add_event(event_time, timer16_4->internal_update(current_time));
	add_event(event_time, watchdog->internal_update(current_time));

	recompute_bcount(event_time);
}

void h83048_device::device_start()
{
	h8h_device::device_start();
}

void h83048_device::device_reset()
{
	h8h_device::device_reset();
	syscr = 0x0b;
}

READ8_MEMBER(h83048_device::syscr_r)
{
	return syscr;
}

WRITE8_MEMBER(h83048_device::syscr_w)
{
	syscr = data;
	update_irq_filter();
	logerror("syscr = %02x\n", data);
}
